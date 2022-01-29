/*
 * Minimalistic implementation of the XModem/YModem protocol suite, including
 * a compact version of an CRC16 algorithm. The code is just enough to upload
 * an image to an MCU that bootstraps itself over an UART.
 *
 * Copyright (c) 2014 Daniel Mack <github@zonque.org>
 *
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#define X_STX 0x02
#define X_ACK 0x06
#define X_NAK 0x15
#define X_EOF 0x04

#define min(a, b)       ((a) < (b) ? (a) : (b))

struct xmodem_chunk {
        uint8_t start;
        uint8_t block;
        uint8_t block_neg;
        uint8_t payload[1024];
        uint16_t crc;
} __attribute__((packed));

#define CRC_POLY 0x1021

static uint16_t crc_update(uint16_t crc_in, int incr)
{
        uint16_t xor = crc_in >> 15;
        uint16_t out = crc_in << 1;

        if (incr)
                out++;

        if (xor)
                out ^= CRC_POLY;

        return out;
}

static uint16_t crc16(const uint8_t *data, uint16_t size)
{
        uint16_t crc, i;

        for (crc = 0; size > 0; size--, data++)
                for (i = 0x80; i; i >>= 1)
                        crc = crc_update(crc, *data & i);

        for (i = 0; i < 16; i++)
                crc = crc_update(crc, 0);

        return crc;
}
static uint16_t swap16(uint16_t in)
{
        return (in >> 8) | ((in & 0xff) << 8);
}

enum {
        PROTOCOL_XMODEM,
        PROTOCOL_YMODEM,
};

static int xymodem_send(int serial_fd, const char *filename, int protocol, int wait)
{
size_t len;
int ret, fd;
uint8_t answer;
struct stat stat;
const uint8_t *buf;
uint8_t eof = X_EOF;
struct xmodem_chunk chunk;
int skip_payload = 0 , retry = 0;

        fd = open(filename, O_RDONLY);
        if (fd < 0) {
                perror("open");
                return -errno;
        }

        fstat(fd, &stat);
        len = stat.st_size;
        buf = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);
        if (!buf) {
                perror("mmap");
                return -errno;
        }

        if (wait) {
                printf("Waiting for receiver ping ...");
                fflush(stdout);

                do {
                        ret = read(serial_fd, &answer, sizeof(answer));
                        if (ret != sizeof(answer)) {
                                perror("read");
                                return -errno;
                        }
                } while (answer != 'C');

        }

        printf("done.\nSending %s \n", filename);

        if (protocol == PROTOCOL_YMODEM) {
                strncpy((char *) chunk.payload, filename, sizeof(chunk.payload));
                chunk.block = 0;
                skip_payload = 1;
        } else {
                chunk.block = 1;
        }

        chunk.start = X_STX;

        while (len) {
                size_t z = 0;
                int next = 0;
                char status;

                if (!skip_payload) {
                        z = min(len, sizeof(chunk.payload));
                        memcpy(chunk.payload, buf, z);
                        memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);
                } else {
                        skip_payload = 0;
                }

                chunk.crc = swap16(crc16(chunk.payload, sizeof(chunk.payload)));
                chunk.block_neg = 0xff - chunk.block;

                ret = write(serial_fd, &chunk, sizeof(chunk));
                if (ret != sizeof(chunk))
                        return -errno;
                ret = read(serial_fd, &answer, sizeof(answer));
                if (ret != sizeof(answer))
                        return -errno;
                switch (answer) {
                case X_NAK:
                        status = 'N';
                        retry++;
                        break;
                case X_ACK:
                        status = '.';
                        next = 1;
                        break;
                default:
                        retry++;
                        status = '?';
                        break;
                }

                printf("%c", status);
                fflush(stdout);
                if ( retry > 5 )
                return -1;

                if (next) {
                        chunk.block++;
                        len -= z;
                        buf += z;
                }
        }

        ret = write(serial_fd, &eof, sizeof(eof));
        if (ret != sizeof(eof))
                return -errno;

        /* send EOT again for YMODEM */
        if (protocol == PROTOCOL_YMODEM) {
                ret = write(serial_fd, &eof, sizeof(eof));
                if (ret != sizeof(eof))
                        return -errno;
        }

        printf("\nDone.\n");

        return 0;
}

static int open_serial(const char *path, int baud)
{
int fd;
struct termios tty;

    fd = open(path, O_RDWR | O_SYNC);
    if (fd < 0)
    {
            perror("Error opening port");
            return -errno;
    }

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
            perror("tcgetattr");
            return -errno;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 1;                            // read doesn't block
    tty.c_cc[VTIME] = 50;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
            perror("tcsetattr");
            return -errno;
    }
    return fd;
}

int serial_open(char *port, int baud)
{
struct termios tty;
int serial_port;

    serial_port = open(port, O_RDWR);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s on port %s\n", errno, strerror(errno),port);
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 30;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be B115200
    cfsetispeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    return serial_port; // success
}

static void dump_serial(int serial_fd)
{
        char in;

        for (;;) {
                read(serial_fd, &in, sizeof(in));
                printf("%c", in);
                fflush(stdout);
        }
}

int main(int argc, char **argv)
{
        int ret, serial_fd;

        serial_fd = open_serial("/dev/ttyACM0", B115200);
        if (serial_fd < 0)
                return -errno;

        ret = xymodem_send(serial_fd, argv[1], PROTOCOL_XMODEM, 1);

        dump_serial(serial_fd);
        close(serial_fd);
        return ret;
}
