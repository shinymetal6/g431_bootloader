/*
 * bootloader.c
 *
 *  Created on: Jan 28, 2022
 *      Author: fil
 */

#include "main.h"
#include "bootloader.h"
#include "flash.h"
#include "uart.h"
#include "xmodem.h"
FLASH_EraseInitTypeDef erase_init =
{
	.TypeErase = FLASH_TYPEERASE_PAGES,
	.Page = FLASH_PAGES_NUMBER-1,
	.Banks = FLASH_TYPEERASE_PAGES,
	.NbPages = 1
};

uint32_t check_for_sw_reset(void)
{
uint32_t datalo, datahi;
uint32_t error = 0u,ret_val = HAL_ERROR;

    datahi = (*(volatile uint32_t*)FLASH_LASTPAGE_ADDRESS);
    datalo = (*(volatile uint32_t*)(FLASH_LASTPAGE_ADDRESS+4));
    if (( datahi == 0xdeadbeef ) && ( datalo == 0xbeefdead ))
    {
        HAL_FLASH_Unlock();
        ret_val = HAL_FLASHEx_Erase(&erase_init, &error);
        HAL_FLASH_Lock();
    }
    return ret_val;
}

void bootloader(void)
{
	if ( check_for_sw_reset() != 0 )
	{
		if (   HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0 )
			flash_jump_to_app();
	}
#ifdef VERBOSE
	uart_transmit_str((uint8_t*)"\n\r================================\n\r");
	uart_transmit_str((uint8_t*)"UART Bootloader\n\r");
	uart_transmit_str((uint8_t*)"================================\n\r\n\r");
#endif
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	while (1)
	{
#ifdef VERBOSE
		uart_transmit_str((uint8_t*)"Please send a new binary file with Xmodem protocol to update the firmware.\n\r");
#endif
		xmodem_receive();
		/* We only exit the xmodem protocol, if there are any errors.
		* In that case, notify the user and start over. */
		uart_transmit_str((uint8_t*)"\n\rFailed... Please try again.\n\r");
	}
}
