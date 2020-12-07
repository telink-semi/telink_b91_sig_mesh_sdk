/********************************************************************************************************
 * @file	flash.h
 *
 * @brief	This is the header file for B91
 *
 * @author	Z.W.H
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *          
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *          
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *          
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions 
 *              in binary form must reproduce the above copyright notice, this list of 
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *          
 *              3. Neither the name of TELINK, nor the names of its contributors may be 
 *              used to endorse or promote products derived from this software without 
 *              specific prior written permission.
 *          
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or 
 *              relating to such deletion(s), modification(s) or alteration(s).
 *         
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *         
 *******************************************************************************************************/
#pragma once

#include "mspi.h"
#include "compiler.h"
#include "../../common/types.h"

#define PAGE_SIZE 256

/**
 * @brief     flash command definition
 */
typedef enum
{
	FLASH_WRITE_STATUS_CMD		=	0x01,
	FLASH_WRITE_CMD				=	0x02,
	FLASH_READ_CMD				=	0x03,

	FLASH_WRITE_DISABLE_CMD 	= 	0x04,
	FLASH_READ_STATUS_CMD		=	0x05,
	FLASH_WRITE_ENABLE_CMD 		= 	0x06,

	FLASH_CHIP_ERASE_CMD		=	0x60,   //or 0xc7

	FLASH_PES_CMD				=	0x75,
	FLASH_PER_CMD				=	0x7A,
	FLASH_QUAD_PAGE_PROGRAM_CMD	=	0x32,
	FLASH_READ_DEVICE_ID_CMD	=	0x90,

	FLASH_FAST_READ_CMD			=	0x0B,
	FLASH_X2READ_CMD			=	0xBB,
	FLASH_DREAD_CMD				=	0x3B,
	FLASH_X4READ_CMD			=	0xEB,
	FLASH_QREAD_CMD				=	0x6B,

	FLASH_SECT_ERASE_CMD		=	0x20,   //sector size = 4KBytes
	FLASH_32KBLK_ERASE_CMD		=	0x52,
	FLASH_64KBLK_ERASE_CMD		=	0xD8,
	FLASH_GD_PUYA_READ_UID_CMD	=	0x4B,	//Flash Type = GD/PUYA
	FLASH_XTX_READ_UID_CMD		=	0x5A,	//Flash Type = XTX
	FLASH_PAGE_ERASE_CMD		=	0x81,   //caution: only P25Q40L support this function

	FLASH_POWER_DOWN			=	0xB9,
	FLASH_POWER_DOWN_RELEASE	=	0xAB,
	FLASH_GET_JEDEC_ID			=	0x9F,
	FLASH_READ_STATUS_1_CMD		=	0x35,

	FLASH_VOLATILE_SR_WRITE_CMD	=	0x50,
	FLASH_SET_BURST_WITH_WRAP_CMD	=	0x77,
	FLASH_ENABLE_SO_TO_OUTPUT_CMD	=	0x70,
	FLASH_READ_DEVICE_ID_DUAL_CME	=	0x92,
	RLASH_READ_DEVICE_ID_QUAD_CMD	=	0x94,
	FLASH_ERASE_SECURITY_REGISTERS_CMD	=	0x44,
	FLASH_PROGRAM_SECURITY_REGISTERS_CMD	=	0x42,
	FLASH_READ_SECURITY_REGISTERS_CMD	=	0x48,
	FLASH_ENABLE_RESET_CMD	=	0x99,

	FLASH_ENABLE_RESET	=	0x66,
	FLASH_DISABLE_SO_TO_OUTPUT	=	0x80,
}flash_command_e;

/**
 * @brief     flash type definition
 */
typedef enum{
	FLASH_TYPE_PUYA	= 0,
}flash_type_e;

void flash_en_support_arch_flash(unsigned char en);

/**
 * @brief     	This function serves to Stop XIP operation before flash.
 * @param[in] 	none.
 * @return    none.
 */
_attribute_text_sec_ void flash_stop_xip(void);
/**
 * @brief     	This function serves to erase a page(256 bytes).
 * @param[in] 	addr - the start address of the page needs to erase.
 * @return    	none
 * @note      	only 8359 support
 */
_attribute_text_sec_ void flash_erase_page(unsigned int addr);

/**
 * @brief 		This function serves to erase a sector.
 * @param[in]   addr the start address of the sector needs to erase.
 * @return 		none
 */
_attribute_text_sec_ void flash_erase_sector(unsigned long addr);

/**
 * @brief 		This function serves to erase a block(32k).
 * @param[in]   addr the start address of the block needs to erase.
 * @return 		none
 */
_attribute_text_sec_ void flash_erase_32kblock(unsigned int addr);

/**
 * @brief 		This function serves to erase a block(64k).
 * @param[in]   addr the start address of the block needs to erase.
 * @return 		none
 */
_attribute_text_sec_ void flash_erase_64kblock(unsigned int addr);

/**
 * @brief     	This function serves to erase a page(256 bytes).
 * @param[in] 	addr - the start address of the page needs to erase.
 * @return    	none
 */
_attribute_text_sec_ void flash_erase_chip(void);

/**
 * @brief 		This function writes the buffer's content to a page.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to write into the page
 * @param[in]   buf the start address of the content needs to write into
 * @return 		none
 */
_attribute_text_sec_ void flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf);

/**
 * @brief 		This function reads the content from a page to the buf.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to read out from the page
 * @param[out]  buf the start address of the buffer
 * @return 		none
 */
_attribute_text_sec_ void flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf);

/**
 * @brief 		This function write the status of flash.
 * @param[in]  	the value of status
 * @return 		status
 */
_attribute_text_sec_ void flash_write_status(unsigned short data);

/**
 * @brief 		This function reads the status of flash.
 * @param[in]  	none
 * @return 		none
 */
_attribute_text_sec_ unsigned short flash_read_status(void);

/**
 * @brief  	Deep Power Down mode to put the device in the lowest consumption mode
 * 			it can be used as an extra software protection mechanism,while the device
 * 			is not in active use,since in the mode,  all write,Program and Erase commands
 * 			are ignored,except the Release from Deep Power-Down and Read Device ID(RDI)
 * 			command.This release the device from this mode
 * @param[in] none
 * @return none.
 */
_attribute_text_sec_ void flash_deep_powerdown(void);

/**
 * @brief		The Release from Power-Down or High Performance Mode/Device ID command is a
 * 				Multi-purpose command.it can be used to release the device from the power-Down
 * 				State or High Performance Mode or obtain the devices electronic identification
 * 				(ID)number.Release from Power-Down will take the time duration of tRES1 before
 * 				the device will resume normal operation and other command are accepted.The CS#
 * 				pin must remain high during the tRES1(8us) time duration.
 * @param[in]   none
 * @return      none.
 */
_attribute_text_sec_ void flash_release_deep_powerdown(void);

/***********************************
 * @brief	  This function serves to read MID of flash
 * @param[in] buf - store MID of flash
 * @return    none.
 */
_attribute_text_sec_ void flash_read_mid(unsigned char *buf);

/**
 * @brief	  This function serves to read UID of flash
 * @param[in] idcmd - different flash vendor have different read-uid command
 *                    GD/PUYA:0x4B; XTX: 0x5A
 * @param[in] buf   - store UID of flash
 * @return    none.
 */
_attribute_text_sec_ void flash_read_uid(unsigned char idcmd,unsigned char *buf);

/**
 * @brief This function serves to protect data for flash.
 * @param[in]   type - flash type include GD,Puya and XTX
 * @param[in]   data - refer to Driver API Doc.
 * @return none
 */
_attribute_text_sec_ void flash_lock(flash_type_e type , unsigned short data);

/**
 * @brief This function serves to protect data for flash.
 * @param[in]   type - flash type include GD,Puya and XTX
 * @return none
 */
_attribute_text_sec_ void flash_unlock(flash_type_e type);
/**
 * @brief 		This function serves to set   priority threshold.when the  interrupt priority>Threshold flash process will disturb by interrupt.
 * @param[in]   preempt_en -1 can disturb by interrupt, 0 can disturb by interrupt.
 * @param[in]	threshold -priority Threshold.
 * @return    	none
 */
_attribute_text_sec_ void flash_plic_preempt_config(unsigned char preempt_en,unsigned char threshold);

/*******************************      BLE Stack Use     ******************************/
_attribute_text_code_ unsigned int flash_get_jedec_id(void);

/** \defgroup GP4  Flash Examples
 *
 * 	@{
 */

/*! \page flash Table of Contents
	- [API-FLASH-CASE1:FLASH READ/WRITE PAGE](#FLASH_RW)
	- [API-FLASH-CASE2:FLASH ERASE SECTOR](#FLASH_ERASE)
	- [API-FLASH-CASE3:FLASH READ/WRITE STATUS](#FLASH_STATUS)
	- [API-FLASH-CASE4:FLASH DEEP POWER DOWN/RELEASE](#FLASH_POWERDOWN)
	- [API-FLASH-CASE5:FLASH READ MID/UID](#FLASH_MUID)
	- [API-FLASH-CASE6:FLASH LOCK](#FLASH_LOCK)
	- [API-FLASH-CASE7:FLASH UNLOCK](#FLASH_UNLOCK)

\n
Variables used in the following cases are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define FLASH_ADDR				0x020000
#define FLASH_BUFF_LEN			256

volatile unsigned char Flash_Read_Buff[FLASH_BUFF_LEN]={0};
volatile unsigned char Flash_Write_Buff[FLASH_BUFF_LEN]=
{		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,


		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,

		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,

		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
		0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
		0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,
		0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,
		0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,
		0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
		0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
};

volatile unsigned char status;
unsigned char mid_buf[4]={0};
unsigned char uid_buf[16]={0};
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=FLASH_RW> API-FLASH-CASE1:FLASH READ/WRITE PAGE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_write_page() | flash_write_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Write_Buff) | write 256 bytes to the specified address of flash | ^ |
| ^ | ^ | flash_read_page() | flash_read_page(FLASH_ADDR,FLASH_BUFF_LEN,(unsigned char *)Flash_Read_Buff) | read 256 bytes from the specified address of flash to check whether the written data is right | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_ERASE> API-FLASH-CASE2:FLASH ERASE SECTOR </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_erase_sector() |  flash_erase_sector(FLASH_ADDR) | erase a sector from the specified address of flash | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_STATUS> API-FLASH-CASE3:FLASH READ/WRITE STATUS </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | status = flash_read_status() || read the status of flash | ^ |
| ^ | ^ | flash_write_status() | status = flash_write_status(0x04) | write the status of flash | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_POWERDOWN> API-FLASH-CASE4:FLASH DEEP POWER DOWN/RELEASE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_deep_powerdown() || put the device in the lowest consumption mode | ^ |
| ^ | ^ | flash_release_deep_powerdown() || wake up the device from the lowest consumption mode | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_MUID> API-FLASH-CASE5:FLASH READ MID/UID </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_read_mid() | flash_read_mid(mid_buf) | read MID of flash | ^ |
| ^ | ^ | flash_read_uid() | flash_read_uid(FLASH_READ_UID_CMD,uid_buf) | read UID of flash | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_LOCK> API-FLASH-CASE6:FLASH LOCK </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_lock() | flash_lock(FLASH_TYPE_GD, 0x1c) | lock first 512k bytes of flash | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=FLASH_UNLOCK> API-FLASH-CASE6:FLASH UNLOCK </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | flash_unlock() | flash_unlock(FLASH_TYPE_GD) | unlock the whole flash | ^ |
| ^ | main_loop() | None || Main program loop | ^ |


<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP4
