/********************************************************************************************************
 * @file	app_config.h
 *
 * @brief	for TLSR chips
 *
 * @author	BLE GROUP
 * @date	2020.06
 *
 * @par     Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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



////////////////////////// FEATURE SELECT //////////////////////////////////////
#define BLE_APP_PM_ENABLE					0
#define PM_NO_SUSPEND_ENABLE				1
#define PM_DEEPSLEEP_RETENTION_ENABLE		0
#define TEST_CONN_CURRENT_ENABLE            0 //test connection current, disable UI to have a pure power
#define BLE_REMOTE_SECURITY_ENABLE      	0
#define REMOTE_IR_ENABLE					0
#define BLE_AUDIO_ENABLE					0
#define BLE_REMOTE_OTA_ENABLE				1

#define	BLT_APP_LED_ENABLE					1
#define UI_LED_ENABLE          				1
#define LED_ON_LEVAL          				1

////////////////////////// BOARD SELECT ////////////////////////////////////////
#define EVK_BOARD							0    			//Ver:C1T213A20_V1_1
#define REMOTE_BOARD						1    			//Ver:C1T216A5_V3_0, DP for SW

#define	BOARD_SELECT						EVK_BOARD

////////////////////////// AUDIO CONFIG (RCU board) /////////////////////////////
#if (BLE_AUDIO_ENABLE)
	/* RCU Audio MODE:
	 * TL_AUDIO_RCU_ADPCM_GATT_TLEINK
	 */
	#define TL_AUDIO_MODE  			TL_AUDIO_RCU_ADPCM_GATT_TLEINK

	#define BLE_DMIC_ENABLE						0
	//DMIC for EVK_BOARD DDATA-->PB2   DCLK1-->PB3
	#if	BLE_DMIC_ENABLE
		#define PB2_FUNC						AS_DMIC_DI
		#define PB3_FUNC						AS_DMIC_SCK

		#define PB2_INPUT_ENABLE				1
		#define PB3_INPUT_ENABLE				1
	#endif

#endif


#if (BLE_REMOTE_OTA_ENABLE)
	#define OTA_VERSION_FLASH_ADDR			0xEF000
	#define OTA_VERSION_NUMBER				0x01
#endif



#define			CR_VOL_UP				0xf0  ////
#define			CR_VOL_DN				0xf1
#define			CR_VOL_MUTE				0xf2
#define			CR_CHN_UP				0xf3
#define			CR_CHN_DN				0xf4  ////
#define			CR_POWER				0xf5
#define			CR_SEARCH				0xf6
#define			CR_ENTER				0xf7
#define			CR_PLAY					0xf8  ////
#define			CR_PAUSE				0xf9
#define			CR_STOP					0xfa
#define			CR_FAST_BACKWARD		0xfb
#define			CR_FAST_FORWARD			0xfc  ////
#define			CR_HOME					0xfd
#define			CR_BACK					0xfe
#define			CR_MENU					0xff

//special key
#define		 	VOICE					0xc0
#define 		KEY_MODE_SWITCH			0xc1
#define		 	PHY_TEST				0xc2


#define 		IR_VK_0			0x00
#define 		IR_VK_1			0x01
#define 		IR_VK_2			0x02
#define			IR_VK_3			0x03
#define			IR_VK_4			0x04
#define 		IR_VK_5			0x05
#define 		IR_VK_6			0x06
#define 		IR_VK_7			0x07
#define 		IR_VK_8			0x08
#define 		IR_VK_9			0x09

#define 		IR_POWER		0x12
#define			IR_AUDIO_MUTE	0x0d
#define 		IR_NETFLIX		0x0f
#define			IR_BACK			0x0e
#define			IR_VOL_UP		0x0b
#define			IR_VOL_DN		0x0c
#define 		IR_NEXT			0x20
#define 		IR_PREV			0x21
#define			IR_MENU			0x23
#define 		IR_HOME			0x24
#define 		IR_OPER_KEY		0x2e
#define 		IR_INFO			0x2f
#define			IR_REWIND		0x32
#define 		IR_FAST_FOWARD	0x34
#define 		IR_PLAY_PAUSE	0x35
#define			IR_GUIDE		0x41
#define 		IR_UP			0x45
#define			IR_DN			0x44
#define 		IR_LEFT			0x42
#define 		IR_RIGHT		0x43
#define			IR_SEL			0x46
#define 		IR_RED_KEY		0x6b
#define 		IR_GREEN_KEY	0x6c
#define 		IR_YELLOW_KEY	0x6d
#define 		IR_BLUE_KEY		0x6e
#define 		IR_RECORD		0x72
#define 		IR_OPTION		0x73
#define 		IR_STOP			0x74
#define 		IR_SEARCH		0x75
#define 		IR_TEXT			0x76
#define 		IR_VOICE		0x77
#define 		IR_PAUSE		0x78

#define			T_VK_CH_UP		0xd0
#define			T_VK_CH_DN		0xd1

#if (BOARD_SELECT == EVK_BOARD)

	#if (TEST_CONN_CURRENT_ENABLE) //test current, disable keyboard
			#define	UI_KEYBOARD_ENABLE						0
	#else
			#define	UI_KEYBOARD_ENABLE						0
	#endif

	#if (UI_KEYBOARD_ENABLE)   // if test pure power, kyeScan GPIO setting all disabled
			//---------------  KeyMatrix PB2/PB3/PB4/PB5 -----------------------------
			#define	MATRIX_ROW_PULL					PM_PIN_PULLDOWN_100K
			#define	MATRIX_COL_PULL					PM_PIN_PULLUP_10K

			#define	KB_LINE_HIGH_VALID				0   //dirve pin output 0 when keyscan, scanpin read 0 is valid

			#define			CR_VOL_UP				0xf0  ////
			#define			CR_VOL_DN				0xf1
			#define			VOICE					0xc0

			#define		KB_MAP_NORMAL	{	CR_VOL_UP,		VK_1,	 \
											CR_VOL_DN,		VOICE }

	#if REMOTE_IR_ENABLE
			#define			IR_VOL_UP		0x0b
			#define 		IR_VK_1			0x01
			#define			IR_VOL_DN		0x0c
			#define 		IR_HOME			0x24
	#endif
			//////////////////// KEY CONFIG (EVK board) ///////////////////////////
			#define  KB_DRIVE_PINS  {GPIO_PC2, GPIO_PC0}
			#define  KB_SCAN_PINS   {GPIO_PC3, GPIO_PC1}

			//drive pin as gpio
			#define	PC2_FUNC				AS_GPIO
			#define	PC0_FUNC				AS_GPIO

			//drive pin need 100K pulldown
			#define	PULL_WAKEUP_SRC_PC2		MATRIX_ROW_PULL
			#define	PULL_WAKEUP_SRC_PC0		MATRIX_ROW_PULL

			//drive pin open input to read gpio wakeup level
			#define PC2_INPUT_ENABLE		1
			#define PC0_INPUT_ENABLE		1

			//scan pin as gpio
			#define	PC3_FUNC				AS_GPIO
			#define	PC1_FUNC				AS_GPIO

			//scan  pin need 10K pullup
			#define	PULL_WAKEUP_SRC_PC3		MATRIX_COL_PULL
			#define	PULL_WAKEUP_SRC_PC1		MATRIX_COL_PULL

			//scan pin open input to read gpio level
			#define PC3_INPUT_ENABLE		1
			#define PC1_INPUT_ENABLE		1

			#define		KB_MAP_NUM		KB_MAP_NORMAL
			#define		KB_MAP_FN		KB_MAP_NORMAL
	#endif
	#if UI_LED_ENABLE
			#define	GPIO_LED_WHITE			GPIO_PB6
			#define	GPIO_LED_GREEN			GPIO_PB5
			#define	GPIO_LED_BLUE			GPIO_PB4
			#define GPIO_LED_RED			GPIO_PB7

			#define LED_ON_LEVAL 			1 		//gpio output high voltage to turn on led

			#define PB7_FUNC				AS_GPIO
			#define PB6_FUNC				AS_GPIO
			#define PB5_FUNC				AS_GPIO
			#define PB4_FUNC				AS_GPIO

			#define	PB7_OUTPUT_ENABLE		1
			#define	PB6_OUTPUT_ENABLE		1
			#define PB5_OUTPUT_ENABLE		1
			#define	PB4_OUTPUT_ENABLE		1
	#endif

#elif(BOARD_SELECT == REMOTE_BOARD)


/////////IF SELECT REMOTE BOARD THIS IS MUST FOR DP AS SWIRE TO DOWNLOAD FW !!!//////////
	#define PA5_FUNC						AS_USB_DM
	#define PA6_FUNC						AS_USB_DP
	#define PA5_INPUT_ENABLE				1
	#define PA6_INPUT_ENABLE				1

	#define	KB_LINE_HIGH_VALID				0   //dirve pin output 0 when keyscan, scanpin read 0 is valid

	#if (TEST_CONN_CURRENT_ENABLE) //test current, disable keyboard
			#define	UI_KEYBOARD_ENABLE						0
	#else
			#define	UI_KEYBOARD_ENABLE						1
	#endif

	#if (UI_KEYBOARD_ENABLE)   // if test pure power, kyeScan GPIO setting all disabled
		//---------------  KeyMatrix PB2/PB3/PB4/PB5 -----------------------------
		#define		MATRIX_ROW_PULL					PM_PIN_PULLDOWN_100K
		#define		MATRIX_COL_PULL					PM_PIN_PULLUP_10K
		#if (REMOTE_IR_ENABLE)  //with IR keymap
			#define 	GPIO_IR_CONTROL					GPIO_PD0

			#define		KB_MAP_NORMAL	{\
							{0, 	     1,				    2,			3,		4,		}, \
							{VOICE,		 KEY_MODE_SWITCH,	7,			8,		9	    }, \
							{10,		 11,				12,			13,		14,		}, \
							{15,		 16,				17,			18,		19,		}, \
							{20,		 21,				22,			23,		24,		}, \
							{25,		 26,				27,			28,		29,		}, }

			#define		KB_MAP_BLE	{\
							VK_B,		CR_POWER,		VK_NONE,		VK_C,				CR_HOME,	 \
							VOICE,		VK_NONE,		VK_NONE,		CR_VOL_UP,			CR_VOL_DN,	 \
							VK_2,		VK_RIGHT,		CR_VOL_DN,		VK_3,				VK_1,	 \
							VK_5,		VK_ENTER,		CR_VOL_UP,		VK_6,				VK_4,	 \
							VK_8,		VK_DOWN,		VK_UP ,			VK_9,				VK_7,	 \
							VK_0,		CR_BACK,		VK_LEFT,		CR_VOL_MUTE,		CR_MENU,	 }

			#define		KB_MAP_IR	{\
							VK_NONE,	VK_NONE,		VK_NONE,	VK_NONE,			VK_NONE,	 \
							VK_NONE,	VK_NONE,		VK_NONE,	VK_NONE,			VK_NONE,	 \
							IR_VK_2,	VK_NONE,		VK_NONE,	IR_VK_3,			IR_VK_1, 	 \
							IR_VK_5,	VK_NONE,		VK_NONE,	IR_VK_6,			IR_VK_4,	 \
							IR_VK_8,	VK_NONE,		VK_NONE,	IR_VK_9,			IR_VK_7,	 \
							IR_VK_0,	VK_NONE,		VK_NONE,	VK_NONE,			VK_NONE,	 }
		#else   //key map
			#define		KB_MAP_NORMAL {\
							VK_B,		CR_POWER,		VK_NONE,		VK_C,				CR_HOME,	 \
							VOICE,	VK_NONE,		VK_NONE,		CR_VOL_UP,			CR_VOL_DN,	 \
							VK_2,		VK_RIGHT,		CR_VOL_DN,		VK_3,				VK_1,	 \
							VK_5,		VK_ENTER,		CR_VOL_UP,		VK_6,				VK_4,	 \
							VK_8,		VK_DOWN,		VK_UP ,			VK_9,				VK_7,	 \
							VK_0,		CR_BACK,		VK_LEFT,		CR_VOL_MUTE,		CR_MENU,	 }

		#endif  //end of REMOTE_IR_ENABLE
	#endif//end of UI_KEYBOARD_ENABLE
//////////////////// KEY CONFIG (RCU board) ///////////////////////////
		#define KB_DRIVE_PINS  	{GPIO_PE2, GPIO_PB4, GPIO_PB5, GPIO_PE1, GPIO_PE4}
		#define KB_SCAN_PINS   	{GPIO_PB1, GPIO_PB0, GPIO_PA4, GPIO_PA0, GPIO_PE6, GPIO_PE5}
		//drive pin as gpio
		#define	PE2_FUNC				AS_GPIO
		#define	PB4_FUNC				AS_GPIO
		#define	PB5_FUNC				AS_GPIO
		#define	PE1_FUNC				AS_GPIO
		#define	PE4_FUNC				AS_GPIO
		//drive pin need 100K pulldown
		#define	PULL_WAKEUP_SRC_PE2		MATRIX_ROW_PULL
		#define	PULL_WAKEUP_SRC_PB4		MATRIX_ROW_PULL
		#define	PULL_WAKEUP_SRC_PB5		MATRIX_ROW_PULL
		#define	PULL_WAKEUP_SRC_PE1		MATRIX_ROW_PULL
		#define	PULL_WAKEUP_SRC_PE4		MATRIX_ROW_PULL
		//drive pin open input to read gpio wakeup level
		#define PE2_INPUT_ENABLE		1
		#define PB4_INPUT_ENABLE		1
		#define PB5_INPUT_ENABLE		1
		#define PE1_INPUT_ENABLE		1
		#define PE4_INPUT_ENABLE		1
		//scan pin as gpio
		#define	PB1_FUNC				AS_GPIO
		#define	PB0_FUNC				AS_GPIO
		#define	PA4_FUNC				AS_GPIO
		#define	PA0_FUNC				AS_GPIO
		#define	PE6_FUNC				AS_GPIO
		#define	PE5_FUNC				AS_GPIO
		//scan  pin need 10K pullup
		#define	PULL_WAKEUP_SRC_PB1		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PB0		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PA4		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PA0		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PE6		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PE5		MATRIX_COL_PULL
		//scan pin open input to read gpio level
		#define PB1_INPUT_ENABLE		1
		#define PB0_INPUT_ENABLE		1
		#define PA4_INPUT_ENABLE		1
		#define PA0_INPUT_ENABLE		1
		#define PE6_INPUT_ENABLE		1
		#define PE5_INPUT_ENABLE		1

		#define	KB_MAP_NUM		KB_MAP_NORMAL
		#define	KB_MAP_FN		KB_MAP_NORMAL

	#if BLT_APP_LED_ENABLE
		#define LED_ON_LEVAL 						1 			//gpio output high voltage to turn on led
		#define	GPIO_LED_RED						GPIO_PB6
		#define	GPIO_LED_GREEN						GPIO_PC3
	#endif

#endif

#ifndef	UI_KEYBOARD_ENABLE
#define UI_KEYBOARD_ENABLE									0
#endif

#ifndef	UI_BUTTON_ENABLE
#define UI_BUTTON_ENABLE									0
#endif

/////////////////// DEEP SAVE FLG //////////////////////////////////
#define USED_DEEP_ANA_REG                   DEEP_ANA_REG1 //u8,can save 8 bit info when deep
#define CONN_DEEP_FLG	                    BIT(0) //if 1: conn deep, 0: adv deep

/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ  	16000000

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};



/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms





#define DEBUG_GPIO_ENABLE							0

#if(DEBUG_GPIO_ENABLE)


	#define GPIO_CHN0							GPIO_PE1
	#define GPIO_CHN1							GPIO_PE2
//	#define GPIO_CHN2							GPIO_PA0
    #define GPIO_CHN3							GPIO_PA4
	#define GPIO_CHN4							GPIO_PA3
	#define GPIO_CHN5							GPIO_PB0
//	#define GPIO_CHN6							GPIO_PB2
	#define GPIO_CHN7							GPIO_PE0

	#define GPIO_CHN8							GPIO_PA2
	#define GPIO_CHN9							GPIO_PA1
	#define GPIO_CHN10							GPIO_PB1
//	#define GPIO_CHN11							GPIO_PB3



	#define PE1_OUTPUT_ENABLE					1
	#define PE2_OUTPUT_ENABLE					1
//	#define PA0_OUTPUT_ENABLE					1
	#define PA4_OUTPUT_ENABLE					1
	#define PA3_OUTPUT_ENABLE					1
	#define PB0_OUTPUT_ENABLE					1
//	#define PB2_OUTPUT_ENABLE					1
	#define PE0_OUTPUT_ENABLE					1

	#define PA2_OUTPUT_ENABLE					1
	#define PA1_OUTPUT_ENABLE					1
	#define PB1_OUTPUT_ENABLE					1
//	#define PB3_OUTPUT_ENABLE					1


	#define DBG_CHN0_LOW		gpio_write(GPIO_CHN0, 0)
	#define DBG_CHN0_HIGH		gpio_write(GPIO_CHN0, 1)
	#define DBG_CHN0_TOGGLE		gpio_toggle(GPIO_CHN0)
	#define DBG_CHN1_LOW		gpio_write(GPIO_CHN1, 0)
	#define DBG_CHN1_HIGH		gpio_write(GPIO_CHN1, 1)
	#define DBG_CHN1_TOGGLE		gpio_toggle(GPIO_CHN1)
	#define DBG_CHN2_LOW		//gpio_write(GPIO_CHN2, 0)
	#define DBG_CHN2_HIGH		//gpio_write(GPIO_CHN2, 1)
	#define DBG_CHN2_TOGGLE		//gpio_toggle(GPIO_CHN2)
	#define DBG_CHN3_LOW		gpio_write(GPIO_CHN3, 0)
	#define DBG_CHN3_HIGH		gpio_write(GPIO_CHN3, 1)
	#define DBG_CHN3_TOGGLE		gpio_toggle(GPIO_CHN3)
	#define DBG_CHN4_LOW		gpio_write(GPIO_CHN4, 0)
	#define DBG_CHN4_HIGH		gpio_write(GPIO_CHN4, 1)
	#define DBG_CHN4_TOGGLE		gpio_toggle(GPIO_CHN4)
	#define DBG_CHN5_LOW		gpio_write(GPIO_CHN5, 0)
	#define DBG_CHN5_HIGH		gpio_write(GPIO_CHN5, 1)
	#define DBG_CHN5_TOGGLE		gpio_toggle(GPIO_CHN5)
	#define DBG_CHN6_LOW		//gpio_write(GPIO_CHN6, 0)
	#define DBG_CHN6_HIGH		//gpio_write(GPIO_CHN6, 1)
	#define DBG_CHN6_TOGGLE		//gpio_toggle(GPIO_CHN6)
	#define DBG_CHN7_LOW		gpio_write(GPIO_CHN7, 0)
	#define DBG_CHN7_HIGH		gpio_write(GPIO_CHN7, 1)
	#define DBG_CHN7_TOGGLE		gpio_toggle(GPIO_CHN7)
	#define DBG_CHN8_LOW		gpio_write(GPIO_CHN8, 0)
	#define DBG_CHN8_HIGH		gpio_write(GPIO_CHN8, 1)
	#define DBG_CHN8_TOGGLE		gpio_toggle(GPIO_CHN8)
	#define DBG_CHN9_LOW		gpio_write(GPIO_CHN9, 0)
	#define DBG_CHN9_HIGH		gpio_write(GPIO_CHN9, 1)
	#define DBG_CHN9_TOGGLE		gpio_toggle(GPIO_CHN9)
	#define DBG_CHN10_LOW		gpio_write(GPIO_CHN10, 0)
	#define DBG_CHN10_HIGH		gpio_write(GPIO_CHN10, 1)
	#define DBG_CHN10_TOGGLE	gpio_toggle(GPIO_CHN10)
	#define DBG_CHN11_LOW		//gpio_write(GPIO_CHN11, 0)
	#define DBG_CHN11_HIGH		//gpio_write(GPIO_CHN11, 1)
	#define DBG_CHN11_TOGGLE	//gpio_toggle(GPIO_CHN11)
#else
	#define DBG_CHN0_LOW
	#define DBG_CHN0_HIGH
	#define DBG_CHN0_TOGGLE
	#define DBG_CHN1_LOW
	#define DBG_CHN1_HIGH
	#define DBG_CHN1_TOGGLE
	#define DBG_CHN2_LOW
	#define DBG_CHN2_HIGH
	#define DBG_CHN2_TOGGLE
	#define DBG_CHN3_LOW
	#define DBG_CHN3_HIGH
	#define DBG_CHN3_TOGGLE
	#define DBG_CHN4_LOW
	#define DBG_CHN4_HIGH
	#define DBG_CHN4_TOGGLE
	#define DBG_CHN5_LOW
	#define DBG_CHN5_HIGH
	#define DBG_CHN5_TOGGLE
	#define DBG_CHN6_LOW
	#define DBG_CHN6_HIGH
	#define DBG_CHN6_TOGGLE
	#define DBG_CHN7_LOW
	#define DBG_CHN7_HIGH
	#define DBG_CHN7_TOGGLE
	#define DBG_CHN8_LOW
	#define DBG_CHN8_HIGH
	#define DBG_CHN8_TOGGLE
	#define DBG_CHN9_LOW
	#define DBG_CHN9_HIGH
	#define DBG_CHN9_TOGGLE
	#define DBG_CHN10_LOW
	#define DBG_CHN10_HIGH
	#define DBG_CHN10_TOGGLE
	#define DBG_CHN11_LOW
	#define DBG_CHN11_HIGH
	#define DBG_CHN11_TOGGLE
#endif  //end of DEBUG_GPIO_ENABLE



#include "../common/default_config.h"
