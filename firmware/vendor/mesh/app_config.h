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

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include "vendor/common/version.h"    // include mesh_config.h inside.

#define BLE_MODULE_PM_ENABLE				0
#define PM_NO_SUSPEND_ENABLE				1
#define PM_DEEPSLEEP_RETENTION_ENABLE		0
#define TEST_CONN_CURRENT_ENABLE            0 //test connection current, disable UI to have a pure power
#define BLE_OTA_ENABLE						1
#define TELIK_SPP_SERVICE_ENABLE			0
#define BLE_MODULE_INDICATE_DATA_TO_MCU		0
#define BATT_CHECK_ENABLE       			0   //enable or disable battery voltage detection
#define BLT_APP_LED_ENABLE					0

#define	BLT_APP_LED_ENABLE					1
#define UI_LED_ENABLE          				1
#define LED_ON_LEVAL          				1

//firmware check
#define FIRMWARES_SIGNATURE_ENABLE          0


//////////////// SMP SETTING  //////////////////////////////
#define BLE_SECURITY_ENABLE 			   	0
//////////////////board sel/////////////////////////////////////
#define PCBA_8258_DONGLE_48PIN          1
#define PCBA_8258_C1T139A30_V1_0        2
#define PCBA_8258_C1T139A30_V1_2        3
#define PCBA_8258_C1T140A3_V1_1         4   // 32pin

#ifndef PCBA_8258_SEL // user can define in user_app_config.h
#if (MESH_USER_DEFINE_MODE == MESH_IRONMAN_MENLO_ENABLE)
#define PCBA_8258_SEL			PCBA_8258_C1T140A3_V1_1  // PCBA_8258_DONGLE_48PIN   //
#else
#define PCBA_8258_SEL			PCBA_8258_DONGLE_48PIN
#endif
#endif


#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known
#define	__LOG_RT_ENABLE__		0
//#define	__DEBUG_PRINT__			0

#if DUAL_MESH_ZB_BL_EN
#define FLASH_2M_ENABLE         1
#elif DUAL_VENDOR_EN
#define FLASH_2M_ENABLE         1
#else
#define FLASH_2M_ENABLE         0
#endif

#if FLASH_2M_ENABLE
#if DUAL_MESH_ZB_BL_EN
#define PINGPONG_OTA_DISABLE    1 // it can disable only when 1M flash.
#else
#define PINGPONG_OTA_DISABLE    0 // it can disable only when 1M flash.
#endif
#if	PINGPONG_OTA_DISABLE
#define SWITCH_FW_ENABLE		0 // set to 0, just for particular customer 
#endif
#endif

//////////// product  Information  //////////////////////////////
#define ID_VENDOR				0x248a			// for report
#define ID_PRODUCT_BASE			0x880C
#define STRING_VENDOR			L"Telink"
#define STRING_PRODUCT			L"BLE Mesh"
#define STRING_SERIAL			L"TLSR825X"

#define DEV_NAME                "SigMesh"

#define APPLICATION_DONGLE		0					// or else APPLICATION_DEVICE
#define	USB_PRINTER				1
#define	FLOW_NO_OS				1

/////////////////////HCI ACCESS OPTIONS///////////////////////////////////////
#define HCI_USE_NONE	0
#define HCI_USE_UART	1
#define HCI_USE_USB		2

#ifndef HCI_ACCESS
#if (WIN32 || PTS_TEST_EN)
#define HCI_ACCESS		HCI_USE_USB
#else
#define HCI_ACCESS		HCI_USE_NONE
#endif 

#if (HCI_ACCESS==HCI_USE_UART)
#define UART_TX_PIN		UART_TX_PB1
#define UART_RX_PIN		UART_RX_PB0
#endif
#endif

#ifndef HCI_LOG_FW_EN
#define HCI_LOG_FW_EN   0
#if HCI_LOG_FW_EN
#define PRINT_DEBUG_INFO                    1
#endif
#endif

#define ADC_ENABLE		0
#if ADC_ENABLE
#define ADC_BASE_MODE	1	//GPIO voltage
#define ADC_VBAT_MODE	2	//Battery Voltage

#define ADC_MODE		ADC_VBAT_MODE
#define ADC_CHNM_ANA_INPUT 		GPIO_PB3 // one of ADC_GPIO_tab[]
#define ADC_PRESCALER	ADC_PRESCALER_1F8
#endif

#define ONLINE_STATUS_EN        0

#if DUAL_MESH_ZB_BL_EN
#define DUAL_MODE_ADAPT_EN 			1   // dual mode as master with Zigbee
#else
#define DUAL_MODE_ADAPT_EN 			0   // dual mode as master with Zigbee
#endif
#if (0 == DUAL_MODE_ADAPT_EN)
#define DUAL_MODE_WITH_TLK_MESH_EN  0   // dual mode as slave with Telink mesh
#endif

#define SPEECH_ENABLE			0

/////////////////// mesh project config /////////////////////////////////
#ifndef TRANSITION_TIME_DEFAULT_VAL
#if (MESH_RX_TEST || (!MD_DEF_TRANSIT_TIME_EN) || SPEECH_ENABLE)
#define TRANSITION_TIME_DEFAULT_VAL (0)
#else
	#if MI_API_ENABLE
#define TRANSITION_TIME_DEFAULT_VAL	0
	#else
#define TRANSITION_TIME_DEFAULT_VAL (GET_TRANSITION_TIME_WITH_STEP(1, TRANSITION_STEP_RES_1S)) // (0x41)  // 0x41: 1 second // 0x00: means no default transition time
	#endif
#endif
#endif

/////////////////// MODULE /////////////////////////////////
#if (MI_SWITCH_LPN_EN || GATT_LPN_EN)
#define BLE_REMOTE_PM_ENABLE			1
#else
#define BLE_REMOTE_PM_ENABLE			0
#endif
#if BLE_REMOTE_PM_ENABLE
#define PM_DEEPSLEEP_RETENTION_ENABLE   1   // must
#else
#define PM_DEEPSLEEP_RETENTION_ENABLE   0
#endif
#define BLE_REMOTE_SECURITY_ENABLE      0
#define BLE_IR_ENABLE					0
#define BLE_SIG_MESH_CERTIFY_ENABLE 	0

#ifndef BLT_SOFTWARE_TIMER_ENABLE
#define BLT_SOFTWARE_TIMER_ENABLE		0
#endif

//////////////////////////// KEYSCAN/MIC  GPIO //////////////////////////////////
#define	MATRIX_ROW_PULL					PM_PIN_PULLDOWN_100K
#define	MATRIX_COL_PULL					PM_PIN_PULLUP_10K

#define	KB_LINE_HIGH_VALID				0   //dirve pin output 0 when keyscan, scanpin read 0 is valid
#define DEEPBACK_FAST_KEYSCAN_ENABLE	1   //proc fast scan when deepsleep back trigged by key press, in case key loss
#define KEYSCAN_IRQ_TRIGGER_MODE		0
#define LONG_PRESS_KEY_POWER_OPTIMIZE	1   //lower power when pressing key without release

//stuck key
#define STUCK_KEY_PROCESS_ENABLE		0
#define STUCK_KEY_ENTERDEEP_TIME		60  //in s

//repeat key
#define KB_REPEAT_KEY_ENABLE			0
#define	KB_REPEAT_KEY_INTERVAL_MS		200
#define KB_REPEAT_KEY_NUM				1
//

//----------------------- GPIO for UI --------------------------------
#if ((0 == PM_DEEPSLEEP_RETENTION_ENABLE) && DUAL_VENDOR_EN)
#define SLEEP_FUNCTION_DISABLE          1
#endif

//---------------  Button 
#if (PCBA_8258_SEL == PCBA_8258_DONGLE_48PIN)
#define PULL_WAKEUP_SRC_PD6     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PD5     PM_PIN_PULLUP_1M	//btn
#define PD6_INPUT_ENABLE		1
#define PD5_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD6
#define	SW2_GPIO				GPIO_PD5
#elif (PCBA_8258_SEL == PCBA_8258_C1T140A3_V1_1)
#define PULL_WAKEUP_SRC_PD7     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PA1     PM_PIN_PULLUP_1M	//btn
#define PD7_INPUT_ENABLE		1
#define PA1_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD7
#define	SW2_GPIO				GPIO_PA1
#elif(PCBA_8258_SEL == PCBA_8258_C1T139A30_V1_2)
#define PULL_WAKEUP_SRC_PB2     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PB3     PM_PIN_PULLUP_1M	//btn
#define PB2_INPUT_ENABLE		1
#define PB3_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PB2            // SW2 in board
#define	SW2_GPIO				GPIO_PB3            // SW4 in board

#if 1 // must output 0, because it is keyboard array. pull down is not enough to output low level.
#define PB4_FUNC                AS_GPIO
#define PB4_OUTPUT_ENABLE       1
#define PB4_DATA_OUT            0
#endif


#elif(PCBA_8258_SEL == PCBA_8258_C1T139A30_V1_0)    // PCBA_8258_C1T139A30_V1_0
#define PULL_WAKEUP_SRC_PD2     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PD1     PM_PIN_PULLUP_1M	//btn
#define PD2_INPUT_ENABLE		1
#define PD1_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD2
#define	SW2_GPIO				GPIO_PD1
#endif


/////////////////// Clock  /////////////////////////////////

#if DUAL_MESH_ZB_BL_EN // keep same with zb
#define CLOCK_SYS_CLOCK_HZ  	32000000
#elif (MI_API_ENABLE)
#define CLOCK_SYS_CLOCK_HZ  	48000000
#elif DEBUG_CFG_CMD_GROUP_AK_EN
#define CLOCK_SYS_CLOCK_HZ  	32000000
#elif SPEECH_ENABLE
#define CLOCK_SYS_CLOCK_HZ  	48000000
#else
#define CLOCK_SYS_CLOCK_HZ  	16000000
#endif
#define     PWM_PCLK_SPEED				12000000 //pwm clock 12M.
enum{
	CLOCK_PWM_CLOCK_1S = PWM_PCLK_SPEED,
	CLOCK_PWM_CLOCK_1MS = (CLOCK_PWM_CLOCK_1S / 1000),
	CLOCK_PWM_CLOCK_1US = (CLOCK_PWM_CLOCK_1S / 1000000),
};

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
}; 

#define XIAOMI_MODULE_ENABLE	MI_API_ENABLE
#define XIAOMI_TEST_CODE_ENABLE 	0

#define C1T21_3A_1_1	0

#if C1T21_3A_1_1
#define GPIO_LED_WHITE					GPIO_PB1// 1
#define GPIO_LED_GREEN					GPIO_PB0// 5
#define GPIO_LED_BLUE					GPIO_PB7// 2
#define GPIO_LED_RED					GPIO_PB4// 0
#else
#define GPIO_LED_WHITE					GPIO_PB6// 3
#define GPIO_LED_GREEN					GPIO_PB5// 1
#define GPIO_LED_BLUE					GPIO_PB4// 0
#define GPIO_LED_RED					GPIO_PB7// 2

#endif
//---------------  LED / PWM

#define PWM_R       GPIO_LED_RED		//red
#define PWM_G       GPIO_LED_GREEN		//green
#define PWM_B       GPIO_LED_BLUE		//blue
#define PWM_W       GPIO_LED_WHITE		//white

#if C1T21_3A_1_1	
#define PWM_FUNC_R  AS_PWM0  // AS_PWM_SECOND
#define PWM_FUNC_G  AS_PWM5  // AS_PWM_SECOND
#define PWM_FUNC_B  AS_PWM2  // AS_PWM_SECOND
#define PWM_FUNC_W  AS_PWM3  // AS_PWM_SECOND

#define PWMID_R     0
#define PWMID_G     5
#define PWMID_B     2
#define PWMID_W     3
                    
#define PWM_INV_R   0
#define PWM_INV_G   0
#define PWM_INV_B   0
#define PWM_INV_W   0

#else
#define PWM_FUNC_R  AS_PWM2  // AS_PWM_SECOND
#define PWM_FUNC_G  AS_PWM1  // AS_PWM_SECOND
#define PWM_FUNC_B  AS_PWM0  // AS_PWM_SECOND
#define PWM_FUNC_W  AS_PWM3  // AS_PWM_SECOND

#define PWMID_R     2
#define PWMID_G     1
#define PWMID_B     0
#define PWMID_W     3
                    
#define PWM_INV_R   0
#define PWM_INV_G   0
#define PWM_INV_B   0
#define PWM_INV_W   0
#endif
#ifndef GPIO_LED
#define GPIO_LED	PWM_R
#endif

/////////////open SWS digital pullup to prevent MCU err, this is must ////////////
#define PA7_DATA_OUT			1

//save suspend current
#if BLE_REMOTE_PM_ENABLE
#define PA5_FUNC 	AS_GPIO     // USB DM
#define PA6_FUNC 	AS_GPIO     // USB DP
#elif(HCI_ACCESS == HCI_USE_USB)
#define PA5_FUNC			AS_USB
#define PA5_INPUT_ENABLE	1
#define PA6_FUNC			AS_USB
#define PA6_INPUT_ENABLE	1
#endif

/////////////////// Clock  /////////////////////////////////
#define	USE_SYS_TICK_PER_US
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC


//////////////////Extern Crystal Type///////////////////////
#define CRYSTAL_TYPE			XTAL_12M		//  extern 12M crystal

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#if (MI_API_ENABLE)
#define WATCHDOG_INIT_TIMEOUT		20000  //in mi mode the watchdog timeout is 20s
#else
#define WATCHDOG_INIT_TIMEOUT		2000  //in mi mode the watchdog timeout is 20s
#endif

#if MI_API_ENABLE && (MSC_TYPE != MSC_NONE)
/////////////////// MSC //////////////////////////////////
#define PULL_WAKEUP_SRC_PB4             PM_PIN_PULLUP_10K
#define GPIO_MSC_RESET                  GPIO_PB4
#define PB4_FUNC                        AS_GPIO

#define PULL_WAKEUP_SRC_PC2             PM_PIN_PULLUP_10K
#define GPIO_MSC_SDA                    GPIO_PC2

#define PULL_WAKEUP_SRC_PC3             PM_PIN_PULLUP_10K
#define GPIO_MSC_SCL                    GPIO_PC3    

//i2c clock = system_clock/(4*DivClock)
#define I2C_CLOCK_100K                  (CLOCK_SYS_CLOCK_HZ/(4*100000))
#define I2C_CLOCK_400K                  (CLOCK_SYS_CLOCK_HZ/(4*400000))
#define I2C_MSC_FREQ                    I2C_CLOCK_400K 
#endif

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

#if MI_API_ENABLE
#include "mi_config.h"
#endif
/////////////////// set default   ////////////////

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
