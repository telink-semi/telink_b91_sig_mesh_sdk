/********************************************************************************************************
 * @file	hids.h
 *
 * @brief	for TLSR chips
 *
 * @author	BLE GROUP
 * @date	2020.06
 *
 * @par		Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
 *
 *			The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or  
 *          alter) any information contained herein in whole or in part except as expressly authorized  
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible  
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)  
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#pragma once


/**
 * @brief	hids_uuid Hids Characteristic UUID
 */
#define CHARACTERISTIC_UUID_HID_BOOT_KEY_INPUT          0x2A22    //!< HID Boot Keyboard Input Report
#define CHARACTERISTIC_UUID_HID_BOOT_KEY_OUTPUT         0x2A32    //!< HID Boot Keyboard Output Report
#define CHARACTERISTIC_UUID_HID_BOOT_MOUSE_INPUT        0x2A33    //!< HID Boot Mouse Input Report
#define CHARACTERISTIC_UUID_HID_INFORMATION             0x2A4A    //!< HID Information
#define CHARACTERISTIC_UUID_HID_REPORT_MAP              0x2A4B    //!< HID Report Map
#define CHARACTERISTIC_UUID_HID_CONTROL_POINT           0x2A4C    //!< HID Control Point
#define CHARACTERISTIC_UUID_HID_REPORT                  0x2A4D    //!< HID Report
#define CHARACTERISTIC_UUID_HID_PROTOCOL_MODE           0x2A4E    //!< HID Protocol Mode


/**
 *  @brief  HID Report ID
 */
#define HID_REPORT_ID_KEYBOARD_INPUT                    1   //!< Keyboard input report ID
#define HID_REPORT_ID_CONSUME_CONTROL_INPUT             2   //!< Consumer Control input report ID
#define HID_REPORT_ID_MOUSE_INPUT                       3   //!< Mouse input report ID
#define HID_REPORT_ID_GAMEPAD_INPUT						4   //!< Gamepad  input report ID
#define HID_REPORT_ID_LED_OUT                           0   //!< LED output report ID
#define HID_REPORT_ID_FEATURE                           0   //!< Feature report ID

#define HID_REPORT_ID_CTRL_VOICE                        9

#define HID_REPORT_ID_AUDIO_FIRST_INPUT                 10//250
#define HID_REPORT_ID_AUDIO_SECND_INPUT                 11//251
#define HID_REPORT_ID_AUDIO_THIRD_INPUT                 12//247



/**
 *  @brief  HID Report type
 */

#define HID_REPORT_TYPE_INPUT                           1
#define HID_REPORT_TYPE_OUTPUT                          2
#define HID_REPORT_TYPE_FEATURE                         3



/**
 *  @brief  Definition for HID protocol mode
 */
#define HID_PROTOCOL_MODE_BOOT                          0
#define HID_PROTOCOL_MODE_REPORT                        1
#define DFLT_HID_PROTOCOL_MODE                          HID_PROTOCOL_MODE_REPORT



/**
 *  @brief  Definition for HID information flags
 */
#define HID_FLAGS_REMOTE_WAKE                           0x01      // RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE                  0x02      // NormallyConnectable
