/********************************************************************************************************
 * @file	ota_stack.h
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
#ifndef STACK_BLE_SERVICE_OTA_STACK_H_
#define STACK_BLE_SERVICE_OTA_STACK_H_




#ifndef BLE_OTA_FW_CHECK_EN
#define BLE_OTA_FW_CHECK_EN					1
#endif

#define FW_MAX_SIZE							(FW_SIZE_MAX_K * 1024) // 0x40000			//256K



#define FLAG_FW_CHECK						0x5D
//#define FW_CHECK_AGTHM2						0x02



typedef struct{
	u8  ota_start_flag;
	u8  ota_reboot_pending;
#if (BLE_OTA_FW_CHECK_EN)
	u8 	fw_check_en;
	u8  fw_check_match;

	u32 fw_crc_init;

	u16 fw_crc_last_index;
#endif
	u8  ota_forbid_flag;
	u8  ota_response_pending;
}ota_service_t;

extern ota_service_t blcOta;


extern u32	blt_ota_start_tick;
extern u32	blt_ota_receive_pkt_tick;
extern u32	blt_ota_timeout_us;

extern u32	ota_program_offset;
extern int 	ota_firmware_size_k;

extern u32	ota_version_flashAddr;
extern u32	ota_version_number;

extern u8	ota_response_buf[10];
extern u32 	ota_reboot_tick;


void bls_ota_procTimeout(void);
void bls_ota_conn_terminate_clear_flag(void);
void bls_ota_response(void);
void bls_ota_reboot(void);




#endif /* STACK_BLE_SERVICE_OTA_STACK_H_ */
