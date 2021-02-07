/********************************************************************************************************
 * @file     ble_ll_ota.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/


#include "tl_common.h"
#include "proj_lib/ble/ble_common.h"
#include "proj_lib/ble/trace.h"
#include "drivers/9518/pm.h"
#include "drivers/9518/flash.h"
#include "drivers/9518/watchdog.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "mesh_ota.h"
#include "version.h"
//9518
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble_common.h"
#include "stack/ble/ble_format.h"
#include "stack/ble/service/ota.h"
#include "stack/ble/service/ota_stack.h"
#include <stack/ble/trace.h>
#include "stack/ble/ll/ll_conn/ll_conn.h"
#include "stack/ble/attr/gatt.h"
#include "stack/ble/ll/ll_stack.h"
#include "stack/ble/ll/ll.h"

#if AIS_ENABLE
#include "proj_lib/mesh_crypto/aes_cbc.h"
#include "user_ali.h"
#endif
_attribute_data_retention_  unsigned short 		crc16_poly[2] = {0, 0xa001}; //0x8005 <==> 0xa001

static const unsigned long crc32_half_tbl[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned short crc16 (unsigned char *pD, int len)
{
    unsigned short crc = 0xffff;
    //unsigned char ds;
    int i,j;

    for(j=len; j>0; j--)
    {
        unsigned char ds = *pD++;
        for(i=0; i<8; i++)
        {
            crc = (crc >> 1) ^ crc16_poly[(crc ^ ds ) & 1];
            ds = ds >> 1;
        }
    }

     return crc;
}


#if 0
unsigned long crc32_cal(unsigned long crc, unsigned char* input, unsigned long* table, int len)
{
    unsigned char* pch = input;
    for(int i=0; i<len; i++)
    {
        crc = (crc>>8) ^ table[(crc^*pch) & 0xff];
        pch++;
    }

    return crc;
}
#endif


//_attribute_ram_code_
unsigned long crc32_half_cal(unsigned long crc, unsigned char* input, unsigned long* table, int len)
{
    unsigned char* pch = input;
    for(int i=0; i<len; i++)
    {
        crc = (crc>>4) ^ table[(crc^*pch) & 0x0f];
        pch++;
    }

    return crc;
}

_attribute_data_retention_	int ota_adr_index = -1;
_attribute_data_retention_	u32 blt_ota_start_tick;
_attribute_data_retention_	u32 blt_ota_receive_pkt_tick;
_attribute_data_retention_	u32 blt_ota_timeout_us = 30000000;  //default 30 second

_attribute_data_retention_	ota_service_t blcOta;

//_attribute_data_retention_	int		ota_program_bootAddr = 0x40000;



#if (FLASH_2M_ENABLE && PINGPONG_OTA_DISABLE)
_attribute_data_retention_	int 	ota_program_bootAddr = FLASH_ADR_UPDATE_NEW_FW; // it will be used in cpu_wakeup init, and set value for ota_program_offset_
#else
_attribute_data_retention_	int 	ota_program_bootAddr = 0x80000;
#endif

_attribute_data_retention_	u32 	ota_program_offset = 0;
_attribute_data_retention_	int 	ota_firmware_size_k = FW_SIZE_MAX_K;


_attribute_data_retention_	u32		ota_version_flashAddr = 0xEF000;
_attribute_data_retention_	u32		ota_version_number = 0xFFFFFFFF;


_attribute_data_retention_	u16		ota_handle;
_attribute_data_retention_	u8		ota_response_buf[10];
_attribute_data_retention_	u32 	ota_reboot_tick;


_attribute_data_retention_	ota_startCb_t		otaStartCb = NULL;
_attribute_data_retention_	ota_versionCb_t 	otaVersionCb = NULL;
_attribute_data_retention_	ota_resIndicateCb_t otaResIndicateCb ;



u32 blt_ota_finished_time = 0;
u8  blt_ota_finished_flag = 0;
u8  blt_ota_terminate_flag = 0;

_attribute_data_retention_ ota_service_t blcOta;


#if 0
unsigned long crc32_cal(unsigned long crc, unsigned char* input, unsigned long* table, int len)
{
    unsigned char* pch = input;
    for(int i=0; i<len; i++)
    {
        crc = (crc>>8) ^ table[(crc^*pch) & 0xff];
        pch++;
    }

    return crc;
}
#endif


u8 fw_ota_value =0;
u8 get_fw_ota_value()
{
	return fw_ota_value;
}



void bls_cb_ota_procTimeout(void)
{
	blt_slave_ota_finish_handle();

	if(blt_ota_start_tick && clock_time_exceed(blt_ota_start_tick , blt_ota_timeout_us)){  //OTA timeout
		rf_link_slave_ota_finish_led_and_reboot(OTA_TIMEOUT);
	}
}

void blt_ota_finished_flag_set(u8 reset_flag)
{
	if(blcOta.ota_start_flag && (blt_ota_finished_time == 0)){
		blt_ota_finished_flag = reset_flag;
		blt_ota_finished_time = clock_time()|1;
	}
}

void rf_link_slave_ota_finish_led_and_reboot(u8 st)
{
	if(OTA_SUCCESS == st){
        ota_set_flag ();
    }
    else{
       if(ota_adr_index>=0){
			irq_disable();

			//for(int i=0;i<=ota_adr_index;i+=256)
			for(int i=(ota_adr_index&0x3ff00); i>=0; i-=256) //erase from end to head
			{  //4K/16 = 256
				flash_erase_sector(ota_program_offset + (i<<4));
			}
		}
    }
	
#if KEEP_ONOFF_STATE_AFTER_OTA 
	set_keep_onoff_state_after_ota();
#endif
	if(otaResIndicateCb){
		otaResIndicateCb(st);   // should be better at last.
	}
    irq_disable ();
    start_reboot();
}

void blt_slave_ota_finish_handle()		
{	
    if(blt_ota_finished_time){
        static u8 terminate_cnt;
        u8 reboot_flag = 0;
        if((0 == terminate_cnt) && (blt_ota_terminate_flag)){
               terminate_cnt = 6;
               bls_ll_terminateConnection(0x13);
        }
        
        if(terminate_cnt){
            terminate_cnt--;
            if(!terminate_cnt){
                reboot_flag = 1;
            }
        }
        
        if((!blt_ota_terminate_flag)
         &&((u32)(clock_time() - blt_ota_finished_time) > 2000*1000 * sys_tick_per_us)){
            blt_ota_terminate_flag = 1;    // for ios: no last read command
        }
        
        if(((u32)(clock_time() - blt_ota_finished_time) > 4000*1000 * sys_tick_per_us)){
            reboot_flag = 1;
        }
        
        if(reboot_flag){
            rf_link_slave_ota_finish_led_and_reboot(blt_ota_finished_flag);
            // have been reboot
        }
    }
}

#if(AIS_ENABLE)
#define MAIN_VERSION		0
#define SUB_VERSION			0
#define MODIFY_VERSION		0

const ais_fw_info_t  ais_fw_info = { 
	.device_type = (u8)MESH_PID_SEL, // device type
    .fw_version = (MAIN_VERSION<<16) | (SUB_VERSION<<8) | MODIFY_VERSION, 
};

void ota_save_data_ali(u32 adr, u8 * data, u16 len){
#if 1 // (! PINGPONG_OTA_DISABLE)
	if (adr == 0)
	{
		fw_ota_value = data[0x20];
		data[0x20] = 0xff;					//FW flag invalid
	}
#endif

	flash_write_page(ota_program_offset + adr, len, data);
}

int ais_ota_version_get()
{
	ais_msg_t ais_version;
	memset(&ais_version, 0x00, sizeof(ais_version));
	ais_version.msg_type = AIS_FW_VERSION_RSP;
	ais_version.length = ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_fw_info);
	memcpy(ais_version.data, &ais_fw_info, sizeof(ais_fw_info));
	if(ais_gatt_auth.auth_ok){
		ais_version.enc_flag = 1;
		AES128_pkcs7_padding(ais_version.data, sizeof(ais_fw_info), ais_version.data);
		aes_cbc_encrypt(ais_version.data, sizeof(ais_fw_info), &ctx, ais_gatt_auth.ble_key, iv);
	}
	return bls_att_pushNotifyData(AIS_NOTIFY_HANDLE, (u8 *)&ais_version, OFFSETOF(ais_msg_t,data)+(ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_fw_info)));
}

int ais_ota_req(u8 *p)
{
	ais_msg_t ais_msg_rsp;
	//ais_ota_req_t *ota_req_p = (ais_ota_req_t *)p;
	memset(&ais_msg_rsp, 0x00, sizeof(ais_msg_rsp));		

	ais_msg_rsp.msg_type = AIS_OTA_RSP;
	ais_msg_rsp.length = ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_ota_rsp_t);
	ais_msg_rsp.ais_ota_rsp.one_round_pkts = 0;//must set to 0 now.
#if 0 // set 0 always allow ota.
	if((ota_req_p->device_type == ais_fw_info.device_type) && (ota_req_p->fw_version > ais_fw_info.fw_version)&&(ota_req_p->ota_flag == 0))
#endif
	{
		ais_msg_rsp.ais_ota_rsp.allow_ota = ais_gatt_auth.auth_ok;

		ota_adr_index = -1;
		blcOta.ota_start_flag = ais_gatt_auth.auth_ok;   //set flag
		blt_ota_start_tick = clock_time()|1;  //mark time
		if(otaStartCb && ais_gatt_auth.auth_ok){
			otaStartCb();
		}
	}
	
	if(ais_gatt_auth.auth_ok){	
		ais_msg_rsp.enc_flag = 1;
		AES128_pkcs7_padding(ais_msg_rsp.data, sizeof(ais_ota_rsp_t), ais_msg_rsp.data);
		aes_cbc_encrypt(ais_msg_rsp.data, sizeof(ais_ota_rsp_t), &ctx, ais_gatt_auth.ble_key, iv);
	}
	
	return bls_att_pushNotifyData(AIS_NOTIFY_HANDLE, (u8 *)&ais_msg_rsp, OFFSETOF(ais_msg_t,data)+(ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_ota_rsp_t)));
}

int ais_ota_result(u8 result)
{
	ais_msg_t ais_msg_result;
	memset(&ais_msg_result, 0x00, sizeof(ais_msg_result));		

	ais_msg_result.msg_type = AIS_OTA_RESULT;
	ais_msg_result.length = ais_gatt_auth.auth_ok?AES_BLOCKLEN:1;
	ais_msg_result.ota_result = (OTA_SUCCESS==result) ? 1:0;

	if(ais_gatt_auth.auth_ok){
		ais_msg_result.enc_flag = 1;
		AES128_pkcs7_padding(ais_msg_result.data, 1, ais_msg_result.data);
		aes_cbc_encrypt(ais_msg_result.data, 1, &ctx, ais_gatt_auth.ble_key, iv);
	}
	return bls_att_pushNotifyData(AIS_NOTIFY_HANDLE, (u8 *)&ais_msg_result, OFFSETOF(ais_msg_t,data)+(ais_gatt_auth.auth_ok?AES_BLOCKLEN:1));
}

int ais_ota_rc_report(u8 frame_desc, u32 trans_size)
{
	ais_msg_t ais_msg_result;
	memset(&ais_msg_result, 0x00, sizeof(ais_msg_result));		

	ais_msg_result.msg_type = AIS_OTA_RECEVIED;
	ais_msg_result.length = ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_ota_receive_t);
	ais_msg_result.ais_ota_rcv.seg_index = frame_desc;
	ais_msg_result.ais_ota_rcv.trans_size_last = trans_size;
	if(ais_gatt_auth.auth_ok){
		ais_msg_result.enc_flag = 1;
		AES128_pkcs7_padding(ais_msg_result.data, sizeof(ais_ota_receive_t), ais_msg_result.data);
		aes_cbc_encrypt(ais_msg_result.data, sizeof(ais_ota_receive_t), &ctx, ais_gatt_auth.ble_key, iv);
	}
	return bls_att_pushNotifyData(AIS_NOTIFY_HANDLE, (u8 *)&ais_msg_result, OFFSETOF(ais_msg_t,data)+(ais_gatt_auth.auth_ok?AES_BLOCKLEN:sizeof(ais_ota_receive_t)));
}


extern u8 mesh_cmd_ut_rx_seg[];
const u8 company[4] = {'K', 'N', 'L', 'T'};
int ais_otaWrite(void * p)
{

	rf_packet_att_data_t *req = (rf_packet_att_data_t*)p;
	static u8 err_flg = OTA_SUCCESS;
	static u32 fw_rcv_total_size = 0;
	ais_msg_t *msg_p = (ais_msg_t *)req->dat;
	if(ais_gatt_auth.auth_ok && (msg_p->msg_type != AIS_OTA_DATA)){
		u16 len = ((req->l2cap-3)+AES_BLOCKLEN-1)/AES_BLOCKLEN*AES_BLOCKLEN;
		aes_cbc_decrypt(msg_p->data, (len>AIS_MAX_DATA_SIZE)?0:len, &ctx, ais_gatt_auth.ble_key, iv);
	}
	
	if(msg_p->msg_type == AIS_FW_VERSION_GET){
		ais_ota_version_get();
	}
	else if(msg_p->msg_type == AIS_OTA_REQ){
		ais_ota_req(msg_p->data);
	}

	if(!blcOta.ota_start_flag){
		return 0;
	}
	
	if(msg_p->msg_type == AIS_OTA_END){
		if(FW_CHECK_AGTHM1 == get_ota_check_type()){
			if(is_valid_ota_check_type1()){
				err_flg = OTA_SUCCESS;
			}
			else{
				err_flg = OTA_DATA_CRC_ERR;
			}
		}
		 	
		blt_ota_finished_flag_set(err_flg);

		ais_ota_result(err_flg);
	}
	else if(msg_p->msg_type == AIS_OTA_DATA){
		u16 cur_index =  ota_adr_index+1;
		if((msg_p->frame_seq == (cur_index%(msg_p->frame_total+1)))){
			blt_ota_start_tick = clock_time()|1;  //mark time			
			u16 data_len = msg_p->length;
			
			if(cur_index == 0){
				if(memcmp(req->dat+12, company, sizeof(company))){
					err_flg = OTA_ERR_STS;
				}
			}
			//log_data(TR_24_ota_adr,ota_adr);
			if((cur_index*data_len)>=(ota_firmware_size_k<<10)){
				err_flg = OTA_OVERFLOW;
			}else{
				ota_save_data (fw_rcv_total_size, req->dat + 4, data_len);

				flash_read_page(ota_program_offset + fw_rcv_total_size, data_len, mesh_cmd_ut_rx_seg);

				if(!memcmp(mesh_cmd_ut_rx_seg,req->dat + 4, data_len)){  //OK
					ota_adr_index++;
					fw_rcv_total_size += data_len;				
					if((!ais_gatt_auth.auth_ok) || (msg_p->frame_total == msg_p->frame_seq)){
						ais_ota_rc_report(msg_p->frame_desc, fw_rcv_total_size);
					}
				}
				else{ //flash write err
					err_flg = OTA_WRITE_FLASH_ERR;
				}
			}
				
		}
		else if(msg_p->frame_seq == (cur_index%(msg_p->frame_total+1))){  //maybe repeated OTA data, we neglect it, do not consider it ERR
			ais_ota_rc_report((msg_p->frame_desc & 0xf0)|(ota_adr_index % (msg_p->frame_total+1)), fw_rcv_total_size);
		}
		else{  //adr index err, missing at least one OTA data
			ais_ota_rc_report((msg_p->frame_desc & 0xf0)|(ota_adr_index % (msg_p->frame_total+1)), fw_rcv_total_size);
		}
	}

	if(err_flg){
		blt_ota_finished_flag_set(err_flg);
	}

	return 0;
}
#endif



void bls_ota_set_fwSize_and_fwBootAddr(int firmware_size_k, int boot_addr)
{
	ota_firmware_size_k = firmware_size_k;
	ota_program_bootAddr = boot_addr;
}


void bls_ota_set_VersionFlashAddr_and_VersionNumber(u32 version_flash_addr, u32 version_number)
{
	ota_version_flashAddr = version_flash_addr;

	flash_read_page(ota_version_flashAddr, 4, (u8 *)&ota_version_number);

	if(ota_version_number != version_number){
		if(ota_version_number != 0xFFFFFFFF){
			flash_erase_sector(ota_version_flashAddr);
		}
		flash_write_page(ota_version_flashAddr, 4, (u8 *)&version_number);
		flash_read_page(ota_version_flashAddr, 4, (u8 *)&ota_version_number);
	}
}


void bls_ota_registerStartCmdCb(ota_startCb_t cb)
{
	otaStartCb = cb;
}

void bls_ota_registerVersionReqCb(ota_versionCb_t cb)
{
	otaVersionCb = cb;
}

void bls_ota_registerResultIndicateCb(ota_resIndicateCb_t cb)
{
	otaResIndicateCb = cb;
}

void bls_ota_setTimeout(u32 timeout_us)
{
	blt_ota_timeout_us = timeout_us;
}



void ota_set_flag()
{
	u32 flag = 0x4b;
	flash_write_page(ota_program_offset + 0x20, 1, (u8 *)&flag);		//Set FW flag
	flag = 0;
	flash_write_page((ota_program_offset ? 0 : ota_program_bootAddr) + 0x20, 4, (u8 *)&flag);	//Invalid flag
}

void ota_save_data(u32 adr, u8* data){

	if (adr == 32)
	{
		data[0] = 0xff;					//FW flag invalid, 0x20
	}

	flash_write_page(ota_program_offset + adr, 16, data);
}


int otaWrite(void * p)
{
	rf_packet_att_data_t *req = (rf_packet_att_data_t*)p;

	ota_handle =  req->hl | (req->hh<<8);

	int err_flg = OTA_SUCCESS;

	u16 ota_adr =  req->dat[0] | (req->dat[1]<<8);
	u16 crc;
	if(ota_adr == CMD_OTA_FW_VERSION){
		//to be add
		if(otaVersionCb){
			otaVersionCb();
		}
	}
	else if(ota_adr == CMD_OTA_VERSION_REQUEST){
		u32 new_version =  req->dat[2] | (req->dat[3]<<8) | (req->dat[4]<<16) | (req->dat[5]<<24);

		if(new_version <= ota_version_number){
			err_flg = OTA_VERSION_NUMBER_ERR;
			blcOta.ota_forbid_flag = 1;   //set flag
		}
		else{
			blcOta.ota_forbid_flag = 0;   //clear flag
		}

		u8 rsp[7];
		rsp[0] = CMD_OTA_VERSION_RESPONSE;
		rsp[1] = CMD_OTA_VERSION_RESPONSE >> 8;
		rsp[2] = ota_version_number;
		rsp[3] = ota_version_number >> 8;
		rsp[4] = ota_version_number >> 16;
		rsp[5] = ota_version_number >> 24;
		rsp[6] = err_flg;

		if(blc_gatt_pushHandleValueNotify (BLS_CONN_HANDLE, ota_handle, (u8 *)&rsp, 7))
		{
			ota_response_buf[0] = ota_handle;
			ota_response_buf[1] = ota_handle >> 8;
			ota_response_buf[2] = 7;	// data length
			foreach(i, 7){
				ota_response_buf[3 + i] = rsp[i];
			}

			blcOta.ota_response_pending = 1;
		}
	}
	else if(ota_adr == CMD_OTA_START){
		flash_en_support_arch_flash(1);
		blcOta.ota_start_flag = 16;   //set flag
		blt_ota_start_tick = blt_ota_receive_pkt_tick = clock_time();  //mark time
		ota_adr_index = -1;
		if(otaStartCb){
			otaStartCb();
		}

		#if (BLE_OTA_FW_CHECK_EN)
		blcOta.fw_check_en = 0;
		blcOta.fw_crc_last_index = (FW_MAX_SIZE>>4);
		#endif
	}
	else if(ota_adr == CMD_OTA_START_LONG_PACKET){
		ota_response_buf[0] = ota_handle;
		ota_response_buf[1] = ota_handle >> 8;
		ota_response_buf[2] = 3;	// data length
		ota_response_buf[3] = CMD_OTA_RESULT;
		ota_response_buf[4] = CMD_OTA_RESULT >> 8;

		if(blcOta.ota_forbid_flag){
			err_flg = OTA_VERSION_NUMBER_ERR;
			ota_response_buf[5] = err_flg;

			blcOta.ota_response_pending = 1;
		}
		else if(req->dat[2] != 64 || req->dat[2] != 128){
			err_flg = OTA_PDU_ERR;
			ota_response_buf[5] = err_flg;

			blcOta.ota_response_pending = 1;
		}
		else if(req->dat[2] > bltData.connEffectiveMaxRxOctets){
			err_flg = OTA_PDU_LARGER_DLE;
			ota_response_buf[5] = err_flg;

			blcOta.ota_response_pending = 1;
		}
		else{
			if(req->dat[2] == 64){
				blcOta.ota_start_flag = 64;		//set flag
			}
			else{
				blcOta.ota_start_flag = 128;	//set flag
			}
			blt_ota_start_tick = blt_ota_receive_pkt_tick = clock_time();	//mark time
			ota_adr_index = -1;
			if(otaStartCb){
				otaStartCb();
			}

			#if (BLE_OTA_FW_CHECK_EN)
			blcOta.fw_check_en = 0;
			blcOta.fw_crc_last_index = (FW_MAX_SIZE>>4);
			#endif
		}
	}
	else if(ota_adr == CMD_OTA_END){

		u16 adrIndex_max	   = req->dat[2] | (req->dat[3]<<8);
		u16 adrIndex_max_check = req->dat[4] | (req->dat[5]<<8);

		#if (BLE_OTA_FW_CHECK_EN)
		if(blcOta.fw_check_en && !blcOta.fw_check_match){
			err_flg = OTA_FW_CHECK_ERR;
		}
		#endif

		ota_response_buf[0] = ota_handle;
		ota_response_buf[1] = ota_handle >> 8;
		ota_response_buf[2] = 3;	// data length
		ota_response_buf[3] = CMD_OTA_RESULT;
		ota_response_buf[4] = CMD_OTA_RESULT >> 8;

		//if no index_max check, set ota success directly, otherwise we check if any index_max match
		if( req->l2cap == 9 && (adrIndex_max ^ adrIndex_max_check) == 0xffff){  //index_max valid, we can check
			if(adrIndex_max != ota_adr_index){  //last one or more packets missed
				err_flg = OTA_DATA_UNCOMPLETE;
			}
		}

		if(!err_flg){
			if(otaResIndicateCb){
				otaResIndicateCb(OTA_SUCCESS);  //OTA successed indicate
			}

			ota_set_flag ();
//			start_reboot();
			ota_response_buf[5] = err_flg;
			blcOta.ota_response_pending = 1;
		}
		blcOta.ota_start_flag = 0;   //clear flag
	}
	else{
		blt_ota_start_tick = blt_ota_receive_pkt_tick = clock_time();  //mark time
		if(ota_adr_index + 1 == ota_adr){   //correct OTA data index
			if(blcOta.ota_start_flag == 16){
				crc = (req->dat[19]<<8) | req->dat[18];
				if(crc == crc16(req->dat, 18)){
					#if(BLE_OTA_FW_CHECK_EN)
					int fw_check_err = 0;
					if(!ota_adr){
						#if 0	//the initial version, there is no need to judge whether FLAG_FW_CHECK
						if(req->dat[8] == FLAG_FW_CHECK && req->dat[9] == FW_CHECK_AGTHM2){
							blcOta.fw_check_en = 1;
						}
						else if(req->dat[8] == 0){ //adr_0x06 is 0, no need check
							blcOta.fw_check_en = 0;
						}
						else{ //adr_0x06 is unexpected data, firmware is err
							blcOta.fw_check_en = 0;
							fw_check_err = 1;
						}
						#else
						blcOta.fw_check_en = 1;	//mandatory	FLAG_FW_CHECK
						#endif

						blcOta.fw_crc_init = 0xFFFFFFFF;  //crc init set to 0xFFFFFFFF
					}

					if(blcOta.fw_check_en && ota_adr<=blcOta.fw_crc_last_index){
						if(ota_adr == 1){
							u32 fw_size = req->dat[10] | req->dat[11]<<8 | req->dat[12]<<16 | req->dat[13]<<24;

							//firmware size none 0, and smaller than 256k, and (fw_size % 16 == 4)
							if(fw_size && (fw_size < FW_MAX_SIZE) && ((fw_size & 0x0f) == 4) ){
								blcOta.fw_crc_last_index = (fw_size>>4) - 1;
							}
							else{ //adr_0x18 ~ adr_0x1b firmware size err
								blcOta.fw_crc_last_index = (FW_MAX_SIZE>>4);
								blcOta.fw_check_en = 0;
								fw_check_err = 1;
							}
						}
						if(ota_adr == 2){
							if(req->dat[2] != 0x4B || req->dat[3] != 0x4E || req->dat[4] != 0x4C || req->dat[5] != 0x54){
								err_flg = OTA_FW_TYPE_ERR;
							}
						}


						u8 ota_dat[32];
						for(int i=0;i<16;i++){
							ota_dat[i*2] = req->dat[i+2]&0x0f;
							ota_dat[i*2+1] = req->dat[i+2]>>4;
						}
						blcOta.fw_crc_init = crc32_half_cal(blcOta.fw_crc_init, ota_dat, (unsigned long* )crc32_half_tbl, 32);
					}

					if(blcOta.fw_check_en && ota_adr == (blcOta.fw_crc_last_index + 1) ){  //姣旇緝check鍊�
						u32 fw_check_value = req->dat[2] | req->dat[3]<<8 | req->dat[4]<<16 | req->dat[5]<<24;
						if(fw_check_value == blcOta.fw_crc_init){  //crc match
							blcOta.fw_check_match = 1;
						}
						else{
							fw_check_err = 1;
						}
					}

					if(fw_check_err){
						err_flg = OTA_FW_CHECK_ERR;
					}
					else{
						ota_save_data (ota_adr<<4, req->dat + 2);

						u8 flash_check[16];

						flash_read_page(ota_program_offset + (ota_adr<<4),16,flash_check);

						if(!tmemcmp(flash_check,req->dat + 2,16)){  //OK
							ota_adr_index = ota_adr;
						}
						else{ //flash write err
							err_flg = OTA_WRITE_FLASH_ERR;
						}
					}
					#else
					ota_save_data (ota_adr<<4, req->dat + 2);
					u8 flash_check[16];
					flash_read_page(ota_program_offset + (ota_adr<<4),16,flash_check);

					if(!tmemcmp(flash_check,req->dat + 2,16)){  //OK
						ota_adr_index = ota_adr;
					}
					else{ //flash write err
						err_flg = OTA_WRITE_FLASH_ERR;
					}
					#endif
				}
				else{  //crc err
					err_flg = OTA_DATA_CRC_ERR;
				}
			}
			else{

			}
		}
		else if(ota_adr_index >= ota_adr){  //maybe repeated OTA data, we neglect it, do not consider it ERR

		}
		else{  //adr index err, missing at least one OTA data
			err_flg = OTA_PACKET_LOSS;
		}

	}

	if(err_flg){
		if(otaResIndicateCb){
			flash_en_support_arch_flash(0);
			otaResIndicateCb(err_flg);   //OTA fail indicate
		}

//		start_reboot();
		ota_response_buf[0] = ota_handle;
		ota_response_buf[1] = ota_handle >> 8;
		ota_response_buf[2] = 3;	// data length
		ota_response_buf[3] = CMD_OTA_RESULT;
		ota_response_buf[4] = CMD_OTA_RESULT >> 8;
		ota_response_buf[5] = err_flg;
		blcOta.ota_response_pending = 1;
	}

	return 0;
}

int otaRead(void * p)
{
	return 0;
}


void bls_ota_clearNewFwDataArea(void)
{
#if 1
		u32 tmp1 = 0;
		u32 tmp2 = 0;
		int cur_flash_setor;
		for(int i = 0; i < (ota_firmware_size_k>>2); ++i)
		{
			cur_flash_setor = ota_program_offset + i*0x1000;
			flash_read_page(cur_flash_setor, 		4, (u8 *)&tmp1);
			flash_read_page(cur_flash_setor + 2048, 4, (u8 *)&tmp2);

			if(tmp1 != ONES_32 || tmp2 != ONES_32)
			{
				flash_erase_sector(cur_flash_setor);
			}
		}

#else
		u32 tmp1 = 0;
		u32 tmp2 = 0;
		u32 tmp3 = 0;
		flash_read_page(ota_program_offset, 4, (u8 *)&tmp1);
		flash_read_page(ota_program_offset + 4092, 4, (u8 *)&tmp2);
		if(tmp1 != ONES_32 || tmp2 != ONES_32)
		{
			for(int i = (ota_firmware_size_k - 1)>>2; i>=0; --i)  //erase from end to head
			{
				flash_read_page(ota_program_offset + i*0x1000, 4, (u8 *)&tmp1);
				if(tmp1 == ONES_32){
					flash_read_page(ota_program_offset + i*0x1000 + 16, 4, (u8 *)&tmp2);
					if(tmp2 == ONES_32){
						flash_read_page(ota_program_offset + i*0x1000 + 4092, 4, (u8 *)&tmp3);
					}
				}

				if(tmp1 != ONES_32 || tmp2 != ONES_32 || tmp3 != ONES_32)
				{
					flash_erase_sector(ota_program_offset+i*0x1000);
				}
			}
		}
#endif
}


void bls_ota_procTimeout(void)
{
	//bls_cb_ota_procTimeout();
	if(clock_time_exceed(blt_ota_receive_pkt_tick , 3000000)){  //OTA timeout between RF packet, 3 second
		if(otaResIndicateCb){
			otaResIndicateCb(OTA_RF_PACKET_TIMEOUT);   //OTA fail indicate
		}

		ota_response_buf[0] = ota_handle;
		ota_response_buf[1] = ota_handle >> 8;
		ota_response_buf[2] = 3;	// data length
		ota_response_buf[3] = CMD_OTA_RESULT;
		ota_response_buf[4] = CMD_OTA_RESULT >> 8;
		ota_response_buf[5] = OTA_RF_PACKET_TIMEOUT;
		blcOta.ota_response_pending = 1;
		blcOta.ota_start_flag = 0;
	}
	else if(clock_time_exceed(blt_ota_start_tick , blt_ota_timeout_us)){  //OTA total timeout
		if(otaResIndicateCb){
			otaResIndicateCb(OTA_TIMEOUT);   //OTA fail indicate
		}

		ota_response_buf[0] = ota_handle;
		ota_response_buf[1] = ota_handle >> 8;
		ota_response_buf[2] = 3;	// data length
		ota_response_buf[3] = CMD_OTA_RESULT;
		ota_response_buf[4] = CMD_OTA_RESULT >> 8;
		ota_response_buf[5] = OTA_TIMEOUT;
		blcOta.ota_response_pending = 1;
		blcOta.ota_start_flag = 0;
	}
}


void bls_ota_conn_terminate_clear_flag(void){
	blcOta.ota_forbid_flag = 0;
	if(blcOta.ota_response_pending){
		blcOta.ota_response_pending = 0;

		if(ota_response_buf[2] == 3){
			if(!ota_response_buf[5]){
				blcOta.ota_reboot_pending = 1;//success
			}
			else{
				blcOta.ota_reboot_pending = 2;//fail
			}
			ota_reboot_tick = clock_time();
		}
	}
}


void bls_ota_response(void)
{
	static u32 ota_rsp_tick;
	if(clock_time_exceed(ota_rsp_tick, 30000))//30ms
	{
		u16 ota_rsp_handle =  ota_response_buf[0] | (ota_response_buf[1]<<8);
		u8 data_length = ota_response_buf[2];
		u8 rsp[data_length];
		foreach(i, data_length){
			rsp[i] = ota_response_buf[3 + i];
		}

		u8 returnValue = blc_gatt_pushHandleValueNotify (BLS_CONN_HANDLE, ota_rsp_handle, (u8 *)&rsp, data_length);
		if(returnValue == BLE_SUCCESS || returnValue == LL_ERR_CONNECTION_NOT_ESTABLISH)
		{
			blcOta.ota_response_pending = 0;
			if(data_length == 3){
				if(!ota_response_buf[5]){
					blcOta.ota_reboot_pending = 1;//success
				}
				else{
					blcOta.ota_reboot_pending = 2;//fail
				}
				ota_reboot_tick = clock_time();
			}
		}
	}
}


void bls_ota_reboot(void)
{
	if(clock_time_exceed(ota_reboot_tick, 100000))//100ms
	{
		if(blcOta.ota_reboot_pending == 2){//fail
			if(ota_adr_index>=0){
				irq_disable();

				//for(int i=0;i<=ota_adr_index;i+=256)
				for(int i=(ota_adr_index&0x3ff00); i>=0; i-=256) //erase from end to head
				{  //4K/16 = 256
					flash_erase_sector(ota_program_offset + (i<<4));
				}
			}
		}
		start_reboot();
	}
}

