/********************************************************************************************************
 * @file	smp_storage.h
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
#ifndef SMP_STORAGE_H_
#define SMP_STORAGE_H_



#if (SIMPLE_MULTI_MAC_EN)
	#include "smp.h"
	extern u8 device_mac_index;
#endif


#define		NEW_FLASH_STORAGE_KEY_ENABLE			0     //be compatible with old flash storage methods

#define		FLAG_SMP_PARAM_SAVE_OLD					0x5A  // 0101 1010  old storage

#if (SIMPLE_MULTI_MAC_EN)
#define		FLAG_SMP_PARAM_SAVE_BASE				(0x88+(device_mac_index&0x07))//0x8A  // 1000 1010
#else
														  // 10xx 1010  new storage,  xx: see "paring_sts_t" definition
#define		FLAG_SMP_PARAM_SAVE_BASE				0x8A  // 1000 1010
#endif

#define		FLAG_SMP_PARAM_SAVE_UNANTHEN			0x9A  // 1001 1010  new storage Unauthenticated_LTK
#define		FLAG_SMP_PARAM_SAVE_AUTHEN				0xAA  // 1010 1010  new storage Authenticated_LTK_Legacy_Paring
#define		FLAG_SMP_PARAM_SAVE_AUTHEN_SC			0xBA  // 1011 1010  new storage Authenticated_LTK_Secure_Connection

#define		FLAG_SMP_PARAM_SAVE_PENDING				0xBF  // 1011 1111
#define		FLAG_SMP_PARAM_SAVE_ERASE				0x00

#if (SIMPLE_MULTI_MAC_EN)
#define 	FLAG_SMP_PARAM_MASK						0xC8
#define     FLAG_SMP_PARAM_VALID					0x88
#else
#define 	FLAG_SMP_PARAM_MASK						0x0F  // 0000 1111
#define     FLAG_SMP_PARAM_VALID					0x0A  // 0000 1010
#endif

#define 	FLAG_SMP_PARING_STATUS_MASK				0x30  // 0011 1000

#define		FLAG_SMP_SECTOR_USE						0x3C
#define		FLAG_SMP_SECTOR_CLEAR					0x00
#define     FLASH_SECTOR_OFFSET						4080  //0xFF0



typedef enum {
	Index_Update_by_Pairing_Order = 0,     //default value
	Index_Update_by_Connect_Order = 1,
} index_updateMethod_t;


/*
 * smp parameter need save to flash.
 * */
typedef struct {  //82
	u8		flag;
	u8		peer_addr_type;  //address used in link layer connection
	u8		peer_addr[6];

	u8 		peer_key_size;
	u8		peer_id_adrType; //peer identity address information in key distribution, used to identify
	u8		peer_id_addr[6];


	u8 		own_ltk[16];      //own_ltk[16]
	u8		peer_irk[16];
	u8		peer_csrk[16];

}smp_param_save_t;


typedef struct {
	u8 paring_status;  		//if multiConnection, define as paring_status[connMaxNum]
	u8 addrIndex;
	u8 keyIndex;
	u8 cur_bondNum;

#if (SMP_DATABASE_INFO_SOURCE == SMP_INFO_STORAGE_IN_FLASH)
	u32 bond_flash_idx[SMP_BONDING_DEVICE_MAX_NUM];  //mark paired slave mac address in flash
	u8	bond_flag[SMP_BONDING_DEVICE_MAX_NUM];
	u32 index_update_method;
#else
	//may be sram address if use other mcu store smp info
#endif

} bond_device_t;

extern _attribute_aligned_(4) 	bond_device_t	 tbl_bondDevice;



/**
 * @brief      This function is used to get the number of currently bound devices.
 * @param[in]  none.
 * @return     The number of currently bound devices.
 */
u8			blc_smp_param_getCurrentBondingDeviceNumber(void);


/**
 * @brief      This function is used to configure the bonding storage address.
 * @param[in]  addr - SMP bonding storage start address.
 * @return     none.
 */
void 		bls_smp_configParingSecurityInfoStorageAddr (int addr);


/**
 * @brief      This function is used to obtain device binding information based on Index.
 * @param[in]  index - Device bonding index number.
 * @param[out] smp_param_load - The value can refer to the structure 'smp_param_save_t'.
 * @return     0: Failed to load binding information;
 *             others: FLASH address of the information area.
 */
u32 		bls_smp_param_loadByIndex(u8 index, smp_param_save_t* smp_param_load);


/**
 * @brief      This function is used to obtain binding information according to the master address and address type.
 * @param[in]  device_num - Set the maximum number of devices that can be bound.
 * @param[in]  adr_type - Address type.
 * @param[in]  addr - Address.
 * @param[out] smp_param_load - The value can refer to the structure 'smp_param_save_t'.
 * @return     0: Failed to load binding information;
 *             others: FLASH address of the information area.
 */
u32			bls_smp_param_loadByAddr(u8 addr_type, u8* addr, smp_param_save_t* smp_param_load);


/**
 * @brief      This function is used to configure the storage order of binding information.
 * @param[in]  method - The storage order of binding info method value can refer to the structure 'index_updateMethod_t'.
 *                      0: Index update by pairing order;
 *                      1: Index update by connect order.
 * @return     none.
 */
void		bls_smp_setIndexUpdateMethod(index_updateMethod_t method);


/**
 * @brief      This function is used for the slave device to clear all binding information stored in the local FLASH.
 * @param[in]  none.
 * @return     none.
 */
void		bls_smp_eraseAllParingInformation(void);



#endif /* SMP_STORAGE_H_ */
