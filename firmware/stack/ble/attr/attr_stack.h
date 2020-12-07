/********************************************************************************************************
 * @file	attr_stack.h
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
#ifndef STACK_BLE_ATTR_ATTR_STACK_H_
#define STACK_BLE_ATTR_ATTR_STACK_H_







#define ATT_MTU_SIZE                        23  //!< Minimum ATT MTU size




//TELINK MTU no longer than 256, so 1 byte is enough
typedef struct{
	u8 init_MTU;
	u8 effective_MTU;
	u8 Data_pending_time;    //10ms unit
	u8 Data_permission_check;
}att_para_t;
extern att_para_t bltAtt;






extern u16	blt_indicate_handle;








/************************* Stack Interface, user can not use!!! ***************************/
extern u32 att_service_discover_tick;

u8 * l2cap_att_handler(u16 connHandle, u8 * p);

static inline u16  blc_att_getEffectiveMtuSize(u16 connHandle)
{
	return bltAtt.effective_MTU;
}



void blt_att_procHoldAttributeCommand(void);













u8 blc_gatt_requestServiceAccess(u16 connHandle, int gatt_perm);






#endif /* STACK_BLE_ATTR_ATTR_STACK_H_ */
