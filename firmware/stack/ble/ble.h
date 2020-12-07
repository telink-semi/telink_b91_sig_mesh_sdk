/********************************************************************************************************
 * @file	ble.h
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
#ifndef BLE_H_
#define BLE_H_


#include "ble_config.h"
#include "ble_common.h"
#include "ble_format.h"

#include "l2cap/l2cap.h"
#include "phy/phy_stack.h"
#include "phy/phy.h"
#include "phy/phy_test.h"


#include "attr/att.h"
#include "attr/gatt.h"


#include "smp/smp.h"
#include "smp/smp_stack.h"
#include "smp/smp_central.h"
#include "smp/smp_peripheral.h"
#include "smp/smp_storage.h"

#include "gap/gap.h"
#include "gap/gap_event.h"

#include "crypt/aes_ccm.h"
#include "crypt/le_crypto.h"
#include "crypt/aes/aes_att.h"

#include "hci/hci.h"
#include "hci/hci_const.h"
#include "hci/hci_event.h"
#include "hci/usb_desc.h"

#include "stack/ble/service/ota.h"
#include "stack/ble/service/ota_stack.h"
#include "service/device_information.h"
#include "service/hids.h"
#include "service/uuid.h"

#include "ll/ll.h"
#include "ll/ll_stack.h"
#include "ll/ll_adv.h"
#include "ll/ll_init.h"
#include "ll/ll_pm.h"
#include "ll/ll_scan.h"
#include "ll/ll_whitelist.h"
#include "ll/ll_conn/ll_conn.h"
#include "ll/ll_conn/conn_stack.h"
#include "ll/ll_conn/ll_slave.h"
#include "ll/ll_conn/ll_master.h"
#include "ll/ll_conn/ll_conn_csa.h"

#include "ll/ll_ext_adv.h"


#include "stack/ble/trace.h"




volatile void  stmemset(register char * dest,register int val,register unsigned int len);
volatile void * stmemcpy(register char * out, register char * in, register unsigned int len);





#endif /* BLE_H_ */
