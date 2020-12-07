/********************************************************************************************************
 * @file	att.h
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
#include "tl_common.h"
#include "stack/ble/ble_config.h"
#include "stack/ble/ble_common.h"
#include "stack/ble/l2cap/l2cap.h"
#include "gatt.h"





/** @defgroup ATT_PERMISSIONS_BITMAPS GAP ATT Attribute Access Permissions Bit Fields
 * @{
 * (See the Core_v5.0(Vol 3/Part C/10.3.1/Table 10.2) for more information)
 */
#define ATT_PERMISSIONS_AUTHOR				 0x10 //Attribute access(Read & Write) requires Authorization
#define ATT_PERMISSIONS_ENCRYPT				 0x20 //Attribute access(Read & Write) requires Encryption
#define ATT_PERMISSIONS_AUTHEN				 0x40 //Attribute access(Read & Write) requires Authentication(MITM protection)
#define ATT_PERMISSIONS_SECURE_CONN			 0x80 //Attribute access(Read & Write) requires Secure_Connection
#define ATT_PERMISSIONS_SECURITY			 (ATT_PERMISSIONS_AUTHOR | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN | ATT_PERMISSIONS_SECURE_CONN)

//user can choose permission below
#define ATT_PERMISSIONS_READ                 0x01 //!< Attribute is Readable
#define ATT_PERMISSIONS_WRITE                0x02 //!< Attribute is Writable
#define ATT_PERMISSIONS_RDWR	             (ATT_PERMISSIONS_READ | ATT_PERMISSIONS_WRITE)   //!< Attribute is Readable & Writable


#define ATT_PERMISSIONS_ENCRYPT_READ         (ATT_PERMISSIONS_READ  | ATT_PERMISSIONS_ENCRYPT) 		//!< Read requires Encryption
#define ATT_PERMISSIONS_ENCRYPT_WRITE        (ATT_PERMISSIONS_WRITE | ATT_PERMISSIONS_ENCRYPT) 		//!< Write requires Encryption
#define ATT_PERMISSIONS_ENCRYPT_RDWR         (ATT_PERMISSIONS_RDWR  | ATT_PERMISSIONS_ENCRYPT) 		//!< Read & Write requires Encryption


#define ATT_PERMISSIONS_AUTHEN_READ          (ATT_PERMISSIONS_READ  | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN) 		//!< Read requires Authentication
#define ATT_PERMISSIONS_AUTHEN_WRITE         (ATT_PERMISSIONS_WRITE | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN) 		//!< Write requires Authentication
#define ATT_PERMISSIONS_AUTHEN_RDWR          (ATT_PERMISSIONS_RDWR  | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN) 		//!< Read & Write requires Authentication


#define ATT_PERMISSIONS_SECURE_CONN_READ	 (ATT_PERMISSIONS_READ  | ATT_PERMISSIONS_SECURE_CONN | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN)   //!< Read requires Secure_Connection
#define ATT_PERMISSIONS_SECURE_CONN_WRITE    (ATT_PERMISSIONS_WRITE | ATT_PERMISSIONS_SECURE_CONN | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN)  //!< Write requires Secure_Connection
#define ATT_PERMISSIONS_SECURE_CONN_RDWR	 (ATT_PERMISSIONS_RDWR  | ATT_PERMISSIONS_SECURE_CONN | ATT_PERMISSIONS_ENCRYPT | ATT_PERMISSIONS_AUTHEN)   //!< Read & Write requires Secure_Connection


#define ATT_PERMISSIONS_AUTHOR_READ          (ATT_PERMISSIONS_READ | ATT_PERMISSIONS_AUTHOR) 		//!< Read requires Authorization
#define ATT_PERMISSIONS_AUTHOR_WRITE         (ATT_PERMISSIONS_WRITE | ATT_PERMISSIONS_AUTHEN) 		//!< Write requires Authorization
#define ATT_PERMISSIONS_AUTHOR_RDWR          (ATT_PERMISSIONS_RDWR | ATT_PERMISSIONS_AUTHOR) 		//!< Read & Write requires Authorization

#define ATT_PERMISSIONS_READ_AUTHOR			 ATT_PERMISSIONS_AUTHOR_READ
#define ATT_PERMISSIONS_WRITE_AUTHOR		 ATT_PERMISSIONS_AUTHOR_WRITE
#define ATT_PERMISSIONS_RDWD_AUTHOR			 ATT_PERMISSIONS_AUTHOR_RDWR

/** @} End GAP_ATT_PERMISSIONS_BITMAPS */


/** @ add to group GATT_Characteristic_Property GATT characteristic properties
 * @{
 */
#define CHAR_PROP_BROADCAST              0x01 //!< permit broadcasts of the Characteristic Value
#define CHAR_PROP_READ                   0x02 //!< permit reads of the Characteristic Value
#define CHAR_PROP_WRITE_WITHOUT_RSP      0x04 //!< Permit writes of the Characteristic Value without response
#define CHAR_PROP_WRITE                  0x08 //!< Permit writes of the Characteristic Value with response
#define CHAR_PROP_NOTIFY                 0x10 //!< Permit notifications of a Characteristic Value without acknowledgement
#define CHAR_PROP_INDICATE               0x20 //!< Permit indications of a Characteristic Value with acknowledgement
#define CHAR_PROP_AUTHEN                 0x40 //!< permit signed writes to the Characteristic Value
#define CHAR_PROP_EXTENDED               0x80 //!< additional characteristic properties are defined
/** @} end of group GATT_Characteristic_Property */




typedef int (*att_readwrite_callback_t)(void* p);

typedef struct attribute
{
  u16  attNum;
  u8*  p_perm;
  u8   uuidLen;
  u32  attrLen;    //4 bytes aligned
  u8* uuid;
  u8* pAttrValue;
  att_readwrite_callback_t w;
  att_readwrite_callback_t r;
} attribute_t;




/**
 * @brief	This function is used to define ATT MTU size exchange callback
 */
typedef int (*att_mtuSizeExchange_callback_t)(u16, u16);

/**
 * @brief	This function is used to define ATT Handle value confirm callback
 */
typedef int (*att_handleValueConfirm_callback_t)(void);


/**
 * @brief	This function is used to set effective ATT MTU size
 * @param	connHandle - connect handle
 * @param	effective_mtu - bltAtt.effective_MTU
 * @return	none.
 */
void  blt_att_setEffectiveMtuSize(u16 connHandle, u8 effective_mtu);

/**
 * @brief	This function is used to reset effective ATT MTU size
 * @param	connHandle - connect handle
 * @return	none.
 */
void  blt_att_resetEffectiveMtuSize(u16 connHandle);


/**
 * @brief	This function is used to push notify data
 * @param	attHandle - ATT handle
 * @param	*p - the pointer of notify data
 * @param	len - the length of notify data
 * @return	0: success
 * 			other: fail
 */
ble_sts_t  	bls_att_pushNotifyData (u16 attHandle, u8 *p, int len);

/**
 * @brief	This function is used to push indicate data
 * @param	attHandle - ATT handle
 * @param	*p - the pointer of indicate data
 * @param	len - the length of indicate data
 * @return	0: success
 * 			other: fail
 */
ble_sts_t	 bls_att_pushIndicateData (u16 attHandle, u8 *p, int len);

/**
 * @brief	This function is used to set ATT table
 * @param	*p - the pointer of attribute table
 * @return	none.
 */
void		 bls_att_setAttributeTable (u8 *p);



//mtu size
/**
 * @brief	This function is used to set RX MTU size
 * @param	mtu_size - ATT MTU size
 * @return	0: success
 * 			other: fail
 */
ble_sts_t  blc_att_setRxMtuSize(u16 mtu_size);

/**
 * @brief	This function is used to request MTU size exchange
 * @param	connHandle - connect handle
 * @param	mtu_size - ATT MTU size
 * @return	0: success
 * 			other: fail
 */
ble_sts_t	 blc_att_requestMtuSizeExchange (u16 connHandle, u16 mtu_size);

/**
 * @brief	This function is used to response MTU size exchange
 * @param	mtu_size - ATT MTU size
 * @return	0: success
 * 			other: fail
 */
ble_sts_t	 blc_att_responseMtuSizeExchange (u16 connHandle, u16 mtu_size);


// 0x04: ATT_OP_FIND_INFO_REQ
/**
 * @brief	This function is used to find attribute information for request
 * @param	*dat - the data of request
 * @param	start_attHandle - the start of ATT handle
 * @param	end_attHandle - the end of ATT handle
 * @return	none.
 */
void 	att_req_find_info(u8 *dat, u16 start_attHandle, u16 end_attHandle);

// 0x06: ATT_OP_FIND_BY_TYPE_VALUE_REQ
/**
 * @brief	This function is used to find attribute by type value for request
 * @param	*dat - the data of request
 * @param	start_attHandle - the start of ATT handle
 * @param	end_attHandle - the end of ATT handle
 * @param	*uuid - the uuid of attribute
 * @param	attr_value - the value of attribute
 * @param	len - the length of attribute
 * @return	none.
 */
void 	att_req_find_by_type (u8 *dat, u16 start_attHandle, u16 end_attHandle, u8 *uuid, u8* attr_value, int len);

// 0x08: ATT_OP_READ_BY_TYPE_REQ
/**
 * @brief	This function is used to read attribute by type value for request
 * @param	*dat - the data of request
 * @param	start_attHandle - the start of ATT handle
 * @param	end_attHandle - the end of ATT handle
 * @param	*uuid - the uuid of attribute
 * @param	uuid_len - the length of UUID
 * @return	none.
 */
void 	att_req_read_by_type (u8 *dat, u16 start_attHandle, u16 end_attHandle, u8 *uuid, int uuid_len);

// 0x0a: ATT_OP_READ_REQ
/**
 * @brief	This function is used to read attribute value for request
 * @param	*dat - the data of request
 * @param	attHandle -  ATT handle
 * @return	none.
 */
void 	att_req_read (u8 *dat, u16 attHandle);

// 0x0c: ATT_OP_READ_BLOB_REQ
/**
 * @brief	This function is used to read part of attribute value for request
 * @param	*dat - the data of request
 * @param	attHandle -  ATT handle
 * @param	offset -  the offset of ATT attribute value
 * @return	none.
 */
void 	att_req_read_blob (u8 *dat, u16 attHandle, u16 offset);

// 0x10: ATT_OP_READ_BY_GROUP_TYPE_REQ
/**
 * @brief	This function is used to read attribute by group for request
 * @param	*dat - the data of request
 * @param	start_attHandle - the start of ATT handle
 * @param	end_attHandle - the end of ATT handle
 * @param	*uuid - the uuid of attribute
 * @param	uuid_len - the length of UUID
 * @return	none.
 */
void 	att_req_read_by_group_type (u8 *dat, u16 start_attHandle, u16 end_attHandle, u8 *uuid, int uuid_len);

// 0x12: ATT_OP_WRITE_REQ
/**
 * @brief	This function is used to write attribute value for request
 * @param	*dat - the data of request
 * @param	attHandle -  ATT handle
 * @param	*buf -  the data of ATT handle
 * @param	len -  the length of buffer
 * @return	none.
 */
void 	att_req_write (u8 *dat, u16 attHandle, u8 *buf, int len);

// 0x52: ATT_OP_WRITE_CMD
/**
 * @brief	This function is used to write command for request
 * @param	*dat - the data of request
 * @param	attHandle -  ATT handle
 * @param	*buf -  the data of ATT handle
 * @param	len -  the length of buffer
 * @return	none.
 */
void 	att_req_write_cmd (u8 *dat, u16 attHandle, u8 *buf, int len);





