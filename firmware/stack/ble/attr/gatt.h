/********************************************************************************************************
 * @file	gatt.h
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
#ifndef GATT_H_
#define GATT_H_




/************************************ User Interface  *****************************************/

/**
 * @brief	   This function is used to notify a client of the value of a Characteristic Value from a server.
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle  -  attribute handle.
 * @param[in]  *p -  data buffer pointer
 * @param[in]  len - data byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t	blc_gatt_pushHandleValueNotify  (u16 connHandle, u16 attHandle, u8 *p, int len);


/**
 * @brief	   This function is used to indicate the Characteristic Value from a server to a client.
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle - attribute handle.
 * @param[in]  *p -  data buffer pointer
 * @param[in]  len - data byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t	blc_gatt_pushHandleValueIndicate(u16 connHandle, u16 attHandle, u8 *p, int len);


/**
 * @brief	   This function is used to This function is used to request the server to write the value of an attribute without response.
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle - attribute handle.
 * @param[in]  *p -  data buffer pointer
 * @param[in]  len - data byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushWriteComand (u16 connHandle, u16 attHandle, u8 *p, int len);


/**
 * @brief	   This function is used to request the server to write the value of an attribute.
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle - attribute handle.
 * @param[in]  *p -  data buffer pointer
 * @param[in]  len - data byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushWriteRequest (u16 connHandle, u16 attHandle, u8 *p, int len);




/**
 * @brief	   This function is used to obtain the mapping of attribute handles with their associated types
 * 			   and allows a client to discover the list of attributes and their types on a server.
 * @param[in]  connHandle -  connection handle
 * @param[in]  start_attHandle - start attribute handle.
 * @param[in]  end_attHandle -  end attribute handle.
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushFindInformationRequest(u16 connHandle, u16 start_attHandle, u16 end_attHandle);


/**
 * @brief	   This function is used to obtain the handles of attributes that have a 16bit uuid attribute type and attribute value.
 * @param[in]  connHandle -  connection handle
 * @param[in]  start_attHandle - start attribute handle.
 * @param[in]  end_attHandle   - end attribute handle.
 * @param[in]  uuid
 * @param[in]  attr_value - attribute value
 * @param[in]  len - data byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushFindByTypeValueRequest (u16 connHandle, u16 start_attHandle, u16 end_attHandle, u16 uuid, u8* attr_value, int len);


/**
 * @brief	   This function is used to obtain the values of attributes where the attribute type is known but handle is not known.
 * @param[in]  connHandle -  connection handle
 * @param[in]  start_attHandle - start attribute handle.
 * @param[in]  end_attHandle   - end attribute handle.
 * @param[in]  uuid
 * @param[in]  uuid_len -uuid byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushReadByTypeRequest (u16 connHandle, u16 start_attHandle, u16 end_attHandle, u8 *uuid, int uuid_len);


/**
 * @brief	   This function is used to request the server to read the value of an attribute.
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle -   attribute handle.
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushReadRequest (u16 connHandle, u16 attHandle);


/**
 * @brief	   This function is used to request the server to read part of the value of an attribute at a given offset
 * @param[in]  connHandle -  connection handle
 * @param[in]  attHandle - attribute handle.
 * @param[in]  offset
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushReadBlobRequest (u16 connHandle, u16 attHandle, u16 offset);


/**
 * @brief	   This function is used to obtain the values of attributes according to the uuid.
 * @param[in]  connHandle -  connection handle
 * @param[in]  start_attHandle - start attribute handle.
 * @param[in]  end_attHandle   - end attribute handle.
 * @param[in]  uuid -
 * @param[in]  uuid_len - uuid byte number
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t 	blc_gatt_pushReadByGroupTypeRequest (u16 connHandle, u16 start_attHandle, u16 end_attHandle, u8 *uuid, int uuid_len);


#endif /* GATT_H_ */
