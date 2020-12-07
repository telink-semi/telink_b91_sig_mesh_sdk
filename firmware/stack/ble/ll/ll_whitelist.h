/********************************************************************************************************
 * @file	ll_whitelist.h
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
#ifndef LL_WHITELIST_H_
#define LL_WHITELIST_H_




/**
 * @brief      reset whitelist
 * @param[in]  none
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t ll_whiteList_reset(void);


/**
 * @brief      add a device form whitelist
 * @param[in]  type - device mac address type
 * @param[in]  addr - device mac address
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t ll_whiteList_add(u8 type, u8 *addr);


/**
 * @brief      delete a device from whitelist
 * @param[in]  type - device mac address type
 * @param[in]  addr - device mac address
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t ll_whiteList_delete(u8 type, u8 *addr);




/**
 * @brief      get whitelist size
 * @param[out] pointer to size
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t ll_whiteList_getSize(u8 *returnPublicAddrListSize) ;




/**
 * @brief      reset resolvinglist
 * @param[in]  none
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t  ll_resolvingList_reset(void);


/**
 * @brief      add a device to resolvinglist
 * @param[in]  peerIdAddrType - device mac address type
 * @param[in]  peerIdAddr - device mac address
 * @param[in]  peer_irk - peer IRK pointer
 * @param[in]  local_irk - local IRK pointer
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t  ll_resolvingList_add(u8 peerIdAddrType, u8 *peerIdAddr, u8 *peer_irk, u8 *local_irk);


/**
 * @brief      delete a device from resolvinglist
 * @param[in]  peerIdAddrType - device mac address type
 * @param[in]  peerIdAddr - device mac address
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t  ll_resolvingList_delete(u8 peerIdAddrType, u8 *peerIdAddr);


/**
 * @brief      enable resolvinglist resolution
 * @param[in]  resolutionEn - 1: enable; 0:disable
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t  ll_resolvingList_setAddrResolutionEnable (u8 resolutionEn);




#endif /* LL_WHITELIST_H_ */
