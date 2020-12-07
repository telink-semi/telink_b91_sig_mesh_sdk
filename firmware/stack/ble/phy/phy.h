/********************************************************************************************************
 * @file	phy.h
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
#ifndef PHY_H_
#define PHY_H_



#include "stack/ble/hci/hci_cmd.h"



/**
 * @brief      this function is used to initialize 2M/Coded PHY  feature
 * @param	   none
 * @return     none
 */
void		blc_ll_init2MPhyCodedPhy_feature(void);







/**
 * @brief       this function is used to set PHY type for connection
 * @param[in]	connHandle -
 * @param[in]	all_phys - preference PHY for TX & RX
 * @param[in]	tx_phys - preference PHY for TX
 * @param[in]	rx_phys - preference PHY for RX
 * @param[in]	phy_options - LE coding indication prefer
 * @return      none
 */
ble_sts_t  	blc_ll_setPhy(	u16 connHandle,					le_phy_prefer_mask_t all_phys,
							le_phy_prefer_type_t tx_phys, 	le_phy_prefer_type_t rx_phys,
							le_ci_prefer_t phy_options);





#endif /* PHY_H_ */
