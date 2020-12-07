/********************************************************************************************************
 * @file	smp.h
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
#ifndef BLE_SMP_H_
#define BLE_SMP_H_

#include "stack/ble/ble_common.h"
#include "smp_stack.h"



#if (SIMPLE_MULTI_MAC_EN)
	#define	SMP_BONDING_DEVICE_MAX_NUM				16
#else
	#define	SMP_BONDING_DEVICE_MAX_NUM				4
#endif


#ifndef	SIMPLE_MULTI_MAC_EN
#define SIMPLE_MULTI_MAC_EN							0
#endif


/** @addtogroup SMP info storage type definition
 * @{
 */
#define SMP_INFO_STORAGE_IN_FLASH					1
#define SMP_INFO_STORAGE_IN_EEPROM					2
#define SMP_INFO_STORAGE_OTHER_MCU	 				3
/** @} end of group SMP info storage type */


#define SMP_STANDARD_PAIR   						0
#define SMP_FAST_CONNECT   							1


//define smp database(keys/peer device informations ...)
#ifndef SMP_DATABASE_INFO_SOURCE
#define SMP_DATABASE_INFO_SOURCE					SMP_INFO_STORAGE_IN_FLASH
#endif


/** @addtogroup SMP pairing opcode definition
 * @{
 */
#define SMP_OP_PAIRING_REQ							1
#define SMP_OP_PAIRING_RSP							2
#define SMP_OP_PAIRING_CONFIRM						3
#define SMP_OP_PAIRING_RANDOM						4
#define SMP_OP_PAIRING_FAIL							5
#define SMP_OP_ENC_INFO								6
#define SMP_OP_ENC_IDX								7
#define SMP_OP_ENC_IINFO							8
#define SMP_OP_ENC_IADR								9
#define SMP_OP_ENC_SIGN								0x0a
#define	SMP_OP_SEC_REQ								0x0b
#define SMP_OP_PARING_PUBLIC_KEY					0x0c
#define SMP_OP_PARING_DHKEY							0x0d
#define SMP_OP_KEYPRESS_NOTIFICATION				0x0e
#define SMP_OP_WAIT									0x0f
#define SMP_OP_ENC_END								0xFF //TLK defined
/** @} end of group SMP pairing opcode */


/** @addtogroup SMP pairing fail reason definition
 * @{
 */
#define PARING_FAIL_REASON_PASSKEY_ENTRY			0x01
#define PARING_FAIL_REASON_OOB_NOT_AVAILABLE		0x02
#define PARING_FAIL_REASON_AUTH_REQUIRE				0x03
#define PARING_FAIL_REASON_CONFIRM_FAILED			0x04
#define PARING_FAIL_REASON_PARING_NOT_SUPPORTED		0x05
#define PARING_FAIL_REASON_ENCRYPT_KEY_SIZE			0x06
#define PARING_FAIL_REASON_CMD_NOT_SUPPORT			0x07 //-- core 4.2
#define PARING_FAIL_REASON_UNSPECIFIED_REASON		0x08
#define PARING_FAIL_REASON_REPEATED_ATTEMPT			0x09
#define PARING_FAIL_REASON_INVAILD_PARAMETER		0x0A
#define PARING_FAIL_REASON_DHKEY_CHECK_FAIL			0x0B
#define PARING_FAIL_REASON_NUMUERIC_FAILED			0x0C
#define PARING_FAIL_REASON_BREDR_PARING				0x0D
#define PARING_FAIL_REASON_CROSS_TRANSKEY_NOT_ALLOW	0x0E
#define PARING_FAIL_REASON_PARING_TIEMOUT			0x80 //TLK defined
#define PARING_FAIL_REASON_CONN_DISCONNECT			0x81 //TLK defined
#define PARING_FAIL_REASON_SUPPORT_NC_ONLY			0x82 //TLK defined
/** @} end of group SMP pairing fail reasone */


/** @addtogroup SMP encryption key size definition
 * @{
 */
#define	ENCRYPRION_KEY_SIZE_MAXINUM					16
#define	ENCRYPRION_KEY_SIZE_MINIMUN					7
/** @} end of group SMP encryption key size */


/** @addtogroup SMP TK status definition
 * @{
 */
#define TK_ST_REQUEST								BIT(0)
#define TK_ST_UPDATE								BIT(1)
#define TK_ST_CONFIRM_PENDING						BIT(2)
#define TK_ST_NUMERIC_COMPARE						BIT(3)
#define TK_ST_NUMERIC_CHECK_YES					    BIT(4)
#define TK_ST_NUMERIC_CHECK_NO					    BIT(5)
#define TK_ST_NUMERIC_DHKEY_FAIL_PENDING			BIT(6)
#define TK_ST_NUMERIC_DHKEY_SUCC_PENDING			BIT(7)
/** @} end of group SMP TK status */


#define	MASTER_SMP_EVENT							1

//See the Core_v5.0(Vol 3/Part C/10.2, Page 2067) for more information.
typedef enum {
	LE_Security_Mode_1_Level_1 = BIT(0),  No_Authentication_No_Encryption			= BIT(0), No_Security = BIT(0),
	LE_Security_Mode_1_Level_2 = BIT(1),  Unauthenticated_Paring_with_Encryption 	= BIT(1),
	LE_Security_Mode_1_Level_3 = BIT(2),  Authenticated_Paring_with_Encryption 	    = BIT(2),
	LE_Security_Mode_1_Level_4 = BIT(3),  Authenticated_LE_Secure_Connection_Paring_with_Encryption = BIT(3),

	LE_Security_Mode_2_Level_1 = BIT(4),  Unauthenticated_Paring_with_Data_Signing 	= BIT(4),
	LE_Security_Mode_2_Level_2 = BIT(5),  Authenticated_Paring_with_Data_Signing    = BIT(5),

	LE_Security_Mode_1 = (LE_Security_Mode_1_Level_1 | LE_Security_Mode_1_Level_2 | LE_Security_Mode_1_Level_3 | LE_Security_Mode_1_Level_4)
}le_security_mode_level_t;


typedef enum {
	non_debug_mode 	= 0,  // ECDH distribute private/public key pairs
	debug_mode 		= 1,  // ECDH use debug mode private/public key pairs
} ecdh_keys_mode_t;


typedef enum {
	Non_Bondable_Mode = 0,
	Bondable_Mode     = 1,
}bonding_mode_t;


//Paring Methods select
//See the Core_v5.0(Vol 3/Part H/2.3) for more information.
typedef enum {
	LE_Legacy_Paring     = 0,   // BLE 4.0/4.2
	LE_Secure_Connection = 1,   // BLE 4.2/5.0/5.1
}paring_methods_t;


typedef enum {
	IO_CAPABILITY_UNKNOWN 				= 0xff,
	IO_CAPABILITY_DISPLAY_ONLY 			= 0,
	IO_CAPABILITY_DISPLAY_YESNO 		= 1,
	IO_CAPABILITY_KEYBOARD_ONLY 		= 2,
	IO_CAPABILITY_NO_IN_NO_OUT 	        = 3,
	IO_CAPABILITY_KEYBOARD_DISPLAY 		= 4,
} io_capability_t;

typedef struct {
	u8 security_level;
	u8 bonding_maxNum;
	u8 bonding_mode;
	u8 IO_capability;

	u32 ecdh_debug_mode;	//1:debug_mode,0:ecdh public/private key pairs distribute

	u8 MITM_protetion;
	u8 oob_enable;
	u8 paring_method;  		//paring_method
	u8 keyPress_en;

	u8 secure_conn; 		// final paring method is paring_method or not
	u8 stk_method;  		// final available stk generate method
	u8 tk_status;
	u8 peerKey_mask;    	//determine which peer key to store   <0>: EncKey  <1>:IdKey   <2>:Sign, NOTE: not used now in code

	u8 bonding_enable;
	u8 paring_busy;
	u8 key_distribute;
	u8 save_key_flag;
} smp_mng_t;

extern _attribute_aligned_(4)	smp_mng_t        blc_smpMng;

/**
 * @brief      This function is used to set the flag in SMP pairing process.
 * @param[in]  busy - Set the SMP pairing flag.
 * 					  1:is pair busy
 * 					  0:isn't pair busy
 * @return     none.
 */
static inline  void	blc_smp_setParingBusy(u8 busy)
{
	blc_smpMng.paring_busy = busy;
}

/**
 * @brief      This function is used to set the flag in SMP pairing process.
 * @param[in]  none.
 * @return     1:is pair busy
 * 			   0:isn't pair busy
 */
static inline int	blc_smp_isParingBusy(void)
{
	return blc_smpMng.paring_busy;
}

/**
 * @brief      This function is used to set the maximum number of devices that can be bound.
 * @param[in]  device_num - Set the maximum number of devices that can be bound.
 * @return     none.
 */
void 		blc_smp_param_setBondingDeviceMaxNumber ( int device_num);


/**
 * @brief      This function is used to set security level.
 * @param[in]  mode_level - The security level value can refer to the structure 'le_security_mode_level_t'.
 * @return     none.
 */
void 		blc_smp_setSecurityLevel(le_security_mode_level_t  mode_level);


/**
 * @brief      This function is used to enable ECDH to generate public key-private key pairs in advance.
 * @param[in]  enable - 1: Turn on ECDH to generate public key-private key pairs in advance.
 *                      0: Disable this mode.
 * @return     none.
 */
void 		blc_smp_preMakeEcdhKeysEnable(u8 enable);


/**
 * @brief      This function is used to set paring method.
 * @param[in]  method - The paring method value can refer to the structure 'paring_methods_t'.
 *                      0: LE legacy paring;
 *                      1: LE secure connection
 * @return     none.
 */
void 		blc_smp_setParingMethods (paring_methods_t  method);       //select paring methods


/**
 * @brief      This function is used to set whether the device uses the ECDH DEBUG key.
 * @param[in]  mode - The ECDH key mode value can refer to the structure 'ecdh_keys_mode_t'.
 *                    0: non debug mode;
 *                    1: debug mode.
 * @return     none.
 */
void 		blc_smp_setEcdhDebugMode(ecdh_keys_mode_t mode);


/**
 * @brief      This function is used to set bonding mode.
 * @param[in]  mode - The bonding mode value can refer to the structure 'bonding_mode_t'.
 *                    0: non bondable mode;
 *                    1: bondable mode.
 * @return     none.
 */
void 		blc_smp_setBondingMode(bonding_mode_t mode);			   //set bonding_mode


/**
 * @brief      This function is used to set if enable authentication MITM protection.
 * @param[in]  MITM_en - 0: Disable MITM protection;
 *                       1: Enable MITM protection.
 * @return     none.
 */
void 		blc_smp_enableAuthMITM (int MITM_en);


/**
 * @brief      This function is used to set if enable OOB authentication.
 * @param[in]  OOB_en - 0: Disable OOB authentication;
 *                      1: Enable OOB authentication.
 * @return     none.
 */
void 		blc_smp_enableOobAuthentication (int OOB_en);    		   //enable OOB authentication


/**
 * @brief      This function is used to set device's IO capability.
 * @param[in]  ioCapablility - The IO capability's value can refer to the structure 'io_capability_t'.
 * @return     none.
 */
void 		blc_smp_setIoCapability (io_capability_t  ioCapablility);


/**
 * @brief      This function is used to set device's Keypress.
 * @param[in]  keyPress_en - 0: Disable Keypress;
 *                           1: Enable Keypress.
 * @return     none.
 */
void 		blc_smp_enableKeypress (int keyPress_en);


/**
 * @brief      This function is used to set device's security parameters.
 * @param[in]  mode - The bonding mode value can refer to the structure 'bonding_mode_t'.
 * @param[in]  MITM_en - 0: Disable MITM protection;  1: Enable MITM protection.
 * @param[in]  OOB_en - 0: Disable OOB authentication; 1: Enable OOB authentication.
 * @param[in]  keyPress_en - 0: Disable Keypress; 1: Enable Keypress.
 * @param[in]  ioCapablility - The IO capability's value can refer to the structure 'io_capability_t'.
 * @return     none.
 */
void 		blc_smp_setSecurityParamters (bonding_mode_t mode, int MITM_en, int OOB_en,
										  int keyPress_en, io_capability_t ioCapablility);


/**
 * @brief      This function is used to set TK by OOB method.
 * @param[in]  oobData - TK's value, size: 16 byte.
 * @return     none.
 */
void 		blc_smp_setTK_by_OOB (u8 *oobData);


/**
 * @brief      This function is used to set TK by passkey entry method.
 * @param[in]  pinCodeInput - TK's value, input range [000000, 999999].
 * @return     none.
 */
void 		blc_smp_setTK_by_PasskeyEntry (u32 pinCodeInput);


/**
 * @brief      This function is used to set numeric compare confirm YES or NO.
 * @param[in]  YES_or_NO - 1: numeric compare confirm YES;
 *                         0: numeric compare confirm NO.
 * @return     none.
 */
void		blc_smp_setNumericComparisonResult(bool YES_or_NO);



#endif /* BLE_SMP_H_ */
