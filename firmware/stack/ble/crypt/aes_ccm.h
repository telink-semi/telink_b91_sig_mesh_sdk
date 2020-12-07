/********************************************************************************************************
 * @file	aes_ccm.h
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
#pragma  once

#include <stack/ble/ll/ll.h>
#include "stack/ble/ll/ll_conn/conn_stack.h"

#define AES_BLOCK_SIZE     16


//#define		SUCCESS			0
enum {
    AES_SUCC = SUCCESS,
    AES_NO_BUF,               
    AES_FAIL,
};


struct CCM_FLAGS_TAG {
    union {
        struct {
            u8 L : 3;
            u8 M : 3;
            u8 aData :1;
            u8 reserved :1;            
        } bf;
        u8 val;
    };
};

typedef struct CCM_FLAGS_TAG ccm_flags_t;


typedef struct {
    union {
        u8 A[AES_BLOCK_SIZE];
        u8 B[AES_BLOCK_SIZE];
    } bf;
    
    u8 tmpResult[AES_BLOCK_SIZE];
    u8 newAstr[AES_BLOCK_SIZE];
} aes_enc_t;

/**
 * @brief   	this function is used to calculate the aes_ccm value
 * @param[in]   micLen - mic_lenth (should be 4)
 * @param[in]   *key - aes key
 * @param[in]   iv - initial vector (should be 13 bytes nonce)
 * @param[in]   mStr - plaint_text
 * @param[in]   mStrLen - plaint_text length
 * @param[in]   aStr -  a string  (should be AAD the data channel PDU header first octet with NESN, SN and MD bits masked to 0)
 * @param[in]   aStrLen - a string length (should be 1)
 * @param[in]   result - result (result)
 * @return  	0
 */
u8 aes_ccmAuthTran(u8 micLen, u8 *key, u8 *iv, u8 *mStr, u16 mStrLen, u8 *aStr, u16 aStrLen, u8 *result);

/**
 * @brief   	this function is used to decrypt the aes_ccm value
 * @param[in]   micLen - mic_lenth (should be 4)
 * @param[in]   *key - aes key
 * @param[in]   iv - initial vector (should be 13 bytes nonce)
 * @param[in]   mStr - plaint_text
 * @param[in]   mStrLen - plaint_text length
 * @param[in]   aStr -  a string  (should be AAD the data channel PDU header first octet with NESN, SN and MD bits masked to 0)
 * @param[in]   aStrLen - a string length (should be 1)
 * @param[in]   *mic - compare with the result of aes_ccmAuthTran
 * @return  	0 : success
 * 				-1 : fail
 */
u8 aes_ccmDecAuthTran(u8 micLen, u8 *key, u8 *iv, u8 *mStr, u16 mStrLen, u8 *aStr, u8 aStrLen, u8 *mic);

/**
 * @brief   	this function is used to initialize the key for AES module
 * @param[in]   *key - aes key
 * @return  	0 : success
 */
u8 aes_initKey(u8 *key);

/**
 * @brief   	this function is used to encrypt the aes_ccm value
 * @param[in]   *key - aes key
 * @param[in]   iv - initial vector (should be 13 bytes nonce)
 * @param[in]   aStr -  a string  (should be AAD the data channel PDU header first octet with NESN, SN and MD bits masked to 0)
 * @param[in]   *mic - the mic of aes_ccmAuthTran
 * @param[in]   mStrLen - plaint_text length
 * @param[in]   *mStr - plaint_text
 * @param[in]   *result - return mStr
 * @return  	0 : success
 */
u8 aes_ccm_encryption(u8 *key, u8 *iv, u8 *aStr, u8 *mic, u8 mStrLen, u8 *mStr, u8 *result);

/**
 * @brief   	this function is used to decrypt the aes_ccm value
 * @param[in]   *key - aes key
 * @param[in]   iv - initial vector (should be 13 bytes nonce)
 * @param[in]   aStr -  a string  (should be AAD the data channel PDU header first octet with NESN, SN and MD bits masked to 0)
 * @param[in]   *mic - the mic of aes_ccmAuthTran
 * @param[in]   mStrLen - plaint_text length
 * @param[in]   *mStr - plaint_text
 * @param[in]   *result - return mStr
 * @return  	0 : success
 */
u8 aes_ccm_decryption(u8 *key, u8 *iv, u8 *aStr, u8 *mic, u8 mStrLen, u8 *mStr, u8 *result);




/**
 * @brief   	this function is used to decrypt the aes_ccm value
 * @param[in]   *key - aes key
 * @param[in]   *plaintext - plaint_text
 * @param[in]   *result - return encrypt result
 * @return  	none.
 */
void aes_ll_encryption(u32* key, u32* plaintext, u8 *result);

/**
 * @brief   	this function is used to encrypt the packet for ATT
 * @param[in]   *key - aes key
 * @param[in]   *iv - initial vector
 * @param[in]   *mic - the mic of aes_ccmAuthTran
 * @param[in]   *micLen - mic_lenth
 * @param[in]   *ps - att packet
 * @param[in]   len - the length of packet
 * @return  	1
 */
u8 aes_att_encryption_packet(u8 *key, u8 *iv, u8 *mic, u8 mic_len, u8 *ps, u8 len);

/**
 * @brief   	this function is used to decrypt the packet for ATT
 * @param[in]   *key - aes key
 * @param[in]   *iv - initial vector
 * @param[in]   *mic - the mic of aes_ccmAuthTran
 * @param[in]   *micLen - mic_lenth
 * @param[in]   *ps - att packet
 * @param[in]   len - the length of packet
 * @return  	1
 */
u8 aes_att_decryption_packet(u8 *key, u8 *iv, u8 *mic, u8 mic_len, u8 *ps, u8 len);

/**
 * @brief   	this function is used to encrypt the packet for link layer
 * @param[in]   *key - aes key
 * @param[in]   *r - the plaintext
 * @param[in]   *p1 - packet buffer2
 * @param[in]   *p2 - packet buffer2
 * @param[in]   *result - the result of encrypt
 * @return  	none.
 */
void aes_ll_c1(u8 * key, u8 * r, u8 *p1, u8 *p2, u8 * result);

/**
 * @brief   	this function is used to encrypt the packet for link layer
 * @param[in]   *key - aes key
 * @param[in]   *r1 - the plaintext1
 * @param[in]   *r2 - the plaintext2
 * @param[in]   *result - the result of encrypt
 * @return  	none.
 */
void aes_ll_s1(u8 * key, u8 * r1, u8 * r2, u8 * result);

void aes_ecb_encryption(u8 *key, u8 len, u8 *plaintext, u8 *encrypted_data);

void aes_ecb_decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data);

