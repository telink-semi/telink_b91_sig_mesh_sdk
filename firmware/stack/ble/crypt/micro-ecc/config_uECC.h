/********************************************************************************************************
 * @file	config_uECC.h
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
// Configuration of micro-ecc for use with BTstack
//
// We only need/use SECP256R1 for LE Secure Connections
#define uECC_CURVE uECC_secp256r1

// optimization: size vs. speed: uECC_asm_none - uECC_asm_small - uECC_asm_fast
#ifndef uECC_ASM
#define uECC_ASM uECC_asm_none
#endif

// don't use special square functions
#ifndef uECC_SQUARE_FUNC
#define uECC_SQUARE_FUNC 0
#endif
