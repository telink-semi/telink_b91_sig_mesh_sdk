/********************************************************************************************************
 * @file	smp_stack.h
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
#ifndef STACK_BLE_SMP_SMP_STACK_H_
#define STACK_BLE_SMP_SMP_STACK_H_



/******************************* smp start ***********************************************************************/

#define  KEY_FLAG_IDLE						0xFF
#define  KEY_FLAG_NEW						0xFE
#define  KEY_FLAG_FAIL						0xFD

#define  ADDR_NOT_BONDED					0xFF
#define  ADDR_NEW_BONDED					0xFE
#define  ADDR_DELETE_BOND					0xFD

#define	 SMP_TRANSPORT_SPECIFIC_KEY_START   0xEF
#define	 SMP_TRANSPORT_SPECIFIC_KEY_END     0

//Current not used
#define PASSKEY_TYPE_ENTRY_STARTED					0x00
#define PASSKEY_TYPE_DIGIT_ENTERED					0x01
#define PASSKEY_TYPE_DIGIT_ERASED					0x02
#define PASSKEY_TYPE_CLEARED						0x03
#define PASSKEY_TYPE_ENTRY_COMPLETED				0x04


typedef enum {
	JustWorks = 0,
	PK_Init_Dsply_Resp_Input = 1,
	PK_Resp_Dsply_Init_Input = 2,
	PK_BOTH_INPUT = 3,
	OOB_Authentication = 4,
	Numric_Comparison = 5,
} stk_generationMethod_t;


typedef union {
	struct{
		u8 bondingFlag : 2;
		u8 MITM : 1;
		u8 SC	: 1;
		u8 keyPress: 1;
		u8 rsvd: 3;
	};
	u8 authType;
}smp_authReq_t;


typedef struct{
	u8  code; //0x04
	u16 randomValue[16];
}smp_paring_random_t;


typedef struct{
	u8  code; //0x03
	u16 confirmValue[16];
}smp_paring_confirm_t;


typedef struct{
	u8 code; // 0x05
	u8 reason;
}smp_paring_failed_t;


typedef struct{
	u8 code;//0x0b

	smp_authReq_t authReq;
}smp_secure_req_t;


typedef struct{
	u8  code;//0xa
	u8 	signalKey[16];
}smp_signal_info_t;


typedef struct{
	u8 code;//0x9
	u8 addrType;
	u8 bdAddr[6];
}smp_id_addr_info_t;


typedef struct{
	u8 code;//0x8
	u8 idResolveKey[16];
}smp_id_info_t;


typedef struct{
	u8  code;//0x7
	u16 edivPtr[2];
	u8 	masterRand[8];
}smp_master_id_t;


typedef struct{
	u8  code;//0x6
	u8 	LTK[16];
}smp_enc_info_t;

// -------add core 4.2 ------
typedef struct{
	u8  code;//0xc
	u8  publicKeyX[32];
	u8  publicKeyY[32];
}smp_paring_public_key_t;


typedef struct{
	u8  code;//0xd
	u8  DHKeyCheck[16];
}smp_DHkey_check_t;


typedef struct{
	u8  code;//0xe
	u8  notifyType;
}smp_key_notify_t;


typedef struct{
	u8 sc_sk_dhk_own[32];  //  own  private key[32]
	u8 sc_pk_own[64];      //  own  public  key[64]
	u8 sc_pk_peer[64];     // peer  public  key[64]
}smp_sc_key_t;


typedef struct {
	/* data */
	u8 csrk[16];
	u32 signCounter;
} smp_secSigInfo_t;


typedef struct{
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 data[21];
}smp2llcap_type_t;





typedef union{
	struct {
		u8 encKey  : 1;
		u8 idKey   : 1;
		u8 sign    : 1;
		u8 linkKey : 4;
	};
	u8 keyIni;
}smp_keyDistribution_t;


typedef struct{
	u8 code;  				//req = 0x01; rsp = 0x02;
	u8 ioCapablity;
	u8 oobDataFlag;
	smp_authReq_t authReq;
	u8 maxEncrySize;

	smp_keyDistribution_t initKeyDistribution;
	smp_keyDistribution_t rspKeyDistribution;
}smp_paring_req_rsp_t;


/*
 * smp parameter about peer device.
 * */
typedef struct{
	u8		rsvd;
	u8 		peer_addr_type;  //address used in link layer connection
	u8		peer_addr[6];

	u8		peer_key_size;   //bond and key_size
	u8		peer_id_address_type;//peer identity address information in key distribution, used to identify
	u8		peer_id_address[6];

	u8		paring_peer_rand[16];
	u8		paring_confirm[16];  //in security connection oob mode to keep peer random

	u8		peer_csrk[16];
	u8		peer_irk[16];
}smp_param_peer_t;


/*
 * smp parameter about own device.
 * */
typedef struct{
	u8						paring_tk[16]; // in security connection to keep own random
	u8						own_ltk[16];   //used for generate ediv and random

	smp_paring_req_rsp_t  	paring_req;
	smp_paring_req_rsp_t  	paring_rsp;
	smp_authReq_t			auth_req;
	u8						own_conn_type;  //current connection peer own type

	u16						rsvd;
	u8						own_conn_addr[6];
}smp_param_own_t;



#if (SECURE_CONNECTION_ENABLE)
	extern const u8 PublicKey[64];
	extern const u8 PrivateKey[32];
	typedef u8 * (*smp_sc_cmd_handler_t)(u16 conn, u8*p);
	typedef void (*smp_sc_pushPkt_handler_t)(int type);
	extern _attribute_aligned_(4) smp_sc_key_t	smp_sc_key;
	extern smp_sc_cmd_handler_t		func_smp_sc_proc;
	extern smp_sc_pushPkt_handler_t	func_smp_sc_pushPkt_proc;
#endif

typedef void (*smp_check_handler_t)(u32);
typedef u8 * (*smp_info_handler_t)(u16 conn);
typedef int  (*smp_init_handler_t)(u16 conn, u8 *p);
typedef int  (*smp_finish_callback_t)(void);
typedef void (*smp_bond_clean_handler_t)(void);
typedef void (*smp_trigger_cb_t)(u16 conn_handle);


extern u8 	smpPkShftCnt;
extern u8 	smpDistirbuteKeyOrder;
extern u32 	smp_timeout_start_tick;
extern u8 	smpParingOwnIRK[16];
extern u8 	smpOwnRand_const[16];
extern u8 	smpParingOwnCSRK[16];
extern u8 	smpOwnParingConfirm[16];
extern smp2llcap_type_t smpResSignalPkt;

extern smp_keyDistribution_t	smp_DistributeKeyInit ;
extern smp_keyDistribution_t	smp_DistributeKeyResp ;


extern _attribute_aligned_(4) 	smp_param_peer_t smp_param_peer;
extern _attribute_aligned_(4) 	smp_param_own_t  smp_param_own;

extern smp_finish_callback_t	blc_smpFinishCb;
extern smp_info_handler_t		func_smp_info;
extern smp_init_handler_t		func_smp_init;
extern smp_check_handler_t		func_smp_check; //HID on android 7.0



static inline void	blc_smp_setCertTimeoutTick (u32 t)
{
	smp_timeout_start_tick = t;
}


u8* 		blc_smp_sc_handler(u16 conn, u8*p);
void 		blc_smp_sc_pushPkt_handler(int type);
u8* 		blc_smp_pushParingFailed(u8 failReason);
void 		blc_smp_generate_initial_keys(void);
int 		blc_smp_getGenMethod (int SC_en);    //Return STK generate method.
int 		blc_smp_param_init (void);
void 		blc_stack_smp_setBondingMode(u8 mode); //'mode' type: bonding_mode_t
void 		blc_stack_smp_setParingMethods(u8  method); //'method' type: paring_methods_t
void 		blc_stack_smp_enableAuthMITM(int MITM_en);
void 		blc_stack_smp_enableOobAuthentication(int OOB_en);
void 		blc_stack_smp_setIoCapability(u8  ioCapablility); //'ioCapablility' type: io_capability_t
void 		blc_stack_smp_enableKeypress(int keyPress_en);
void 		blc_smp_setInitiatorKey(u8 LTK_distributeEn, u8 IRK_distributeEn, u8 CSRK_DistributeEn);
void 		blc_smp_setResponderKey(u8 LTK_distributeEn, u8 IRK_distributeEn, u8 CSRK_DistributeEn);
int 		blc_stack_smp_setSecurityParamters(u8 mode, u8 method, u8 MITM_en, u8 OOB_en, u8 ioCapablility, u8 keyPress_en); //'mode' type: bonding_mode_t; 'method' type: paring_methods_t

/******************************* smp end *************************************************************************/


/**
 * @brief      This function is used to register the callback function that is triggered when the SMP is completed.
 * @param[in]  cb - Callback function triggered when SMP is completed.
 * @return     none.
 */
void 	blm_smp_registerSmpFinishCb (smp_finish_callback_t cb);



/******************************* smp_peripheral start ***********************************************************************/

//Local Device Paring Status
//See the Core_v5.0(Vol 3/Part C/10.3.1/Table 10.2) for more information.
typedef enum {
	No_Bonded_No_LTK = 0,    //No_LTK
	Unauthenticated_LTK,
	Authenticated_LTK_Legacy_Paring,
	Authenticated_LTK_Secure_Connection,
} paring_sts_t;

typedef struct{
	u8  secReq_conn;    //BIT<7:4> reConn;  BIT<3:0> newConn;
	u8  secReq_pending;
	u16 pending_ms;
}secReq_ctl_t;

extern	_attribute_aligned_(4)	secReq_ctl_t	blc_SecReq_ctrl;

u8 *		 bls_smp_pushPkt (int type);
int 		 bls_smp_encryption_done(u16 connHandle);
void 		 bls_smp_peripheral_paring_loop(void);
paring_sts_t bls_smp_get_paring_statas(u16 connHandle);

void 		 blc_smp_procParingEnd(u8 err_reason);

u8 * 		 l2cap_smp_handler(u16 connHandle, u8 * p);

/******************************* smp_peripheral end *************************************************************************/






/******************************* smp_central start ***********************************************************************/

#define FlAG_BOND				BIT(0)
#define FLAG_FASTSMP			BIT(4)

typedef struct{
	u8  trigger_mask;
	u8  smp_begin_flg;
	u8  manual_smp_start;
	u8  rsvd;
}smp_trigger_t;

typedef struct {
	u8 bond_mark;
	u8 adr_type;
	u8 address[6];
} mac_adr_t;


void 	blm_host_smp_procSecurityTrigger(u16 connHandle);
void	bond_slave_flash_clean (void);

/******************************* smp_central end *************************************************************************/






/******************************* smp_storage start ***********************************************************************/

typedef enum {
	FLASH_OP_SUCCESS = 0,
	FLASH_OP_FAIL 	 = 1,
} flash_op_sts_t;

extern	int	SMP_PARAM_NV_ADDR_START;
extern	int	bond_device_flash_cfg_idx;

#define	SMP_PARAM_NV_UNIT						64
#define	SMP_PARAM_INIT_CLEAR_MAGIN_ADDR 		3072  //64 * 48
#define	SMP_PARAM_LOOP_CLEAR_MAGIN_ADDR 		3520  //64 * 55 (56 device most)
#define	SMP_PARAM_NV_MAX_LEN					4096
#define	SMP_PARAM_NV_SEC_ADDR_START				(SMP_PARAM_NV_ADDR_START + SMP_PARAM_NV_MAX_LEN)
#define	SMP_PARAM_NV_SEC_ADDR_END				(SMP_PARAM_NV_SEC_ADDR_START + SMP_PARAM_NV_MAX_LEN - 1)


void 	bls_smp_param_initFromFlash(void);
void	bls_smp_param_Cleanflash (void);
u32 	bls_smp_loadParamVsRand (u16 ediv, u8* random);
int     bls_smp_loadParamVsAddr (u16 addr_type, u8* addr);
u8		bls_smp_param_getIndexByFLashAddr(u32 flash_addr);
int		bls_smp_param_deleteByIndex(u8 index);
u8		bls_smp_param_getBondFlag_by_flashAddress(u32 flash_addr);
u32		bls_smp_searchBondingDevice_in_Flash_by_Address(u8 addr_type, u8* addr );
u32 	blc_smp_param_updateToNearestByIndex(u8 index);

/******************************* smp_storage end *************************************************************************/

#endif /* STACK_BLE_SMP_SMP_STACK_H_ */
