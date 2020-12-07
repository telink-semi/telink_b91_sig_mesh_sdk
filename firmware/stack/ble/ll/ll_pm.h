/********************************************************************************************************
 * @file	ll_pm.h
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
/*
 * ll_pm.h
 *
 *  Created on: 2017-3-7
 *      Author: Administrator
 */

#ifndef LL_PM_H_
#define LL_PM_H_




////////////////// Power Management ///////////////////////
/**
 * @brief	Telink defined Low power state Type
 */
#define			SUSPEND_DISABLE				0
#define			SUSPEND_ADV					BIT(0)
#define			SUSPEND_CONN				BIT(1)
#define			DEEPSLEEP_RETENTION_ADV		BIT(2)
#define			DEEPSLEEP_RETENTION_CONN	BIT(3)





typedef struct {
	u8		suspend_mask;
	u8		wakeup_src;
	u8		conn_no_suspend;
	u8 		timer_wakeup;

	u8		latency_off;
	u8		no_latency;
	u16 	latency_en;

	u16		sys_latency;
	u16 	user_latency;
	u16		valid_latency;
	u16 	latency_use;

	u32     deepRet_advThresTick;
	u32     deepRet_connThresTick;
	u32     deepRet_earlyWakeupTick;

	u8 		deepRt_en;
	u8		deepRet_type;
	u8		appWakeup_en;
	u8		appWakeup_flg;

	u8		appWakeup_loop_noLatency;
	u8 		timing_miss;
	u8      timing_synced;
	u8 		rsvd;

	u32 	appWakeup_tick;

	u32     current_wakeup_tick; //The system wake-up tick of the actual transfer of the cpu_sleep_wakeup function.

	u16		pm_border_flag;
	u16     rsvd1;

}st_ll_pm_t;

extern _attribute_aligned_(4) st_ll_pm_t  bltPm;

/**
 * @brief	Telink defined ble stack low power mode process callback function
 */
typedef 	void (*ll_module_pm_callback_t)(void);

/**
 * @brief	Telink defined application wake up low power mode process callback function
 */
typedef 	void (*pm_appWakeupLowPower_callback_t)(int);




/******************************* User Interface  ************************************/
/**
 * @brief	for user to initialize low power mode
 * @param	none
 * @return	none
 */
void 		blc_ll_initPowerManagement_module(void);

/**
 * @brief	for user to set low power mode mask
 * @param	mask - low power mode mask
 * @return	none
 */
void		bls_pm_setSuspendMask (u8 mask);

/**
 * @brief	for user to get low power mode mask
 * @param	none
 * @return	bltPm.suspend_mask
 */
u8 			bls_pm_getSuspendMask (void);

/**
 * @brief	for user to set low power mode wake up source
 * @param	source - low power mode wake up source
 * @return	none
 */
void 		bls_pm_setWakeupSource(u8 source);

/**
 * @brief	for user to get low power mode wake up time
 * @param	none
 * @return	bltPm.current_wakeup_tick
 */
u32 		bls_pm_getSystemWakeupTick(void);

/**
 * @brief	for user to set latency manually for save power
 * @param	latency - bltPm.user_latency
 * @return	none
 */
void 		bls_pm_setManualLatency(u16 latency); //manual set latency to save power

/**
 * @brief	for user to set application wake up low power mode
 * @param	wakeup_tick - low power mode wake up time
 * @param	enable - low power mode application wake up enable
 * @return	none
 */
void 		bls_pm_setAppWakeupLowPower(u32 wakeup_tick, u8 enable);

/**
 * @brief	for user to register the callback for application wake up low power mode process
 * @param	cb - the pointer of callback function
 * @return  none.
 */
void 		bls_pm_registerAppWakeupLowPowerCb(pm_appWakeupLowPower_callback_t cb);

/**
 * @brief	for user to set the threshold of sleep tick for entering deep retention mode
 * @param	adv_thres_ms - the threshold of sleep tick for advertisement state
 * @param	conn_thres_ms - the threshold of sleep tick for connection state
 * @return  none.
 */
void 		blc_pm_setDeepsleepRetentionThreshold(u32 adv_thres_ms, u32 conn_thres_ms);

/**
 * @brief	for user to set early wake up tick for deep retention mode
 * @param	earlyWakeup_us - early wake up tick for deep retention mode
 * @return  none.
 */
void 		blc_pm_setDeepsleepRetentionEarlyWakeupTiming(u32 earlyWakeup_us);

/**
 * @brief	for user to set the type of deep retention mode
 * @param	sleep_type - the type of deep retention mode
 * @return  none.
 */
void 		blc_pm_setDeepsleepRetentionType(SleepMode_TypeDef sleep_type);

/************************* Stack Interface, user can not use!!! ***************************/




#endif /* LL_PM_H_ */
