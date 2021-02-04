/********************************************************************************************************
 * @file	app.c
 *
 * @brief	for TLSR chips
 *
 * @author	BLE GROUP
 * @date	2020.06
 *
 * @par     Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *          
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *          
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *          
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions 
 *              in binary form must reproduce the above copyright notice, this list of 
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *          
 *              3. Neither the name of TELINK, nor the names of its contributors may be 
 *              used to endorse or promote products derived from this software without 
 *              specific prior written permission.
 *          
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or 
 *              relating to such deletion(s), modification(s) or alteration(s).
 *         
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *         
 *******************************************************************************************************/
#include "tl_common.h"
#include "drivers/9518/rf.h"
#include "drivers/9518/pm.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll_whitelist.h"
#include "proj_lib/ble/trace.h"
#include "proj_lib/ble/ble_common.h"
#include "drivers/9518/pwm.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
//#include "proj/drivers/adc.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ble_smp.h"
#include "proj_lib/mesh_crypto/mesh_crypto.h"
#include "proj_lib/mesh_crypto/mesh_md5.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "../common/app_provison.h"
#include "../common/app_beacon.h"
#include "../common/app_proxy.h"
#include "../common/app_health.h"
#include "../common/vendor_model.h"
#include "application/keyboard/keyboard.h"
#include "app.h"
#include "stack/ble/gap/gap.h"
#include "proj_lib/ble/l2cap.h"
#include "vendor/common/blt_soft_timer.h"
#include "drivers/9518/rf_pa.h"
#include "../common/remote_prov.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "vendor/common/blt_common.h"
#if MI_API_ENABLE
#include "vendor/common/mi_api/telink_sdk_mible_api.h"
#include "vendor/common/mi_api/libs/mijia_profiles/mi_service_server.h"
#endif 
#if (HCI_ACCESS==HCI_USE_UART)
#include "proj/drivers/uart.h"
#endif

#if DEBUG_CFG_CMD_GROUP_AK_EN
#define LL_RX_FIFO_SIZE		64
#define LL_RX_FIFO_NUM		8
#else
#define LL_RX_FIFO_SIZE		64
#define LL_RX_FIFO_NUM		16
#endif

#define LL_TX_FIFO_SIZE		48
#define LL_TX_FIFO_NUM		17  //only 9 and 17  can be used

_attribute_data_retention_	u8	app_ll_rxfifo[LL_RX_FIFO_SIZE * LL_RX_FIFO_NUM] = {0};
_attribute_data_retention_  u8	app_ll_txfifo[LL_TX_FIFO_SIZE * LL_TX_FIFO_NUM] = {0};



// u8		peer_type;
// u8		peer_mac[12];

//////////////////////////////////////////////////////////////////////////////
//	Initialization: MAC address, Adv Packet, Response Packet
//////////////////////////////////////////////////////////////////////////////

//----------------------- UI ---------------------------------------------
#if (BLT_SOFTWARE_TIMER_ENABLE)
/**
 * @brief   This function is soft timer callback function.
 * @return  <0:delete the timer; ==0:timer use the same interval as prior; >0:timer use the return value as new interval. 
 */
int soft_timer_test0(void)
{
	//gpio 0 toggle to see the effect
	DBG_CHN4_TOGGLE;
	static u32 soft_timer_cnt0 = 0;
	soft_timer_cnt0++;
	return 0;
}
#endif
#if MI_SWITCH_LPN_EN
void mi_mesh_switch_sys_mode(u32 clk)
{
	// ignore hard ware uart function ,and it need to reset all the param part 
	if(clk == 16000000){
		clock_init(SYS_CLK_16M_Crystal);

	}else if (clk == 48000000){
		clock_init(SYS_CLK_48M_Crystal);
	}else{}
}

#endif

#define TEST_FORWARD_ADDR_FILTER_EN     0
#if TEST_FORWARD_ADDR_FILTER_EN
#define FILTER_TYPE_F2	1
#define FILTER_TYPE_F4	2
#define FILTER_TYPE_F6	3
#define FILTER_TYPE_F8	4
#define FILTER_TYPE_FA	5
#define FILTER_TYPE_FC	6

#define FILTER_NODE_MAC_F1 	{0xf1,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_F2 	{0xf2,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_F4	{0xf4,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_F6	{0xf6,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_F8	{0xf8,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_FA	{0xfa,0x22,0x33,0x44,0xff,0xff}
#define FILTER_NODE_MAC_FC	{0xfc,0x22,0x33,0x44,0xff,0xff}


typedef struct{
	u8 mac[6];
}filter_mac_str;

#define FORWARD_FILTER_TYPE		FILTER_TYPE_FA
	#if (FORWARD_FILTER_TYPE == FILTER_TYPE_F2)
	filter_mac_str forward_mac[1]={FILTER_NODE_MAC_F4};
	#elif(FORWARD_FILTER_TYPE == FILTER_TYPE_F4)
	filter_mac_str forward_mac[2]={FILTER_NODE_MAC_F2,FILTER_NODE_MAC_F6};
	#elif(FORWARD_FILTER_TYPE == FILTER_TYPE_F6)
	filter_mac_str forward_mac[2]={FILTER_NODE_MAC_F4,FILTER_NODE_MAC_F8};
	#elif(FORWARD_FILTER_TYPE == FILTER_TYPE_F8)
	filter_mac_str forward_mac[2]={FILTER_NODE_MAC_F6,FILTER_NODE_MAC_FA};
	#elif(FORWARD_FILTER_TYPE == FILTER_TYPE_FA)
	filter_mac_str forward_mac[2]={FILTER_NODE_MAC_F8,FILTER_NODE_MAC_FC};
	#elif(FORWARD_FILTER_TYPE == FILTER_TYPE_FC)
	filter_mac_str forward_mac[1]=FILTER_NODE_MAC_FA;
	#endif

u8 find_mac_in_filter_list(u8 *p_mac)
{
	foreach_arr(i,forward_mac){
		u8 *p_mac_filter = (u8 *)(forward_mac+i);
		if(!memcmp(p_mac_filter,p_mac,sizeof(filter_mac_str))){
			return 1;
		}
	}
	return 0;
}
#endif


//----------------------- handle BLE event ---------------------------------------------
int app_event_handler (u32 h, u8 *p, int n)
{
	static u32 event_cb_num;
	event_cb_num++;
	int send_to_hci = 1;

	if (h == (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META))		//LE event
	{
		u8 subcode = p[0];
		#if MI_API_ENABLE
		telink_ble_mi_app_event(subcode,p,n);
		#endif 
	//------------ ADV packet --------------------------------------------
		if (subcode == HCI_SUB_EVT_LE_ADVERTISING_REPORT)	// ADV packet
		{
			event_adv_report_t *pa = (event_adv_report_t *)p;
			#if MD_REMOTE_PROV
			mesh_cmd_extend_loop_cb(pa);
			#endif
			if(LL_TYPE_ADV_NONCONN_IND != (pa->event_type & 0x0F)){
				return 0;
			}
			#if TEST_FORWARD_ADDR_FILTER_EN
			if(!find_mac_in_filter_list(pa->mac)){
				return 0;
			}
			#endif
			#if 0 // TESTCASE_FLAG_ENABLE
			u8 mac_pts[] = {0xDA,0xE2,0x08,0xDC,0x1B,0x00};	// 0x001BDC08E2DA
			u8 mac_pts2[] = {0xDB,0xE2,0x08,0xDC,0x1B,0x00};	// 0x001BDC08E2DA
			if(memcmp(pa->mac, mac_pts,6) && memcmp(pa->mac, mac_pts2,6)){
				return 0;
			}
			#endif
			
			#if DEBUG_MESH_DONGLE_IN_VC_EN
			send_to_hci = mesh_dongle_adv_report2vc(pa->data, MESH_ADV_PAYLOAD);
			#else
			send_to_hci = app_event_handler_adv(pa->data, MESH_BEAR_ADV, 1);
			#endif
		}

	//------------ connection complete -------------------------------------
		else if (subcode == HCI_SUB_EVT_LE_CONNECTION_COMPLETE)	// connection complete
		{
			#if MI_SWITCH_LPN_EN
			mi_mesh_switch_sys_mode(48000000);
			bls_ll_setAdvParam( ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, \
			 	 	 	 	 	     ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
			 	 	 	 	 	     0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
			#endif
			
			event_connection_complete_t *pc = (event_connection_complete_t *)p;
			if (!pc->status)							// status OK
			{
				//app_led_en (pc->handle, 1);

				//peer_type = pc->peer_adr_type;
				//memcpy (peer_mac, pc->mac, 6);
			}
			#if DEBUG_BLE_EVENT_ENABLE
			rf_link_light_event_callback(LGT_CMD_BLE_CONN);
			#endif

			#if DEBUG_MESH_DONGLE_IN_VC_EN
			debug_mesh_report_BLE_st2usb(1);
			#endif
			proxy_cfg_list_init_upon_connection();
			#if 0 // FEATURE_FRIEND_EN
			fn_update_RecWin(get_RecWin_connected());
			#endif
			mesh_service_change_report();
		}

	//------------ connection update complete -------------------------------
		else if (subcode == HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE)	// connection update
		{
			#if 0 // FEATURE_FRIEND_EN
			fn_update_RecWin(get_RecWin_connected());
			#endif
		}
	}

	//------------ disconnect -------------------------------------
	else if (h == (HCI_FLAG_EVENT_BT_STD | HCI_EVT_DISCONNECTION_COMPLETE))		//disconnect
	{
		#if MI_SWITCH_LPN_EN
		mi_mesh_switch_sys_mode(16000000);
		#endif
		event_disconnection_t	*pd = (event_disconnection_t *)p;
		//app_led_en (pd->handle, 0);
		#if MI_API_ENABLE
		telink_ble_mi_app_event(HCI_EVT_DISCONNECTION_COMPLETE,p,n);
		#endif 
		//terminate reason
		if(pd->reason == HCI_ERR_CONN_TIMEOUT){

		}
		else if(pd->reason == HCI_ERR_REMOTE_USER_TERM_CONN){  //0x13

		}
		#if DEBUG_BLE_EVENT_ENABLE
		rf_link_light_event_callback(LGT_CMD_BLE_ADV);
		#endif 

		#if DEBUG_MESH_DONGLE_IN_VC_EN
		debug_mesh_report_BLE_st2usb(0);
		#endif

		mesh_ble_disconnect_cb();
		#if 0 // FEATURE_FRIEND_EN
        fn_update_RecWin(FRI_REC_WIN_MS);   // restore
        #endif
	}

	if (send_to_hci)
	{
		//blc_hci_send_data (h, p, n);
	}

	return 0;
}

void proc_ui()
{
	static u32 tick, scan_io_interval_us = 40000;
	if (!clock_time_exceed (tick, scan_io_interval_us))
	{
		return;
	}
	tick = clock_time();

	#if 0
	static u8 st_sw1_last,st_sw2_last;	
	u8 st_sw1 = !gpio_read(SW1_GPIO);
	u8 st_sw2 = !gpio_read(SW2_GPIO);
	
	if(!(st_sw1_last)&&st_sw1){
	    scan_io_interval_us = 100*1000; // fix dithering
	    access_cmd_onoff(0xffff, 0, G_ON, CMD_NO_ACK, 0);
		foreach(i,NET_KEY_MAX){
					mesh_key.net_key[i][0].node_identity =1;
		}
	}
	st_sw1_last = st_sw1;
	
	if(!(st_sw2_last)&&st_sw2){
	    scan_io_interval_us = 100*1000; // fix dithering
	    access_cmd_onoff(0xffff, 0, G_OFF, CMD_NO_ACK, 0);
	}
	st_sw2_last = st_sw2;

	
	#endif

	#if 0
	static u8 st_sw2_last;	
	u8 st_sw2 = !gpio_read(SW2_GPIO);
	
	if(!(st_sw2_last)&&st_sw2){ // dispatch just when you press the button 
		//trigger the unprivison data packet 
		static u8 beacon_data_num;
		beacon_data_num =1;
		mesh_provision_para_reset();
		while(beacon_data_num--){
			unprov_beacon_send(MESH_UNPROVISION_BEACON_WITH_URI,0);
		}
		prov_para.initial_pro_roles = MESH_INI_ROLE_NODE;
	    scan_io_interval_us = 100*1000; // fix dithering
	}
	st_sw2_last = st_sw2;
	#endif
}

/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////

// simu uart io printf demo 
#if 0
void test_simu_io_user_define_proc()
{
    static u32 A_debug_print_tick =0;
    u8 data_test[4]={1,2,3,4};
    u8 data_val = 0xff;
    if(clock_time_exceed(A_debug_print_tick,100*1000)){
        A_debug_print_tick = clock_time();
        LOG_USER_MSG_INFO(data_test,sizeof(data_test),"user data val is %d",data_val);
    }
}
#endif


#if SPEECH_ENABLE
#define AUDIO_LINE_IN            0
#define AUDIO_AMIC_IN            1
#define AUDIO_DMIC_IN            2

#define  AUDIO_IN_MODE          AUDIO_AMIC_IN

#define MIC_RESOLUTION_BIT		16

#define MIC_SAMPLE_RATE			16000//set sample for mic


#define MIC_CHANNLE_COUNT		1   //1 or 2
#define	MIC_ENOCDER_ENABLE		0


#define SPK_CHANNLE_COUNT		2
#define SPK_RESOLUTION_BIT		16

#define MIC_SAMPLING_RATE   (MIC_SAMPLE_RATE== 8000) ?  AUDIO_8K :((MIC_SAMPLE_RATE== 16000) ?  AUDIO_16K :(  (MIC_SAMPLE_RATE== 32000) ?  AUDIO_32K :( (MIC_SAMPLE_RATE== 48000) ? AUDIO_48K : AUDIO_16K) ) )
#define MIC_MONO_STEREO       ((MIC_CHANNLE_COUNT==1) ?  MONO_BIT_16 :STEREO_BIT_16 )

#define	MIC_BUFFER_SIZE			8192//2048
#define MIC_DMA_CHN             DMA2


u16		iso_in_buff[MIC_BUFFER_SIZE];
#include "pcm_fifo.h"
FIFI_WITH_SEM kwd_fifo;


#include "libaid_awaken.h"
#define KWD_LEN    320
#define KWD_SAMPLE_RATE 16000
static short kwd_data[KWD_LEN];
static int wakeup_times[40] = {0,0,0,0,0,0,0,0,0,0};

volatile u32		iso_in_w = 0;
volatile u32  	     iso_in_r = 0;
u32		num_iso_in = 0;
/**
 * @brief     This function serves to send data to USB. only adaptive mono 16bit
 * @param[in] audio_rate - audio rate. This value is matched with usb_default.h :MIC_SAMPLE_RATE.
 * @return    none.
 */


int audio_tx_data_to_kwd_buf(unsigned int audio_rate)
{
	unsigned char length = 0;
	iso_in_w = ((audio_get_rx_dma_wptr (MIC_DMA_CHN) - (u32)iso_in_buff) >> 1);
	switch(audio_rate)
	{
		case 	8000:	length = 80;break;
		case	16000:	length = 160;break;
		default:		length = 320;break;
	}
	for (u8 i=0; i<length&& iso_in_r != iso_in_w ; i++)
	{
		short md = iso_in_buff[iso_in_r++ &(MIC_BUFFER_SIZE-1)];

		Fifo_Write(&kwd_fifo, &md, sizeof(short));

	}

	return length;
}
void timer0_irq_handler(void)
{
	if(timer_get_irq_status(TMR_STA_TMR0))
	{
		timer_clr_irq_status(TMR_STA_TMR0);
        audio_tx_data_to_kwd_buf(KWD_SAMPLE_RATE);
	}
}

#endif

void main_loop ()
{
	static u32 tick_loop;

	tick_loop ++;
	
#if SPEECH_ENABLE
	if(Fifo_Read(&kwd_fifo, kwd_data, sizeof(short)*KWD_LEN, sizeof(short)*KWD_LEN) !=0 )
	{
		int ret = AidAwakenProcess(kwd_data,NULL,KWD_LEN);
		/*
			ret值:
			打开灯光				9
			关闭灯光                10
			打开吊灯				11
			关闭吊灯       		    12
			打开客厅灯				15
			关闭客厅灯				16	
			打开卧室灯				17
			关闭卧室灯				18
		*/
		u8 data[4]={0x00,0x00,0x00,0x00};
		u16 adr=0;
		if(ret > 0)
		{
			if(ret == 9){
				data[0]=1;
				adr = 0xffff;
			}else if(ret == 10){
				data[0]=0;
				adr = 0xffff;
			}else if (ret == 11){
				data[0]=1;
				adr = 0xc007;
			}else if(ret == 12){
				data[0]=0;
				adr = 0xc007;
			}else if(ret == 15){
				data[0]=1;
				adr = 0xc000;
			}else if(ret == 16){
				data[0]=0;
				adr = 0xc000;
			}else if (ret == 17){
				data[0]=1;
				adr = 0xc002;
			}else if (ret ==18){
				data[0]=0;
				adr = 0xc002;
			}
			if(adr !=0){
				mesh_tx_cmd2normal_primary(G_ONOFF_SET,data,sizeof(data),adr,0);
			}
		}				
		else if(ret == ERR_MAX_LIMIT)
		{
		}
	}
#endif

#if (BLT_SOFTWARE_TIMER_ENABLE)
	blt_soft_timer_process(MAINLOOP_ENTRY);
#endif
	mesh_loop_proc_prior(); // priority loop, especially for 8269
//	test_simu_io_user_define_proc();
	#if DUAL_MODE_ADAPT_EN
	if(RF_MODE_BLE != dual_mode_proc()){    // should be before is mesh latency window()
        proc_ui();
        proc_led();
        factory_reset_cnt_check();
		return ;
	}
	#endif
	
	#if SIG_MESH_LOOP_PROC_10MS_EN
	if(is_mesh_latency_window()){
	    return ;
	}
	#endif
	
	#if SPIRIT_VENDOR_EN
	mesh_tx_indication_proc();
	#if 0 // for indication test
	static u8 A_send_indication=0;
	if(A_send_indication){
		A_send_indication = 0;
		u16 val= 0x3344;
		access_cmd_attr_indication(VD_MSG_ATTR_INDICA,0xffff,ATTR_CURRENT_TEMP, (u8 *)&val, sizeof(val));
	}
	#endif	
	#endif
	////////////////////////////////////// BLE entry /////////////////////////////////
	blt_sdk_main_loop ();


	////////////////////////////////////// UI entry /////////////////////////////////
	//  add spp UI task:
	proc_ui();
	proc_led();
	factory_reset_cnt_check();
	
	mesh_loop_process();
	#if MI_API_ENABLE
	telink_gatt_event_loop();
	ev_main();
	mi_vendor_cfg_rsp_proc();
	#if XIAOMI_MODULE_ENABLE
	mi_api_loop_run();
	#endif
	mi_schd_process();
	#endif 
	
	#if ADC_ENABLE
	static u32 adc_check_time;
    if(clock_time_exceed(adc_check_time, 1000*1000)){
        adc_check_time = clock_time();
		static u16 T_adc_val;
		#if(MCU_CORE_TYPE == MCU_CORE_8269)     
        T_adc_val = adc_BatteryValueGet();
		#else
		T_adc_val = adc_sample_and_get_result();
		#endif
    }  
	#endif
	//sim_tx_cmd_node2node();

	#if DEBUG_IV_UPDATE_TEST_EN
	iv_index_test_button_firmware();
	#endif
	#if MI_SWITCH_LPN_EN
	mi_mesh_lowpower_loop();
	#endif	
}

#if IRQ_TIMER1_ENABLE
void light_hw_timer1_config(void){
    reg_irq_mask |= FLD_IRQ_TMR1_EN;
    reg_tmr1_tick = 0;
    reg_tmr1_capt = CLOCK_MCU_RUN_CODE_1US * IRQ_TIME1_INTERVAL ;
    reg_tmr_ctrl |= FLD_TMR1_EN;
}
#endif
#if 0 // ecc verify 
void test_ecdsa_sig_verify2()
{
	// test for the part of the 
	micro_ecc_init(NULL);
	u8 ecdsa_sk[32];
	u8 ecdsa_pk[64];
	micro_ecc_keypair_gen(NULL, ecdsa_sk, ecdsa_pk);
	static u32 A_debug_sk_calc = 0;
	unsigned char hash_dat[32]={0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7, 
								0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7};
	unsigned char sign_dat[64];
	micro_ecc_sign(NULL, ecdsa_sk, hash_dat, sign_dat);
	if(micro_ecc_verify(NULL, ecdsa_pk, hash_dat, sign_dat)==0){
		A_debug_sk_calc =2;
	}else{
		A_debug_sk_calc =0x55;
	}

}
#endif


void user_init()
{
	
#if SPEECH_ENABLE
	audio_set_codec_supply();
	//delay_ms(1000);
	AidAwakenInit(NULL,KWD_SAMPLE_RATE,false);
	//delay_ms(1000);
	Fifo_Init(&kwd_fifo, 8192);
	
#if(AUDIO_IN_MODE==AUDIO_LINE_IN)
	audio_init(LINE_IN_ONLY ,MIC_SAMPLING_RATE,MIC_MONO_STEREO);
	audio_rx_dma_chain_init(DMA2,(u16*)&iso_in_buff,MIC_BUFFER_SIZE*2);
#elif(AUDIO_IN_MODE==AUDIO_DMIC_IN)
	audio_set_codec_in_path_a_d_gain(CODEC_IN_D_GAIN_16_DB,CODEC_IN_A_GAIN_8_DB);
	audio_set_dmic_pin(DMIC_GROUPB_B2_DAT_B3_B4_CLK);
	audio_init(DMIC_IN_ONLY ,MIC_SAMPLING_RATE,MIC_MONO_STEREO);
	audio_rx_dma_chain_init(DMA2,(u16*)&iso_in_buff,MIC_BUFFER_SIZE*2);
#elif(AUDIO_IN_MODE==AUDIO_AMIC_IN)
	audio_init(AMIC_IN_ONLY ,MIC_SAMPLING_RATE,MIC_MONO_STEREO);
	audio_rx_dma_chain_init(DMA2,(u16*)&iso_in_buff,MIC_BUFFER_SIZE*2);
#endif

	plic_interrupt_enable(IRQ4_TIMER0);
	timer_set_mode(TIMER0, TIMER_MODE_SYSCLK, 0, 10*sys_clk.pclk*1000);
	timer_start(TIMER0);
	core_interrupt_enable();
#endif
		
//random number generator must be initiated here( in the beginning of user_init_nromal)
	//when deepSleep retention wakeUp, no need initialize again
	random_generator_init();  //this is must
		/*****************************************************************************************
	 Note: battery check must do before any flash write/erase operation, cause flash write/erase
		   under a low or unstable power supply will lead to error flash operation

		   Some module initialization may involve flash write/erase, include: OTA initialization,
				SMP initialization, ..
				So these initialization must be done after  battery check
	*****************************************************************************************/
	#if (BATT_CHECK_ENABLE)  //battery check must do before OTA relative operation
		if(analog_read_reg8(USED_DEEP_ANA_REG) & LOW_BATT_FLG){
			app_battery_power_check(VBAT_ALRAM_THRES_MV + 200);  //2.2 V
		}
		else{
			app_battery_power_check(VBAT_ALRAM_THRES_MV);  //2.0 V
		}
	#endif
	#if DEBUG_EVB_EN
	    set_sha256_init_para_mode(1);	// must 1
	#else
		user_sha256_data_proc();
	#endif
	mesh_global_var_init();
    #if (DUAL_MODE_WITH_TLK_MESH_EN)
	dual_mode_en_init();    // must before proc_telink_mesh_to_sig_mesh_, because "dual_mode_state" is used in it.
    #endif
	proc_telink_mesh_to_sig_mesh();		// must at first
	//set_blc_hci_flag_fun(0);// disable the hci part of for the lib .
	#if (DUAL_MODE_ADAPT_EN)
	dual_mode_en_init();    // must before factory_reset_handle, because "dual_mode_state" is used in it.
	#endif
	
	blc_app_loadCustomizedParameters();  //load customized freq_offset cap value and tp value

	usb_id_init();
	//usb_log_init();
	usb_dp_pullup_en (1);  //open USB enum

	////////////////// BLE stack initialization ////////////////////////////////////
#if (DUAL_VENDOR_EN)
	mesh_common_retrieve(FLASH_ADR_PROVISION_CFG_S);
	if(DUAL_VENDOR_ST_ALI != provision_mag.dual_vendor_st)
#endif
	{ble_mac_init();
	}

	//link layer initialization
	//bls_ll_init (tbl_mac);
#if(MCU_CORE_TYPE == MCU_CORE_8269)
	blc_ll_initBasicMCU(tbl_mac);   //mandatory
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(tbl_mac);				//mandatory
#elif(MCU_CORE_TYPE == MCU_CORE_9518)
	////// Controller Initialization  //////////
	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(tbl_mac);				//mandatory
	blc_ll_initAdvertising_module(); 	//adv module: 		 mandatory for BLE slave,
	blc_ll_initConnection_module();				//connection module  mandatory for BLE slave/master
	blc_ll_initSlaveRole_module();				//slave module: 	 mandatory for BLE slave,
#endif
	blc_ll_initAdvertising_module(); 	//adv module: 		 mandatory for BLE slave,
	blc_ll_initSlaveRole_module();				//slave module: 	 mandatory for BLE slave,
	// init tx and rx init
	blc_ll_initTxFifo(app_ll_txfifo, LL_TX_FIFO_SIZE, LL_TX_FIFO_NUM);
	blc_ll_initRxFifo(app_ll_rxfifo, LL_RX_FIFO_SIZE, LL_RX_FIFO_NUM);
#if (BLE_REMOTE_PM_ENABLE)
	blc_ll_initPowerManagement_module();        //pm module:      	 optional
	#if MI_SWITCH_LPN_EN
	bls_pm_setSuspendMask (SUSPEND_DISABLE);
	#else
	bls_pm_setSuspendMask (SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);
	#endif
	blc_pm_setDeepsleepRetentionThreshold(50, 30);
	blc_pm_setDeepsleepRetentionEarlyWakeupTiming(400);
#else
	bls_pm_setSuspendMask (SUSPEND_DISABLE);//(SUSPEND_ADV | SUSPEND_CONN)
#endif
	
	//l2cap initialization
	//blc_l2cap_register_handler (blc_l2cap_packet_receive);
	blc_l2cap_register_handler (app_l2cap_packet_receive); // define the l2cap part 
	///////////////////// USER application initialization ///////////////////

	u8 status = bls_ll_setAdvParam( ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, \
			 	 	 	 	 	     ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
			 	 	 	 	 	     0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);

#if (DUAL_VENDOR_EN)
    status = status;    // it will be optimized
#else
	if(status != BLE_SUCCESS){  //adv setting err
		write_reg8(0x8000, 0x11);  //debug
		while(1);
	}
#endif
	
	// normally use this settings 
	blc_ll_setAdvCustomedChannel (37, 38, 39);
	bls_ll_setAdvEnable(1);  //adv enable

	rf_set_power_level_index (my_rf_power_index);	
	
    blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_ADVERTISING_REPORT|
								HCI_LE_EVT_MASK_CONNECTION_COMPLETE|
								HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE);

	////////////////// SPP initialization ///////////////////////////////////
#if (HCI_ACCESS != HCI_USE_NONE)
	#if (HCI_ACCESS==HCI_USE_USB)
	//blt_set_bluetooth_version (BLUETOOTH_VER_4_2);
	//bls_ll_setAdvChannelMap (BLT_ENABLE_ADV_ALL);
	usb_bulk_drv_init (0);
	blc_register_hci_handler (app_hci_cmd_from_usb, blc_hci_tx_to_usb);
	#elif (HCI_ACCESS == HCI_USE_UART)	//uart
	uart_drv_init();
	blc_register_hci_handler (blc_rx_from_uart, blc_hci_tx_to_uart);		//default handler
	//blc_register_hci_handler(rx_from_uart_cb,tx_to_uart_cb);				//customized uart handler
	#endif
#endif
	#if ADC_ENABLE
	adc_drv_init();
	#endif
	rf_pa_init();
	bls_app_registerEventCallback (BLT_EV_FLAG_CONNECT, (blt_event_callback_t)&mesh_ble_connect_cb);
	blc_hci_registerControllerEventHandler(app_event_handler);		//register event callback

	//bls_hci_mod_setEventMask_cmd(0xffff);			//enable all 15 events,event list see ble_ll.h
	bls_set_advertise_prepare (app_advertise_prepare_handler);
	//bls_set_update_chn_cb(chn_conn_update_dispatch);
	
	bls_ota_registerStartCmdCb(entry_ota_mode);
	bls_ota_registerResultIndicateCb(show_ota_result);
	#if !GATT_LPN_EN
	app_enable_scan_all_device ();	// enable scan adv packet 
	#endif
	// mesh_mode and layer init
	mesh_init_all();

	// OTA init
	#if (DUAL_MODE_ADAPT_EN && (0 == FW_START_BY_BOOTLOADER_EN) || DUAL_MODE_WITH_TLK_MESH_EN)
	if(DUAL_MODE_NOT_SUPPORT == dual_mode_state)
	#endif	
	{
		#if MI_API_ENABLE
			#define RECORD_DFU_INFO   5
			u32 adr_record;
			if(!find_record_adr(RECORD_DFU_INFO,&adr_record)){
				bls_ota_clearNewFwDataArea();
				telink_record_clean_cpy();// trigger clean recycle ,and it will not need to clean in the conn state
			}
		#else
		#if !SPEECH_ENABLE
		bls_ota_clearNewFwDataArea();	 //must
		#endif
		#endif
	}
	//blc_ll_initScanning_module(tbl_mac);
	//gatt initialization
	#if((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278)||(MCU_CORE_TYPE == MCU_CORE_9518))
	blc_gap_peripheral_init();    //gap initialization
	#endif	
	
	mesh_scan_rsp_init();
	my_att_init (provision_mag.gatt_mode);
	blc_att_setServerDataPendingTime_upon_ClientCmd(10);
#if MI_API_ENABLE
    #if DUAL_VENDOR_EN
    if(DUAL_VENDOR_ST_ALI != provision_mag.dual_vendor_st) // mac have been set to ali mode before.
    #endif
    {
	//// used for the telink callback part 
	//extern int mi_mesh_otp_program_simulation();
	//mi_mesh_otp_program_simulation();
	blc_att_setServerDataPendingTime_upon_ClientCmd(1);
	telink_record_part_init();
	#if 0 // XIAOMI_MODULE_ENABLE
	test_mi_api_part(); // just for test
	#endif
	#if MI_SWITCH_LPN_EN
	mi_mesh_switch_sys_mode(16000000);
	mi_mesh_sleep_init();
	#endif
	blc_l2cap_register_pre_handler(telink_ble_mi_event_cb_att);// for telink event callback
	advertise_init();
	mi_service_init();
	telink_mi_vendor_init();
	//cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
	//cfg_led_event(LED_EVENT_FLASH_1HZ_3S);
	}
#endif 
	extern u32 system_time_tick;
	system_time_tick = clock_time();
#if TESTCASE_FLAG_ENABLE
	memset(&model_sig_cfg_s.hb_sub, 0x00, sizeof(mesh_heartbeat_sub_str)); // init para for test
#endif
#if IRQ_TIMER1_ENABLE
    light_hw_timer1_config();
#endif
#if IRQ_GPIO_ENABLE
	gpio_set_interrupt_init(SW1_GPIO, PM_PIN_PULLUP_1M, 1, FLD_IRQ_GPIO_EN);
#endif
#if (BLT_SOFTWARE_TIMER_ENABLE)
	blt_soft_timer_init();
	//blt_soft_timer_add(&soft_timer_test0, 200*1000);
#endif
#if DEBUG_CFG_CMD_GROUP_AK_EN
	if(blt_rxfifo.num >64){
		blt_rxfifo.num = 64;
	}
#endif

    CB_USER_INIT();


}

#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_ void user_init_deepRetn(void)
{
    blc_app_loadCustomizedParameters();
	blc_ll_initBasicMCU();   //mandatory
	rf_set_power_level_index (my_rf_power_index);
#if MI_SWITCH_LPN_EN
		if(bltPm.appWakeup_flg){ // it may have the rate not response by the event tick part ,so we should use the distance
			if(mi_mesh_sleep_time_exceed_adv_iner()){
				mi_mesh_sleep_init();
				bls_pm_setSuspendMask (SUSPEND_DISABLE);
			}
		}else{// if it is wakeup by the adv event tick part ,we need to refresh the adv tick part
			mi_mesh_sleep_init();
			bls_pm_setSuspendMask (SUSPEND_DISABLE);
		}
#endif

	blc_ll_recoverDeepRetention();

	DBG_CHN0_HIGH;    //debug
	
    light_pwm_init();
#if (HCI_ACCESS == HCI_USE_UART)	//uart
	uart_drv_init();
#endif
#if ADC_ENABLE
	adc_drv_init();
#endif
    // should enable IRQ here, because it may use irq here, for example BLE connect.
    // irq_enable();
}
#endif

