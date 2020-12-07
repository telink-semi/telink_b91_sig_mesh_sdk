/********************************************************************************************************
 * @file	app_audio.c
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
#include "drivers.h"
#include "app_config.h"
#include "application/audio/tl_audio.h"
#include "application/audio/audio_config.h"
#include "stack/ble/ble.h"
#include "app_audio.h"
#include "vendor/common/blt_led.h"
#include "stack/ble/ble.h"

#include "app_att.h"
#include "app_ui.h"

_attribute_data_retention_	u8		ui_mic_enable = 0;
_attribute_data_retention_	u8 		key_voice_press = 0;
_attribute_data_retention_	int     ui_mtu_size_exchange_req = 0;

#if (BLT_APP_LED_ENABLE)
extern  const   led_cfg_t led_cfg[];
#endif

#if (BLE_AUDIO_ENABLE)


#if (TL_AUDIO_MODE == TL_AUDIO_RCU_ADPCM_GATT_TLEINK)					//GATT Telink
#include "../../application/audio/tl_audio.h"

	dma_chain_config_st rx_dma_list_config[2];
	dma_chain_config_st tx_dma_list_config[2];
	extern u8 audio_tx_dma_chn;
	extern u8 audio_rx_dma_chn;
	volatile u8 amic_enable;

	void audio_amic_init(void)
	{
		audio_set_codec_supply();
		audio_set_codec_in_path_a_d_gain(CODEC_IN_D_GAIN_20_DB,CODEC_IN_A_GAIN_0_DB);//recommend setting dgain:20db,again 0db
		audio_init(AMIC_IN_ONLY ,AUDIO_16K,MONO_BIT_16);
		audio_rx_dma_chain_init(DMA2,(u16*)&buffer_mic,TL_MIC_BUFFER_SIZE);
	}
	void audio_dmic_init()
	{
		audio_set_codec_supply();
		audio_set_codec_in_path_a_d_gain(CODEC_IN_D_GAIN_20_DB,CODEC_IN_A_GAIN_0_DB);
		audio_set_dmic_pin(DMIC_GROUPB_B2_DAT_B3_B4_CLK);
		audio_init(DMIC_IN_ONLY ,AUDIO_16K,MONO_BIT_16);
		audio_rx_dma_chain_init(DMA2,(u16*)&buffer_mic,TL_MIC_BUFFER_SIZE);
	}
	void audio_mic_off()//
	{
		audio_clk_en(0,0);
		audio_rx_dma_dis();
		extern void audio_codec_adc_power_down(void);
		audio_codec_adc_power_down();
	}

	u32 	key_voice_pressTick = 0;

	void ui_enable_mic (int en)
	{
		ui_mic_enable = en;

		#if (BLT_APP_LED_ENABLE)
			device_led_setup(led_cfg[en ? LED_AUDIO_ON : LED_AUDIO_OFF]);
		#endif
		#if(BOARD_SELECT == EVK_BOARD)
			gpio_write(GPIO_LED_RED,en);
		#endif
		if(en){  //audio on
			///////////////////// AUDIO initialization///////////////////
			#if (BLE_DMIC_ENABLE)  //Dmic config
				audio_dmic_init();
			#else  //Amic config
				audio_amic_init();
			#endif
		}
		else{  //audio off
			#if (BLE_DMIC_ENABLE)  //Dmic config
			audio_mic_off();
			#else  //audio off
				audio_mic_off();
			#endif
		}

		#if (BATT_CHECK_ENABLE)
			battery_set_detect_enable(!en);
		#endif
	}


	void voice_press_proc(void)
	{
		key_voice_press = 0;
		ui_enable_mic (1);
#if 1
		if(ui_mtu_size_exchange_req && blc_ll_getCurrentState() == BLS_LINK_STATE_CONN){
			ui_mtu_size_exchange_req = 0;
			blc_att_requestMtuSizeExchange(BLS_CONN_HANDLE, 0x009e);
		}
#endif
	}


	void task_audio (void)
	{
		static u32 audioProcTick = 0;
		if(clock_time_exceed(audioProcTick, 500)){
			audioProcTick = clock_time();
		}
		else{
			return;
		}

		///////////////////////////////////////////////////////////////
//		log_event(TR_T_audioTask);
		proc_mic_encoder ();

		if (blc_ll_getTxFifoNumber() < 9)
		{
			int *p = mic_encoder_data_buffer ();
			if (p)					//around 3.2 ms @16MHz clock
			{
				if( BLE_SUCCESS == bls_att_pushNotifyData (AUDIO_MIC_INPUT_DP_H, (u8*)p, ADPCM_PACKET_LEN) ){
					mic_encoder_data_read_ok();
					DBG_CHN5_TOGGLE;
				}
			}
		}
	}

	void blc_checkConnParamUpdate(void)
	{
		extern u32 interval_update_tick;

		if(	 interval_update_tick && clock_time_exceed(interval_update_tick,5*1000*1000) && \
			 blc_ll_getCurrentState() == BLS_LINK_STATE_CONN &&  bls_ll_getConnectionInterval()!= 8 )
		{
			interval_update_tick = clock_time() | 1;
			bls_l2cap_requestConnParamUpdate (8, 8, 99, 400);
		}
	}

#endif
#endif

