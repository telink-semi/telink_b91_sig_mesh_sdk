/********************************************************************************************************
 * @file	rf.h
 *
 * @brief	This is the header file for B91
 *
 * @author	W.Z.W
 * @date	2019
 *
 * @par		Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
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
#ifndef     RF_H
#define     RF_H

#include "gpio.h"
#include "sys.h"

/**********************************************************************************************************************
 *                                         RF  global macro                                                           *
 *********************************************************************************************************************/
/**
 *  @brief This define serve to calculate the DMA length of packet.
 */
#define 	rf_tx_packet_dma_len(rf_data_len)			(((rf_data_len)+3)/4)|(((rf_data_len) % 4)<<22)
//--------------------------------------------FOR BLE---------------------------------------------------------------//
/**
 *  @brief Those setting of offset according to ble packet format, so this setting for ble only.
 */
#define 	RF_BLE_DMA_RFRX_LEN_HW_INFO					0
#define 	RF_BLE_DMA_RFRX_OFFSET_HEADER				4
#define 	RF_BLE_DMA_RFRX_OFFSET_RFLEN				5
#define 	RF_BLE_DMA_RFRX_OFFSET_DATA					6

/**
 *  @brief According to the packet format find the information of packet through offset
 */
#define 	rf_ble_dma_rx_0ffset_crc24(p)				(p[RF_BLE_DMA_RFRX_OFFSET_RFLEN]+6)  //data len:3
#define 	rf_ble_dma_rx_offset_time_stamp(p)			(p[RF_BLE_DMA_RFRX_OFFSET_RFLEN]+9)  //data len:4
#define 	rf_ble_dma_rx_offset_freq_offset(p)			(p[RF_BLE_DMA_RFRX_OFFSET_RFLEN]+13) //data len:2
#define 	rf_ble_dma_rx_offset_rssi(p)				(p[RF_BLE_DMA_RFRX_OFFSET_RFLEN]+15) //data len:1, signed
#define		rf_ble_packet_length_ok(p)					( *((unsigned int*)p) == p[5]+13)    			//dma_len must 4 byte aligned
#define		rf_ble_packet_crc_ok(p)						((p[(p[5]+5 + 11)] & 0x01) == 0x0)
//-------------------------------------------------------------------------------------------------------------------//

/**
 *  @brief According to different packet format find the crc check digit
 */
#define     rf_zigbee_packet_crc_ok(p)       			((p[(p[4]+9+3)] & 0x51) == 0x0)
#define     rf_hybee_packet_crc_ok(p)       			((p[(p[4]+9+3)] & 0x51) == 0x0)

#define     rf_pri_esb_packet_crc_ok(p)            		((p[((p[4] & 0x3f) + 11+3)] & 0x01) == 0x00)
#define     rf_pri_sb_packet_crc_ok(p)              	((p[(reg_rf_sblen & 0x3f)+4+9] & 0x01) == 0x00)

/**********************************************************************************************************************
 *                                       RF global data type                                                          *
 *********************************************************************************************************************/

/**
 *  @brief  select status of rf.
 */
typedef enum {
    RF_MODE_TX = 0,		/**<  Tx mode */
    RF_MODE_RX = 1,		/**<  Rx mode */
    RF_MODE_AUTO=2		/**<  Auto mode */
} rf_status_e;

/**
 *  @brief  select RX_CYC2LNA and TX_CYC2PA pin;
 */

typedef enum {
	RF_RFFE_RX_PB1 = GPIO_PB1,	/**<  pb1 as rffe rx pin */
    RF_RFFE_RX_PD6 = GPIO_PD6,	/**<  pd6 as rffe rx pin */
    RF_RFFE_RX_PE4 = GPIO_PE4	/**<  pe4 as rffe rx pin */
} rf_lna_rx_pin_e;


typedef enum {
	RF_RFFE_TX_PB0 = GPIO_PB0,	/**<  pb0 as rffe tx pin */
	RF_RFFE_TX_PB6 = GPIO_PB6,	/**<  pb6 as rffe tx pin */
    RF_RFFE_TX_PD7 = GPIO_PD7,	/**<  pd7 as rffe tx pin */
    RF_RFFE_TX_PE5 = GPIO_PE5	/**<  pe5 as rffe tx pin */
} rf_pa_tx_pin_e;

/**
 *  @brief  Define power list of RF
 */
typedef enum {
	 /*VBAT*/
	 RF_POWER_P9p11dBm = 63,  /**<  9.1 dbm */
	 RF_POWER_P8p57dBm  = 45, /**<  8.6 dbm */
	 RF_POWER_P8p05dBm  = 35, /**<  8.1 dbm */
	 RF_POWER_P7p45dBm  = 27, /**<  7.5 dbm */
	 RF_POWER_P6p98dBm  = 23, /**<  7.0 dbm */
	 RF_POWER_P5p68dBm  = 18, /**<  6.0 dbm */
	 /*VANT*/
	 RF_POWER_P4p35dBm  = BIT(7) | 63,   /**<   4.4 dbm */
	 RF_POWER_P3p83dBm  = BIT(7) | 50,   /**<   3.8 dbm */
	 RF_POWER_P3p25dBm  = BIT(7) | 41,   /**<   3.3 dbm */
	 RF_POWER_P2p79dBm  = BIT(7) | 36,   /**<   2.8 dbm */
	 RF_POWER_P2p32dBm  = BIT(7) | 32,   /**<   2.3 dbm */
	 RF_POWER_P1p72dBm  = BIT(7) | 26,   /**<   1.7 dbm */
	 RF_POWER_P0p80dBm  = BIT(7) | 22,   /**<   0.8 dbm */
	 RF_POWER_P0p01dBm  = BIT(7) | 20,   /**<   0.0 dbm */
	 RF_POWER_N0p53dBm  = BIT(7) | 18,   /**<  -0.5 dbm */
	 RF_POWER_N1p37dBm  = BIT(7) | 16,   /**<  -1.4 dbm */
	 RF_POWER_N2p01dBm  = BIT(7) | 14,   /**<  -2.0 dbm */
	 RF_POWER_N3p37dBm  = BIT(7) | 12,   /**<  -3.4 dbm */
	 RF_POWER_N4p77dBm  = BIT(7) | 10,   /**<  -4.8 dbm */
	 RF_POWER_N6p54dBm = BIT(7) | 8,     /**<  -6.5 dbm */
	 RF_POWER_N8p78dBm = BIT(7) | 6,     /**<  -8.8 dbm */
	 RF_POWER_N12p06dBm = BIT(7) | 4,    /**<  -12.1 dbm */
	 RF_POWER_N17p83dBm = BIT(7) | 2,    /**<  -17.8 dbm */
	 RF_POWER_N23p54dBm = BIT(7) | 1,    /**<  -23.5 dbm */

	 RF_POWER_N30dBm    = 0xff,          /**<  -30 dbm */
	 RF_POWER_N50dBm    = BIT(7) | 0,    /**<  -50 dbm */

} rf_power_level_e;

/**
 *  @brief  Define power index list of RF
 */
typedef enum {
	 /*VBAT*/
	 RF_POWER_INDEX_P9p11dBm,	/**< power index of 9.1 dbm */
	 RF_POWER_INDEX_P8p57dBm,	/**< power index of 8.6 dbm */
	 RF_POWER_INDEX_P8p05dBm,	/**< power index of 8.1 dbm */
	 RF_POWER_INDEX_P7p45dBm,	/**< power index of 7.5 dbm */
	 RF_POWER_INDEX_P6p98dBm,	/**< power index of 7.0 dbm */
	 RF_POWER_INDEX_P5p68dBm,	/**< power index of 6.0 dbm */
	 /*VANT*/
	 RF_POWER_INDEX_P4p35dBm,	/**< power index of 4.4 dbm */
	 RF_POWER_INDEX_P3p83dBm,	/**< power index of 3.8 dbm */
	 RF_POWER_INDEX_P3p25dBm,	/**< power index of 3.3 dbm */
	 RF_POWER_INDEX_P2p79dBm,	/**< power index of 2.8 dbm */
	 RF_POWER_INDEX_P2p32dBm,	/**< power index of 2.3 dbm */
	 RF_POWER_INDEX_P1p72dBm,	/**< power index of 1.7 dbm */
	 RF_POWER_INDEX_P0p80dBm,	/**< power index of 0.8 dbm */
	 RF_POWER_INDEX_P0p01dBm,	/**< power index of 0.0 dbm */
	 RF_POWER_INDEX_N0p53dBm,	/**< power index of -0.5 dbm */
	 RF_POWER_INDEX_N1p37dBm,	/**< power index of -1.4 dbm */
	 RF_POWER_INDEX_N2p01dBm,	/**< power index of -2.0 dbm */
	 RF_POWER_INDEX_N3p37dBm,	/**< power index of -3.4 dbm */
	 RF_POWER_INDEX_N4p77dBm,	/**< power index of -4.8 dbm */
	 RF_POWER_INDEX_N6p54dBm,	/**< power index of -6.5 dbm */
	 RF_POWER_INDEX_N8p78dBm,	/**< power index of -8.8 dbm */
	 RF_POWER_INDEX_N12p06dBm,	/**< power index of -12.1 dbm */
	 RF_POWER_INDEX_N17p83dBm,	/**< power index of -17.8 dbm */
	 RF_POWER_INDEX_N23p54dBm,	/**< power index of -23.5 dbm */
} rf_power_level_index_e;



/**
 *  @brief  Define RF mode
 */
typedef enum {
	RF_MODE_BLE_2M 		   =    BIT(0),		/**< ble 2m mode */
	RF_MODE_BLE_1M 		   = 	BIT(1),		/**< ble 1M mode */
    RF_MODE_BLE_1M_NO_PN   =    BIT(2),		/**< ble 1M close pn mode */
	RF_MODE_ZIGBEE_250K    =    BIT(3),		/**< zigbee 250K mode */
    RF_MODE_LR_S2_500K     =    BIT(4),		/**< ble 500K mode */
    RF_MODE_LR_S8_125K     =    BIT(5),		/**< ble 125K mode */
    RF_MODE_PRIVATE_250K   =    BIT(6),		/**< private 250K mode */
    RF_MODE_PRIVATE_500K   =    BIT(7),		/**< private 500K mode */
    RF_MODE_PRIVATE_1M     =    BIT(8),		/**< private 1M mode */
    RF_MODE_PRIVATE_2M     =    BIT(9),		/**< private 2M mode */
    RF_MODE_ANT     	   =    BIT(10),	/**< ant mode */
    RF_MODE_BLE_2M_NO_PN   =    BIT(11),	/**< ble 2M close pn mode */
    RF_MODE_HYBEE_1M   	   =    BIT(12),	/**< hybee 1M mode */
    RF_MODE_HYBEE_2M   	   =    BIT(13),	/**< hybee 2M mode */
    RF_MODE_HYBEE_500K     =    BIT(14),	/**< hybee 500K mode */
} rf_mode_e;



/**
 *  @brief  Define RF channel
 */
typedef enum {
	 RF_CHANNEL_0   =    BIT(0),	/**< RF channel 0 */
	 RF_CHANNEL_1   =    BIT(1),	/**< RF channel 1 */
	 RF_CHANNEL_2   =    BIT(2),	/**< RF channel 2 */
	 RF_CHANNEL_3   =    BIT(3),	/**< RF channel 3 */
	 RF_CHANNEL_4   =    BIT(4),	/**< RF channel 4 */
	 RF_CHANNEL_5   =    BIT(5),	/**< RF channel 5 */
	 RF_CHANNEL_NONE =   0x00,		/**< none RF channel*/
	 RF_CHANNEL_ALL =    0x0f,		/**< all RF channel */
} rf_channel_e;

/**********************************************************************************************************************
 *                                         RF global constants                                                        *
 *********************************************************************************************************************/
extern const rf_power_level_e rf_power_Level_list[30];


/**********************************************************************************************************************
 *                                         RF function declaration                                                    *
 *********************************************************************************************************************/


/**
 * @brief   	This function serves to judge the statue of  RF receive.
 * @return  	none.
 */
static inline unsigned char rf_receiving_flag(void)
{
	//if the value of [2:0] of the reg_0x140840 isn't 0 , it means that the RF is in the receiving packet phase.(confirmed by junwen)
	return ((read_reg8(0x140840)&0x07) > 1);
}


/**
 * @brief	  	This function serves to set the which irq enable
 * @param[in]	mask Options that need to be enabled.
 * @return	 	Yes: 1, NO: 0.
 */
static inline void rf_set_irq_mask(rf_irq_e mask)
{
	BM_SET(reg_rf_irq_mask,mask);
}


/**
 * @brief	  	This function serves to clear the TX/RX irq mask
 * @param[in]   mask RX/TX irq value.
 * @return	 	none
 */
static inline void rf_clr_irq_mask(rf_irq_e mask)
{
	BM_CLR(reg_rf_irq_mask,mask);
}


/**
 *	@brief	  	This function serves to determine whether sending a packet of data is finished
 *	@param[in]	mask RX/TX irq status
 *	@return	 	Yes: 1, NO: 0.
 */
static inline unsigned short rf_get_irq_status(rf_irq_e mask)
{
	return ((unsigned short )BM_IS_SET(reg_rf_irq_status,mask));
}


/**
 *@brief	This function serves to clear the Tx/Rx finish flag bit.
 *			After all packet data are sent, corresponding Tx finish flag bit
 *			will be set as 1.By reading this flag bit, it can check whether
 *			packet transmission is finished. After the check, it is needed to
 *			manually clear this flag bit so as to avoid misjudgment.
 *@return	none
 */
static inline void rf_clr_irq_status(rf_irq_e mask)
{
	 BM_SET(reg_rf_irq_status, mask);
}


/**
 * @brief   	This function serves to settle adjust for RF Tx.This function for adjust the differ time
 * 				when rx_dly enable.
 * @param[in]   txstl_us   adjust TX settle time.
 * @return  	none
 */
static inline void 	rf_tx_settle_us(unsigned short txstl_us)
{
	REG_ADDR16(0x80140a04) = txstl_us;
}


/**
 * @brief   	This function serves to set RF access code.
 * @param[in]   acc   the value of access code.
 * @return  	none.
 */
static inline void rf_access_code_comm (unsigned int acc)
{
	reg_rf_access_code = acc;
	//The following two lines of code are for trigger access code in S2,S8 mode.It has no effect on other modes.
	reg_rf_modem_mode_cfg_rx1_0 &= ~FLD_RF_LR_TRIG_MODE;
	write_reg8(0x140c25,read_reg8(0x140c25)|0x01);
}


/**
 * @brief		this function is to enable/disable each access_code channel for
 *				RF Rx terminal.
 * @param[in]	pipe  	Bit0~bit5 correspond to channel 0~5, respectively.
 *						0:Disable 1:Enable.
 *						If "enable" is set as 0x3f (i.e. 00111111),
 *						all access_code channels (0~5) are enabled.
 * @return	 	none
 */
static inline void rf_rx_acc_code_pipe_en(rf_channel_e pipe)
{
    write_reg8(0x140c4d, (read_reg8(0x140c4d)&0xc0) | pipe); //rx_access_code_chn_en
}


/**
 * @brief		this function is to select access_code channel for RF tx terminal.
 * @param[in]	pipe  	Bit0~bit2 the value correspond to channel 0~5, respectively.
 *						if value > 5 enable channel 5.And only 1 channel can be selected everytime.
 *						0:Disable 1:Enable
 *						If "enable" is set as 0x7 (i.e. 0111),
 *						the access_code channel (5) is enabled.
 * @return	 	none
 */
static inline void rf_tx_acc_code_pipe_en(rf_channel_e pipe)
{
    write_reg8(0x140a15, (read_reg8(0x140a15)&0xf8) | pipe); //Tx_Channel_man[2:0]
}


/**
 * @brief 	  This function serves to reset RF Tx/Rx mode.
 * @return 	  none.
 */
static inline void rf_set_tx_rx_off(void)
{
	write_reg8 (0x80140a16, 0x29);
	write_reg8 (0x80140828, 0x80);	// rx disable
	write_reg8 (0x80140a02, 0x45);	// reset tx/rx state machine
}


/**
 * @brief    This function serves to turn off RF auto mode.
 * @return   none.
 */
static inline void rf_set_tx_rx_off_auto_mode(void)
{
	write_reg8 (0x80140a00, 0x80);
}


/**
 * @brief    This function serves to set CRC advantage.
 * @return   none.
 */
static inline void rf_set_ble_crc_adv (void)
{
	write_reg32 (0x80140824, 0x555555);
}


/**
 * @brief  	  	This function serves to set CRC value for RF.
 * @param[in]  	crc  CRC value.
 * @return 		none.
 */
static inline void rf_set_ble_crc_value (unsigned int crc)
{
	write_reg32 (0x80140824, crc);
}


/**
 * @brief  	   This function serves to set the max length of rx packet.Use byte_len to limit what DMA
 * 			   moves out will not exceed the buffer size we define.And old chip do this through dma size.
 * @param[in]  byte_len  the longest of rx packet.
 * @return     none.
 */
static inline void rf_set_rx_maxlen(unsigned int byte_len)
{
	reg_rf_rxtmaxlen = byte_len;
}



/**
 * @brief	  	This function serves to DMA rxFIFO address
 *	            The function apply to the configuration of one rxFiFO when receiving packets,
 *	            In this case,the rxFiFo address can be changed every time a packet is received
 *	            Before setting, call the function "rf_set_rx_dma" to clear DMA fifo mask value(set 0)
 * @param[in]	rx_addr   the DMA rxFIFO address.
 * @return	 	none
 */
static inline void rf_set_rx_buffer(unsigned int rx_addr)
{
	dma_set_dst_address(DMA1,convert_ram_addr_cpu2bus(rx_addr));
}


/**
 * @brief      This function serves to initiate information of RF
 * @return	   none.
 */
void rf_mode_init(void);


/**
 * @brief     This function serves to set ble_1M  mode of RF
 * @return	  none.
 */
void rf_set_ble_1M_mode(void);


/**
 * @brief     This function serves to  set ble_1M_NO_PN  mode of RF
 * @return	  none.
 */
void rf_set_ble_1M_NO_PN_mode(void);


/**
 * @brief     This function serves to set ble_2M  mode of RF
 * @return	  none.
 */
void rf_set_ble_2M_mode(void);


/**
 * @brief     This function serves to set ble_2M_NO_PN  mode of RF
 * @return	  none.
 */
void rf_set_ble_2M_NO_PN_mode(void);


/**
 * @brief     This function serves to set ble_500K  mode of RF
 * @return	  none.
 */
void rf_set_ble_500K_mode(void);


/**
 * @brief     This function serves to  set zigbee_125K  mode of RF
 * @return	  none.
 */
void rf_set_ble_125K_mode(void);


/**
 * @brief     This function serves to set zigbee_250K  mode of RF
 * @return	  none.
 */
void rf_set_zigbee_250K_mode(void);


/**
 * @brief     This function serves to set pri_250K  mode of RF
 * @return	  none.
 */
void rf_set_pri_250K_mode(void);


/**
 * @brief     This function serves to  set pri_500K  mode of RF
 * @return	  none.
 */
void rf_set_pri_500K_mode(void);


/**
 * @brief     This function serves to set pri_1M  mode of RF
 * @return	  none.
 */
void rf_set_pri_1M_mode(void);


/**
 * @brief     This function serves to set pri_2M  mode of RF
 * @return	  none.
 */
void rf_set_pri_2M_mode(void);


/**
 * @brief     This function serves to set hybee_500K  mode of RF
 * @return	   none.
 */
void rf_set_hybee_500K_mode(void);


/**
 * @brief     This function serves to set hybee_2M  mode of RF
 * @return	  none.
 */
void rf_set_hybee_2M_mode(void);


/**
 * @brief     This function serves to set hybee_1M  mode of RF
 * @return	   none.
 */
void rf_set_hybee_1M_mode(void);


/**
 * @brief     This function serves to set RF tx DMA setting
 * @param[in] fifo_depth  		tx chn deep.
 * @param[in] fifo_byte_size    tx_idx_addr = {tx_chn_adr*bb_tx_size,4'b0}
 * @return	  none.
 */
void rf_set_tx_dma(unsigned char fifo_depth,unsigned char fifo_byte_size);


/**
 * @brief     This function serves to rx dma setting
 * @param[in] buff 		 	  DMA rx buffer.
 * @param[in] wptr_mask  	  DMA fifo mask value (0~fif0_num-1)
 * @param[in] fifo_byte_size  The length of one dma buffer
 * @return	  none.
 */
void rf_set_rx_dma(unsigned char *buff,unsigned char wptr_mask,unsigned char fifo_byte_size);


/**
 * @brief     This function serves to trigger srx on
 * @param[in] tick  Receive packet after tick delay.
 * @return	  none.
 */
void rf_start_srx(unsigned int tick);


/**
 * @brief	  	This function serves to get rssi
 * @return	 	rssi value
 */
signed char rf_get_rssi(void);


/**
 * @brief	  	This function serves to set pin for RFFE of RF
 * @param[in]   tx_pin   select pin to send
 * @param[in]   rx_pin   select pin to receive
 * @return	 	none
 */
void rf_set_rffe_pin(rf_pa_tx_pin_e tx_pin, rf_lna_rx_pin_e rx_pin);



/**
 * @brief  	 	This function serves to set RF Tx mode.
 * @return  	none.
 */
void rf_set_txmode(void);


/**
 * @brief	  	This function serves to set RF Tx packet address to DMA src_addr.
 * @param[in]	addr   the address of RF to send packet.
 * @return	 	none.
 */
void rf_tx_pkt(void* addr);


/**
 * @brief	  	This function serves to judge RF Tx/Rx state.
 * @param[in]	rf_status   Tx/Rx status.
 * @param[in]	rf_channel  RF channel.
 * @return	 	failed:-1, else success.
 */
int rf_set_trx_state(rf_status_e rf_status, signed char rf_channel);


/**
 * @brief   	This function serves to set RF no pn mode baseband channel.
 * @param[in]   chn   freq_num.
 * @return  	none.
 */
void rf_set_chn(signed char chn);


/**
 * @brief   	This function serves to set pri sb mode enable.
 * @return  	none.
 */
void rf_private_sb_en(void);


/**
 * @brief   	This function serves to set pri sb mode payload length.
 * @param[in]   pay_len  packet payload length.
 * @return  	none.
 */
void rf_set_private_sb_len(int pay_len);


/**
 * @brief   	This function serves to disable pn of ble mode.
 * @return  	none.
 */
void rf_pn_disable(void);


/**
 * @brief   	This function serves to get the right fifo packet.
 * @param[in]   fifo_num   the number of fifo set in dma.
 * @param[in]   fifo_dep   deepth of each fifo set in dma.
 * @param[in]   addr      address of rx packet.
 * @return  	the next rx_packet address.
 */
u8* rf_get_rx_packet_addr(int fifo_num,int fifo_dep,void* addr);


/**
 * @brief   	This function serves to set RF power level.
 * @param[in]   level 	 the RF power types.
 * @return 		none.
 */
void rf_set_power_level (rf_power_level_e level);


/**
 * @brief   	This function serves to set RF power level index.
 * @param[in]   idx 	 the RF power types.
 * @return  	none.
 */
_attribute_ram_code_sec_noinline_ void rf_set_power_level_index(rf_power_level_index_e idx);


/**
 * @brief	  	This function serves to close internal cap;
 * @return	 	none
 */
void rf_turn_off_internal_cap(void);


/**
 * @brief	  	This function serves to update the value of internal cap.
 * @param[in]  	value   set the internal cap value
 * @return	 	none
 */
void rf_update_internal_cap(unsigned char value);


/**
 * @brief	  	This function serves to get RF status.
 * @return	 	RF Rx/Tx status.
 */
rf_status_e rf_get_trx_state(void);

/**
 * @brief     	This function serves to RF trigger stx
 * @param[in] 	addr  DMA tx buffer.
 * @param[in] 	tick  Send after tick delay.
 * @return	   	none.
 */
_attribute_ram_code_sec_noinline_ void rf_start_stx(void* addr, unsigned int tick);


/**
 * @brief     	This function serves to RF trigger stx2rx
 * @param[in] 	addr  DMA tx buffer.
 * @param[in] 	tick  Send after tick delay.
 * @return	    none.
 */
_attribute_ram_code_sec_noinline_ void rf_start_stx2rx  (void* addr, unsigned int tick);


/**
 * @brief   	This function serves to set RF baseband channel.This function is suitable for ble open PN mode
 * @param[in]   chn_num  channel numbers.
 * @return  	none.
 */
_attribute_ram_code_sec_noinline_ void rf_set_ble_chn(signed char chn_num);



/**
 * @brief   	This function serves to set RF Rx manual on.
 * @return  	none.
 */
_attribute_ram_code_sec_noinline_ void rf_set_rxmode(void);


/**
 * @brief	  	This function serves to start Rx of auto mode. In this mode,
 *				RF module stays in Rx status until a packet is received or it fails to receive packet when timeout expires.
 *				Timeout duration is set by the parameter "tick".
 *				The address to store received data is set by the function "addr".
 * @param[in]	addr   The address to store received data.
 * @param[in]	tick   Unit is us. It indicates timeout duration in Rx status.Max value: 0xffffff (16777215)
 * @return	 	none
 */
_attribute_ram_code_sec_noinline_ void rf_start_brx  (void* addr, unsigned int tick);


/**
 * @brief	  	This function serves to start tx of auto mode. In this mode,
 *				RF module stays in tx status until a packet is sent or it fails to sent packet when timeout expires.
 *				Timeout duration is set by the parameter "tick".
 *				The address to store send data is set by the function "addr".
 * @param[in]	addr   The address to store send data.
 * @param[in]	tick   Unit is us. It indicates timeout duration in Rx status.Max value: 0xffffff (16777215)
 * @return	 	none
 */
_attribute_ram_code_sec_noinline_ void rf_start_btx (void* addr, unsigned int tick);

/*******************************      BLE Stack Use     ******************************/
 
 /**
 * This define for ble debug the effect of rx_dly.
 * when this function turn on the time of rx_dly will shorten 6.3us
 */
#define 	RF_RX_SHORT_MODE_EN			1
 
/*
 * While rf rx dma fifo is moving data,audio dma fifo is also moving data.
 * The rx fifo dma length may not be updated, instead, write to other ram addresses.
 * Result in ble disconnection.
 * Need to write rx dma write_num_en 0,And use the new crc verification method.
 */
#ifndef FIX_RF_DMA_REWRITE
#define FIX_RF_DMA_REWRITE		1
#endif

#define DMA_RFRX_LEN_HW_INFO				0	// 826x: 8
#define DMA_RFRX_OFFSET_HEADER				4	// 826x: 12
#define DMA_RFRX_OFFSET_RFLEN				5   // 826x: 13
#define DMA_RFRX_OFFSET_DATA				6	// 826x: 14

#define RF_TX_PAKET_DMA_LEN(rf_data_len)		(((rf_data_len)+3)/4)|(((rf_data_len) % 4)<<22)
#define DMA_RFRX_OFFSET_CRC24(p)			(p[DMA_RFRX_OFFSET_RFLEN]+6)  //data len:3
#define DMA_RFRX_OFFSET_TIME_STAMP(p)		(p[DMA_RFRX_OFFSET_RFLEN]+9)  //data len:4
#define DMA_RFRX_OFFSET_FREQ_OFFSET(p)		(p[DMA_RFRX_OFFSET_RFLEN]+13) //data len:2
#define DMA_RFRX_OFFSET_RSSI(p)				(p[DMA_RFRX_OFFSET_RFLEN]+15) //data len:1, signed

#define		RF_BLE_PACKET_LENGTH_OK(p)				( *((unsigned int*)p) == p[5]+13)    			//dma_len must 4 byte aligned
#define		RF_BLE_PACKET_CRC_OK(p)					((p[*((unsigned int*)p) + 3] & 0x01) == 0x0)

#if		(FIX_RF_DMA_REWRITE)
#define		RF_BLE_RF_PAYLOAD_LENGTH_OK(p)			(p[5] <= reg_rf_rxtmaxlen)
#define		RF_BLE_RF_PACKET_CRC_OK(p)				((p[p[5]+5+11] & 0x01) == 0x0)
#endif


_attribute_ram_code_ void ble_rf_set_rx_dma(unsigned char *buff, unsigned char fifo_byte_size);

_attribute_ram_code_ void ble_rf_set_tx_dma(unsigned char fifo_dep, unsigned char fifo_byte_size);

_attribute_ram_code_ void ble_tx_dma_config(void);

_attribute_ram_code_ void ble_rx_dma_config(void);

_attribute_ram_code_ void rf_ble_1m_param_init(void);

/**
*	@brief     This function serves to initiate the mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
_attribute_ram_code_ void rf_drv_init (rf_mode_e rf_mode);

/**
 * @brief   This function serves to settle adjust for RF Tx.This function for adjust the differ time
 * 			when rx_dly enable.
 * @param   txstl_us - adjust TX settle time.
 * @return  none
 */
static inline void 	rf_tx_settle_adjust(unsigned short txstl_us)
{
	REG_ADDR16(0x80140a04) = txstl_us;
}

/**
*	@brief     This function serves to reset RF BaseBand
*	@param[in] none.
*	@return	   none.
*/
static inline void rf_reset_baseband(void)
{
	REG_ADDR8(0x801404e3) = 0;		//rf_reset_baseband,rf reg need re-setting
	REG_ADDR8(0x801404e3) = BIT(0);			//release reset signal
}

/**
 * @brief   This function serves to set RF access code value.
 * @param[in]   ac - the address value.
 * @return  none
 */
static inline void rf_set_ble_access_code_value (unsigned int ac)
{
	write_reg32 (0x80140808, ac);
}

/**
 * @brief   This function serves to reset function for RF.
 * @param   none
 * @return  none
 *******************need driver change
 */
static inline void rf_reset_sn_nesn(void)
{
	REG_ADDR8(0x80140a01) =  0x01;
}

/**
 * @brief     This function performs to enable RF Tx.
 * @param[in] none.
 * @return    none.
 */
static inline void rf_ble_tx_on ()
{
	write_reg8  (0x80140a02, 0x45 | BIT(4));	// TX enable
}

/**
 * @brief     This function performs to done RF Tx.
 * @param[in] none.
 * @return    none.
 */
static inline void rf_ble_tx_done ()
{
	write_reg8  (0x80140a02, 0x45);
}

#endif

