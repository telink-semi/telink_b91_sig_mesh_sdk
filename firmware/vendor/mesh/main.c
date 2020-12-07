/********************************************************************************************************
 * @file	main.c
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
#include "drivers/9518/watchdog.h"
#include "vendor/common/user_config.h"
#include "drivers/9518/rf.h"
#include "drivers/9518/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "app_config.h"
#include "../../drivers.h"

#include "tl_common.h"
#include "../common/blt_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
extern void user_init();
extern void main_loop ();
void blc_pm_select_none();

#if (HCI_ACCESS==HCI_USE_UART)
#include "proj/drivers/uart.h"
extern my_fifo_t hci_rx_fifo;

u16 uart_tx_irq=0, uart_rx_irq=0;

_attribute_ram_code_ void irq_uart_handle()
{
	unsigned char irqS = reg_dma_rx_rdy0;
	if(irqS & FLD_DMA_CHN_UART_RX)	//rx
	{
		uart_rx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_RX;
		u8* w = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
		if(w[0]!=0)
		{
			my_fifo_next(&hci_rx_fifo);
			u8* p = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
			reg_dma0_addr = (u16)((u32)p);
		}
	}

	if(irqS & FLD_DMA_CHN_UART_TX)	//tx
	{
		uart_tx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_TX;
	}
}
#endif

#if IRQ_TIMER1_ENABLE
_attribute_ram_code_ void irq_timer_handle()
{
    u32 src = reg_irq_src;
    static u32 A_debug_irq_cnt =0;
    if(src & FLD_IRQ_TMR1_EN){
       A_debug_irq_cnt++;
       reg_tmr_sta = FLD_TMR_STA_TMR1;
       gpio_write(GPIO_PA1,A_debug_irq_cnt%2);
    }
}
#endif

#if	IRQ_GPIO_ENABLE

void irq_gpio_handle()
{
	u32 src = reg_irq_src;
	if(src & FLD_IRQ_GPIO_EN){
		gpio_irq_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_EN;        // clear irq_gpio irq flag		
	}

	/************* gpio irq risc0 *************/
	if(src & FLD_IRQ_GPIO_RISC0_EN){
		gpio_risc0_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC0_EN;        // clear irq_gpio irq flag				
	}

	/************* gpio irq risc1 *************/
	if(src & FLD_IRQ_GPIO_RISC1_EN){
		gpio_risc1_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC1_EN;        // clear irq_gpio irq flag		
	}
	#if (!(__TL_LIB_8258__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278)))
	if(src & FLD_IRQ_GPIO_RISC2_EN){
		gpio_risc2_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
	}
	#endif
}
#endif


_attribute_ram_code_ void irq_handler(void)
{
#if DUAL_MODE_ADAPT_EN
	if(rf_mode == RF_MODE_ZIGBEE){
		irq_zigbee_sdk_handler();
	}else
#endif
	{
		irq_blt_sdk_handler ();  //ble irq proc
	}
#if IRQ_TIMER1_ENABLE
irq_timer_handle();
#endif

}

/**
 * @brief		BLE SDK RF interrupt handler.
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_
void rf_irq_handler(void)
{

	DBG_CHN10_HIGH;

	irq_handler();
	DBG_CHN10_LOW;

}

_attribute_ram_code_
void stimer_irq_handler(void)
{
	DBG_CHN9_HIGH;

	irq_handler();

	DBG_CHN9_LOW;
}

_attribute_ram_code_
void uart0_irq_handler(void)
{
#if (HCI_ACCESS==HCI_USE_UART)
	irq_uart_handle();
#endif
}

FLASH_ADDRESS_DEFINE;
#if(MCU_CORE_TYPE == MCU_CORE_8269)
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	clock_init();
	set_tick_per_us(CLOCK_SYS_CLOCK_HZ/1000000);

	gpio_init();

	rf_drv_init(CRYSTAL_TYPE);

	user_init ();

    irq_enable();

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278)||(MCU_CORE_TYPE == MCU_CORE_9518))
_attribute_ram_code_ int main (void)    //must run in ramcode
{
	FLASH_ADDRESS_CONFIG;
#if (PINGPONG_OTA_DISABLE && (0 == FW_START_BY_BOOTLOADER_EN))
	
    //ota_fw_check_over_write();  // must at first for main_
#endif
	blc_pm_select_internal_32k_crystal();
#if(MCU_CORE_TYPE == MCU_CORE_8258)
	cpu_wakeup_init();
#elif(MCU_CORE_TYPE == MCU_CORE_8278)
	cpu_wakeup_init(LDO_MODE,EXTERNAL_XTAL_24M);
#elif(MCU_CORE_TYPE == MCU_CORE_9518)
	sys_init(LDO_1P4_LDO_1P8);
#endif

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	CCLK_16M_HCLK_16M_PCLK_16M;
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	CCLK_24M_HCLK_24M_PCLK_24M;
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	CCLK_32M_HCLK_32M_PCLK_16M;
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	CCLK_48M_HCLK_48M_PCLK_24M;
#elif (CLOCK_SYS_CLOCK_HZ == 64000000)
	CCLK_64M_HCLK_32M_PCLK_16M;
#endif

	rf_drv_init(RF_MODE_BLE_1M);
	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

#if	(PM_DEEPSLEEP_RETENTION_ENABLE)
		if( pm_is_MCU_deepRetentionWakeup() ){
			//user_init_deepRetn ();
		}
		else
#endif
	{
		user_init();
	}


    irq_enable();
	#if (MESH_USER_DEFINE_MODE == MESH_IRONMAN_MENLO_ENABLE)
	LOG_USER_MSG_INFO(0, 0,"[mesh] Start from SIG Mesh", 0);
	#endif

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#endif
