/********************************************************************************************************
 * @file	usbhw.h
 *
 * @brief	This is the source file for B91
 *
 * @author	D.M.H
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#pragma once

#include "reg_include/register_9518.h"
#include "analog.h"
#include "gpio.h"
/**
 * @brief     This function servers to reset the pointer of control Endpoint.
 * @param[in] data - the value of control Endpoint
 * @return    none.
 */
static inline void usbhw_reset_ctrl_ep_ptr(void) {
	reg_ctrl_ep_ptr = 0;
}

/**
 * @brief     This function servers to get the irq status of control Endpoint.
 * @param[in] none.
 * @return    none.
 */
static inline unsigned int usbhw_get_ctrl_ep_irq(void) {
	return reg_ctrl_ep_irq_sta;
}

/**
 * @brief     This function servers to clear the irq status of control Endpoint.
 * @param[in] ep - select the Endpoint
 * @return    none.
 */
static inline void usbhw_clr_ctrl_ep_irq(int ep) {
#ifdef WIN32
	BM_CLR(reg_ctrl_ep_irq_sta, ep);
#else
	reg_ctrl_ep_irq_sta = ep;
#endif
}

/**
 * @brief     This function servers to set the value of control Endpoint.
 * @param[in] data - the value of control Endpoint
 * @return    none.
 */
static inline void usbhw_write_ctrl_ep_ctrl(unsigned char data) {
	reg_ctrl_ep_ctrl = data;
}

/**
 * @brief     This function servers to read the data of control Endpoint.
 * @param[in] none.
 * @return    the value of control Endpoint
 */
static inline unsigned char usbhw_read_ctrl_ep_data(void) {
	return reg_ctrl_ep_dat;
}

/**
 * @brief     This function servers to write the data of control Endpoint.
 * @param[in] data -  the value of control Endpoint
 * @return    none
 */
static inline void usbhw_write_ctrl_ep_data(unsigned char data) {
	reg_ctrl_ep_dat = data;
}

/**
 * @brief     This function servers to determine whether control Endpoint is busy.
 * @param[in] none.
 * @return    1: busy; 0: not busy.
 */
static inline int usbhw_is_ctrl_ep_busy(void) {
	return reg_ctrl_ep_irq_sta & FLD_USB_EP_BUSY;
}


/**
 * @brief     This function servers to reset the pointer of Endpoint.
 * @param[in] data - the value of control Endpoint
 * @return    none.
 */
static inline void usbhw_reset_ep_ptr(unsigned int ep) {
	reg_usb_ep_ptr(ep & 0x07) = 0;
}



/**
 * @brief     This function servers to get the irq status of Endpoint.
 * @param[in] none.
 * @return    none.
 */
static inline unsigned int usbhw_get_eps_irq(void) {
	return reg_usb_irq;
}



/**
 * @brief     This function servers to clear the irq status of Endpoint.
 * @param[in] ep - select the Endpoint
 * @return    none.
 */
static inline void usbhw_clr_eps_irq(int ep) {
	reg_usb_irq = ep;
}

/**
 * @brief     This function servers to read the data of Endpoint.
 * @param[in] none.
 * @return    the value of Endpoint
 */
static inline unsigned char usbhw_read_ep_data(unsigned int ep) {
	return reg_usb_ep_dat(ep & 0x07);
}

/**
 * @brief     This function servers to write the data of Endpoint.
 * @param[in] data -  the value of Endpoint
 * @return    none
 */
static inline void usbhw_write_ep_data(unsigned int ep, unsigned char data) {
	reg_usb_ep_dat(ep & 0x07) = data;
}

/**
 * @brief     This function servers to enable the specified Endpoint.
 * @param[in] ep -  the value of Endpoint
 * @param[in] en -  1:enable,0:disable
 * @return    none
 */
static inline void usbhw_set_ep_en(unsigned int ep, unsigned char en) {
	if(en)
	{
		reg_usb_edp_en |= ep;
	}
	else
	{
		reg_usb_edp_en &= ~(ep);
	}
}

/**
 * @brief     This function servers to determine whether Endpoint is busy.
 * @param[in] none.
 * @return    1: busy; 0: not busy.
 */
static inline unsigned int usbhw_is_ep_busy(unsigned int ep) {
	return reg_usb_ep_ctrl(ep & 0x07) & FLD_USB_EP_BUSY;
}

/**
 * @brief     This function servers to set the specified data EndPoint to ack.
 * @param[in] ep -  select the data EndPoint.
 * @return    none.
 */
static inline void usbhw_data_ep_ack(unsigned int ep) {
	reg_usb_ep_ctrl(ep & 0x07) = FLD_USB_EP_BUSY;
}

/**
 * @brief     This function servers to set the specified data EndPoint to stall.
 * @param[in] ep -  select the data EndPoint.
 * @return    none.
 */
static inline void usbhw_data_ep_stall(unsigned int ep) {
	reg_usb_ep_ctrl(ep & 0x07) = FLD_USB_EP_STALL;
}


/**
 * @brief     This function servers to set the threshold of printer.
 * @param[in] th - set the threshold for printer
 * @return    none.
 */
static inline void usbhw_set_printer_threshold(unsigned char th) {
	reg_usb_ep8_send_thre = th;
}

enum {
	USB_EDP_PRINTER_IN = 8, // endpoint 8 is alias of enpoint 0,  becareful.  // default hw buf len = 64
	USB_EDP_MOUSE = 2,			// default hw buf len = 8
	USB_EDP_KEYBOARD_IN = 1,	// default hw buf len = 8
	USB_EDP_KEYBOARD_OUT = 3,	// default hw buf len = 16
	USB_EDP_AUDIO_IN = 4,		// default hw buf len = 64
	USB_EDP_PRINTER_OUT = 5,	// default hw buf len = 64
	USB_EDP_SPEAKER = 6,		// default hw buf len = 16
	USB_EDP_MIC = 7,			// default hw buf len = 16
	USB_EDP_MS_IN = USB_EDP_PRINTER_IN,		// mass storage
	USB_EDP_MS_OUT = USB_EDP_PRINTER_OUT,
	USB_EDP_SOMATIC_IN = USB_EDP_AUDIO_IN,		//  when USB_SOMATIC_ENABLE, USB_EDP_PRINTER_OUT disable
	USB_EDP_SOMATIC_OUT = USB_EDP_PRINTER_OUT,
    USB_EDP_CDC_IN = 4,
    USB_EDP_CDC_OUT = 5,
};

// #defined in the standard spec
enum {
	USB_HID_AUDIO       	= 2,
	USB_HID_MOUSE       	= 1,
	USB_HID_KB_MEDIA    	= 3,// media
	USB_HID_KB_SYS      	= 4,// system : power,sleep,wakeup
	USB_HID_SOMATIC			= 5,// somatic sensor,  may have many report ids
};


/**
 * @brief      This function disables the manual interrupt
 *             (Endpont8 is the alias of endpoint0)
 * @param[in]  m - the irq mode needs to set
 * @return     none
 */
void usbhw_disable_manual_interrupt(int m);

/**
 * @brief      This function enable the manual interrupt
 * @param[in]  m - the irq mode needs to set
 * @return     none
 */
void usbhw_enable_manual_interrupt(int m);

/**
 * @brief      This function sends a bulk of data to host via the specified endpoint
 * @param[in]  ep - the number of the endpoint
 * @param[in]  data - pointer to the data need to send
 * @param[in]  len - length in byte of the data need to send
 * @return     none
 */
void usbhw_write_ep(unsigned int ep, unsigned char * data, int len);

/**
 * @brief      This function sends two bytes data to host via the control endpoint
 *             (handy help function)
 * @param[in]  v - the two bytes data need to send
 * @return     none
 */
void usbhw_write_ctrl_ep_u16(unsigned short v);

/**
 * @brief   This function reads two bytes data from host via the control endpoint
 * @param   none
 * @return  the two bytes data read from the control endpoint
 */
unsigned short usbhw_read_ctrl_ep_u16(void);

/**
 * @brief      This function enables or disables the internal pull-up resistor
 *             of DP pin of USB interface
 * @param[in]  En - enables or disables the internal pull-up resistor(1: enable 0: disable)
 * @return     none
 */
static inline void usb_dp_pullup_en (int en)
{
	unsigned char dat = analog_read_reg8(0x0b);
	if (en) {
		dat = dat | BIT(7);
	}
	else
	{
		dat = dat & 0x7f ;
	}

	analog_write_reg8 (0x0b, dat);
}

/**
 * @brief      This function serves to power on or down USB module
 * @param[in]  En - 1: power on 0: power down
 * @return     none
 */
static inline void usb_power_on(unsigned char en)
{
	if(en)
	{
		analog_write_reg8(0x34,analog_read_reg8(0x34)|0x02);
	}
	else
	{
		analog_write_reg8(0x34,analog_read_reg8(0x34)&0xfd);
	}
}

/**
 * @brief      This function serves to set GPIO MUX function as DP and DM pin of USB
 * @param[in]  none.
 * @return     none.
 */
static inline void usb_set_pin_en(void)
{
	reg_gpio_func_mux(GPIO_PA5)=reg_gpio_func_mux(GPIO_PA5)&(~BIT_RNG(2,3));
	gpio_function_dis(GPIO_PA5);

	reg_gpio_func_mux(GPIO_PA6)=reg_gpio_func_mux(GPIO_PA6)&(~BIT_RNG(4,5));
	gpio_function_dis(GPIO_PA6);

	usb_dp_pullup_en (1);
}
