/********************************************************************************************************
 * @file	mesh_lpn.c
 *
 * @brief	for TLSR chips
 *
 * @author	telink
 * @date	Sep. 30, 2010
 *
 * @par     Copyright (c) 2017, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#include "tl_common.h"
#if !WIN32
#include "proj/mcu/watchdog_i.h"
#endif 
#include "stack/ble/ble.h"
#include "proj_lib/ble/blt_config.h"
#include "vendor/common/user_config.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
#include "app_health.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "lighting_model.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "mesh_common.h"
#include "vendor/common/app_heartbeat.h"
#include "blt_soft_timer.h"
#include "vendor/common/mesh_ota.h"
#if(HCI_ACCESS == HCI_USE_UART)
#include "drivers.h"
#endif

#if (FEATURE_LOWPOWER_EN || SPIRIT_PRIVATE_LPN_EN)
u8 lpn_provision_ok = 0;
#endif


#if FEATURE_LOWPOWER_EN
#if (IS_VC_PROJECT || TESTCASE_FLAG_ENABLE)
#define			DEBUG_SUSPEND				1
#else
#define			DEBUG_SUSPEND				0	// can not enable now due to using soft timer.
#endif

#define RETRY_POLL_AGAIN_FLIP_FSN_EN		1	// change FSN to retry the 2nd round.

//u8 fri_request_retry_max = FRI_REQ_RETRY_MAX;
lpn_sub_list_event_t mesh_lpn_subsc_pending;
u8 mesh_lpn_poll_md_pending = 0;
#if PTS_TEST_EN
u8 mesh_lpn_rx_master_key = 1;
#else
u8 mesh_lpn_rx_master_key = 0;
#endif

const u32 mesh_lpn_key_map[] = {SW1_GPIO, SW2_GPIO};
#define	MESH_LPN_CMD_KEY		        (SW2_GPIO)
#define	MESH_LPN_FACTORY_RESET_KEY		(SW1_GPIO)
u32 mesh_lpn_wakeup_key = 0;
u8 key_not_release = 0;
u32 active_time;

u8 lpn_mode = LPN_MODE_NORMAL;
u32 lpn_mode_tick = 0;
u32 lpn_wakeup_tick = 0;

STATIC_ASSERT(LPN_ADV_INTERVAL_MS > (FRI_ESTABLISH_REC_DELAY_MS + FRI_ESTABLISH_WIN_MS + 10));//FRI_ESTABLISH_PERIOD_MS
STATIC_ASSERT(FRI_REQ_TIMEOUT_MS > 1100);
STATIC_ASSERT(LPN_POLL_TIMEOUT_100MS * 100 >= FRI_POLL_INTERVAL_MS * 2); // timeout should not be too short.
STATIC_ASSERT(SUB_LIST_MAX_LPN <= SUB_LIST_CNT_MAX);    // SUB_LIST_MAX_LPN should be equal to sub_list_max later
STATIC_ASSERT(SUB_LIST_MAX_LPN == SUB_LIST_MAX_IN_ONE_MSG);    // comfirm later

#if GATT_LPN_EN
STATIC_ASSERT(MESH_DLE_MODE != MESH_DLE_MODE_EXTEND_BEAR); // extend bear need extend adv for friend ship
#endif

mesh_fri_ship_proc_lpn_t fri_ship_proc_lpn = {};	// for VC
mesh_lpn_subsc_list_t lpn_subsc_list;
mesh_subsc_list_retry_t subsc_list_retry = {};  // for retry procedure
mesh_lpn_par_t mesh_lpn_par = {0};

STATIC_ASSERT((FRI_REC_DELAY_MS > FRI_ESTABLISH_REC_DELAY_MS));


int is_friend_ship_link_ok_lpn()
{
	return mesh_lpn_par.link_ok;
}

u8 mesh_lpn_tx_network_cb(mesh_match_type_t *p_match_type, u8 sec_type)
{
#if PTS_TEST_EN
	if(is_friend_ship_link_ok_lpn()){
		sec_type = FRIENDSHIP;
	}
#endif
	return sec_type;
}

int is_unicast_friend_msg_to_fn(mesh_cmd_nw_t *p_nw)
{
    return (is_in_mesh_friend_st_lpn()
        && (mesh_lpn_par.LPNAdr == p_nw->src)
        && (mesh_lpn_par.FriAdr == p_nw->dst));    // must because of group address
}

void friend_cmd_send_request()
{
	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)&mesh_lpn_par.req, sizeof(mesh_ctl_fri_req_t),"send friend request sno:0x%x par:",mesh_adv_tx_cmd_sno);
	mesh_lpn_par.link_ok = 0;
    mesh_lpn_par.req.LPNCounter++;   // must before
    mesh_tx_cmd_layer_upper_ctl(CMD_CTL_REQUEST, (u8 *)&mesh_lpn_par.req, sizeof(mesh_ctl_fri_req_t), ele_adr_primary, ADR_ALL_NODES,0);
}

void friend_cmd_send_poll()
{
	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)(&mesh_lpn_par.poll), sizeof(mesh_ctl_fri_poll_t),"send friend poll sno:0x%x par:",mesh_adv_tx_cmd_sno);
    fri_ship_proc_lpn.poll_tick = clock_time()|1;
    mesh_tx_cmd_layer_upper_ctl(CMD_CTL_POLL, (u8 *)(&mesh_lpn_par.poll), sizeof(mesh_ctl_fri_poll_t), ele_adr_primary, mesh_lpn_par.FriAdr,0);	
}

void friend_cmd_send_subsc_add(u8 *par_subsc, u32 len)  // only LPN support
{
	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)par_subsc, len,"send friend sub list add, sno:0x%x par:",mesh_adv_tx_cmd_sno);
    subsc_list_retry.tick = clock_time();
    mesh_tx_cmd_layer_upper_ctl(CMD_CTL_SUBS_LIST_ADD, (u8 *)par_subsc, len, ele_adr_primary, mesh_lpn_par.FriAdr,0);
}

void friend_cmd_send_subsc_rmv(u8 *par_subsc, u32 len)  // only LPN support
{
	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)par_subsc, len,"send friend sub list remove sno:0x%x par:",mesh_adv_tx_cmd_sno);
    subsc_list_retry.tick = clock_time();
    mesh_tx_cmd_layer_upper_ctl(CMD_CTL_SUBS_LIST_REMOVE, (u8 *)par_subsc, len, ele_adr_primary, mesh_lpn_par.FriAdr,0);
}

void friend_subsc_repeat()
{
    if(fri_ship_proc_lpn.poll_retry){
        fri_ship_proc_lpn.poll_tick = clock_time()|1;
    }
    
    if(SUBSC_ADD == subsc_list_retry.retry_type){
        friend_cmd_send_subsc_add(&subsc_list_retry.TransNo, 1+subsc_list_retry.subsc_cnt*2);
    }else if(SUBSC_REMOVE == subsc_list_retry.retry_type){
        friend_cmd_send_subsc_rmv(&subsc_list_retry.TransNo, 1+subsc_list_retry.subsc_cnt*2);
    }
}

inline void friend_subsc_stop()
{
    subsc_list_retry.retry_cnt = 0;
}

int friend_subsc_list_cmd_start(u8 type, u16 *adr_list, u32 subsc_cnt)
{
    if(subsc_cnt > SUB_LIST_MAX_IN_ONE_MSG){
        return -1;
    }
    
    subsc_list_retry.subsc_cnt = subsc_cnt;
    subsc_list_retry.TransNo = lpn_subsc_list.TransNo;
    memset(subsc_list_retry.adr, 0, sizeof(subsc_list_retry.adr));
    memcpy(subsc_list_retry.adr, adr_list, subsc_cnt * 2);
    subsc_list_retry.retry_cnt = lpn_get_poll_retry_max();
    subsc_list_retry.retry_type = type;
    
    lpn_subsc_list.TransNo++;

    friend_subsc_repeat();  // send first message

    return 0;
}

void friend_subsc_add(u16 *adr_list, u32 subsc_cnt)
{
    friend_subsc_list_cmd_start(SUBSC_ADD, adr_list, subsc_cnt);
}

void friend_subsc_rmv(u16 *adr_list, u32 subsc_cnt)
{
    friend_subsc_list_cmd_start(SUBSC_REMOVE, adr_list, subsc_cnt);
    friend_subsc_list_rmv_adr((lpn_adr_list_t *)(&lpn_subsc_list.adr), (lpn_adr_list_t *)adr_list, subsc_cnt);
}

void mesh_friend_ship_set_st_lpn(u8 st)
{
    fri_ship_proc_lpn.status = st;
	mesh_lpn_adv_interval_update(1);
}

int is_in_mesh_friend_st_lpn()
{
	if(is_lpn_support_and_en){
    	return (is_friend_ship_link_ok_lpn() && (mesh_lpn_par.FriAdr != 0));
	}else{
		return 0;
	}
}

void mesh_friend_ship_proc_init_lpn()
{
    memset(&fri_ship_proc_lpn, 0, sizeof(mesh_fri_ship_proc_lpn_t));
}

void mesh_friend_ship_clear_LPN()
{
    mesh_ctl_fri_req_t req_bacup;
    memcpy(&req_bacup, &mesh_lpn_par.req, sizeof(req_bacup));
    memset(&mesh_lpn_par, 0, sizeof(mesh_lpn_par));
    memcpy(&mesh_lpn_par.req, &req_bacup, sizeof(req_bacup));
}

int mesh_lpn_subsc_pending_add(u16 op, u16 addr)
{
	mesh_lpn_subsc_pending.op = op;
	mesh_lpn_subsc_pending.sub_addr = addr;
	return 0;
}

void mesh_lpn_gatt_adv_refresh()
{
#if (GATT_LPN_EN)
	if(lpn_provision_ok && gatt_adv_send_flag){
		blt_soft_timer_update(&mesh_lpn_send_gatt_adv, ADV_INTERVAL_MS*1000);
	}
#else
	gatt_adv_send_flag = !lpn_provision_ok;
#endif
}

//------------softer timer cb function----------------------//
int mesh_lpn_send_gatt_adv()
{
	if(gatt_adv_send_flag && (BLS_LINK_STATE_ADV == get_blt_state())){
		lpn_quick_tx(1);
		return (ADV_INTERVAL_MS + (rand()%ADV_INTERVAL_RANDOM_MS))*1000;
	}
	return -1;
}

int mesh_lpn_send_mesh_cmd()
{
	int ret = 0;
	if(my_fifo_get(&mesh_adv_cmd_fifo)){
		lpn_quick_tx(0);
		ret = 1;
	}
	return ret;
}

int mesh_lpn_poll_receive_timeout(void)
{
	ENABLE_SUSPEND_MASK;
	return -1;
}

int mesh_lpn_rcv_delay_wakeup(void)
{
	app_enable_scan_all_device ();	
	mesh_send_adv2scan_mode(0);
	bls_pm_setSuspendMask (SUSPEND_DISABLE); // not enter sleep to receive packets	
	if(is_friend_ship_link_ok_lpn()){
		
		blt_soft_timer_update(&mesh_lpn_poll_receive_timeout, mesh_lpn_par.offer.RecWin*1000);
	}
	return -1;
}

extern	int	sdk_mainLoop_run_flag;
int mesh_lpn_poll_md_wakeup(void)
{
	int ret = -1;	
	if(!is_busy_tx_seg(0)){
		bltParam.sdk_mainLoop_run_flag = 1;
		//sdk_mainLoop_run_flag = 1; // not run mesh_loop_process in early wakeup to save time
	}
	else{
		ret = CMD_INTERVAL_MS * 1000; // for tx segment retry
	}
	
	if(mesh_lpn_send_mesh_cmd()){// fifo not empty, send first, maybe status message
		if(my_fifo_get(&mesh_adv_cmd_fifo)){
			ret = get_mesh_adv_interval()+(rand()%10)*1000;// add 0~10ms random
		}
		u8 *p_buf = my_fifo_get(&mesh_adv_cmd_fifo);
		if(is_in_mesh_friend_st_lpn() && is_busy_tx_seg(0) && is_tx_seg_one_round_ok()){
			if(0 == p_buf){
				ret = FRI_LPN_WAIT_SEG_ACK_MS * 1000; 
				fri_ship_proc_lpn.poll_retry = 0;
				mesh_lpn_poll_md_pending = 1; // poll for ack after one round
			}
		}	
	}
	else if(mesh_lpn_subsc_pending.op){
		if(CMD_CTL_SUBS_LIST_ADD == mesh_lpn_subsc_pending.op){
			friend_subsc_add(&mesh_lpn_subsc_pending.sub_addr, 1);
		}
		else if(CMD_CTL_SUBS_LIST_REMOVE == mesh_lpn_subsc_pending.op){
			friend_subsc_rmv(&mesh_lpn_subsc_pending.sub_addr, 1);
		}
		mesh_lpn_subsc_pending.op = 0;
	}
	else if(mesh_lpn_poll_md_pending){		
		mesh_friend_ship_start_poll();
	}
	else{
	}

	if((-1 == ret) && (mesh_lpn_subsc_pending.op || mesh_lpn_poll_md_pending)){
		ret = FRI_POLL_DELAY_FOR_MD_MS * 1000;
	}
	return ret;
}

//------------end softer timer cb function----------------------//
void mesh_friend_ship_proc_LPN(u8 *bear)
{
    if(get_blt_state() == BLS_LINK_STATE_CONN){
        return ;
    }
    
    static u32 t_rec_delay_and_win = 0;
    u32 poll_retry_interval_ms = t_rec_delay_and_win;
	u32 timeout_ms = (poll_retry_interval_ms*1000) * (2*2+1)/2;    // comfirm later
    #if (0 == DEBUG_PROXY_FRIEND_SHIP)
    if(pts_test_en && fri_ship_proc_lpn.status){
        // retry poll should be more quicklier during establish friend ship.
        #define RETRY_POLL_INTV_MS_MAX      170 // FRND-LPN-BI01 need retry 3 times during 1 second after get offer.
        if(poll_retry_interval_ms > RETRY_POLL_INTV_MS_MAX){
            poll_retry_interval_ms = RETRY_POLL_INTV_MS_MAX;
        }
    }
    #endif
	if(is_mesh_adv_cmd_fifo_empty()){
	    if(fri_ship_proc_lpn.poll_retry && (!bear)
	     && clock_time_exceed(fri_ship_proc_lpn.poll_tick, poll_retry_interval_ms*1000)){
	        fri_ship_proc_lpn.poll_retry--;
	        if(0 == fri_ship_proc_lpn.poll_retry){
	            if(FRI_ST_UPDATE == fri_ship_proc_lpn.status){
	                // retry request in "case FRI_ST_UPDATE:"
	            }else{
	                friend_subsc_stop();
	                mesh_cmd_sig_lowpower_heartbeat();
	                friend_ship_disconnect_cb_lpn();
	                mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);    // restart establish procedure
	            }
	        }else{
	        	#if RETRY_POLL_AGAIN_FLIP_FSN_EN
	        	if((FRI_POLL_RETRY_MAX == fri_ship_proc_lpn.poll_retry) && is_in_mesh_friend_st_lpn()){
	        		mesh_lpn_par.poll.FSN = !mesh_lpn_par.poll.FSN;	// change to poll the 2nd round.
					LOG_MSG_LIB(TL_LOG_FRIEND, 0, 0,"LPN flip FSN, then try again",0);
	        	}
	        	#endif
	            friend_cmd_send_poll();  // retry
	        }
	    }
	    else if(subsc_list_retry.retry_cnt && (!bear) && clock_time_exceed(subsc_list_retry.tick, timeout_ms)){
	        subsc_list_retry.tick = clock_time();   // also refresh when send_subsc
	        subsc_list_retry.retry_cnt--;
	        if(0 == subsc_list_retry.retry_cnt){
	            // whether restart establish procedure or not
	            mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);    // restart establish procedure
	        }else{
	            LOG_MSG_LIB(TL_LOG_FRIEND, 0, 0,"friend_subsc_repeat_***********************",0);
	            friend_subsc_repeat();
	        }
	    }
	}
    mesh_cmd_bear_t *p_bear = (mesh_cmd_bear_t *)bear;
    //mesh_cmd_nw_t *p_nw = &p_bear->nw;
    mesh_cmd_lt_ctl_unseg_t *p_lt_ctl_unseg = &p_bear->lt_ctl_unseg;
    u8 op = -1;
    if(bear){
	    op = p_lt_ctl_unseg->opcode;
    }
    
    if(0 == fri_ship_proc_lpn.status){
        if(bear){
            if(CMD_CTL_SUBS_LIST_CONF == op){
                mesh_ctl_fri_subsc_list_t *p_subsc = CONTAINER_OF(p_lt_ctl_unseg->data,mesh_ctl_fri_subsc_list_t,TransNo);
                if(p_subsc->TransNo == (subsc_list_retry.TransNo)){   //TransNo have increased
                	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)&p_subsc->TransNo, sizeof(p_subsc->TransNo),"rcv sub list confirm:",0);
                    if(SUBSC_ADD == subsc_list_retry.retry_type){
                        friend_subsc_list_add_adr((lpn_adr_list_t *)(&lpn_subsc_list.adr), (lpn_adr_list_t *)(&subsc_list_retry.adr), subsc_list_retry.subsc_cnt);
                    }
                    friend_subsc_stop();
                }
            }else if(CMD_CTL_UPDATE == op){
                mesh_ctl_fri_update_t *p_update = (mesh_ctl_fri_update_t *)(p_lt_ctl_unseg->data);
				LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)p_update, sizeof(mesh_ctl_fri_update_t),"NW_IVI %d, rcv friend update:",p_bear->nw.ivi);
                iv_update_key_refresh_rx_handle(&p_update->flag, p_update->IVIndex);

            }
        }else{
        }
    }else{
        switch(fri_ship_proc_lpn.status){   // Be true only during establish friend ship.
            case FRI_ST_REQUEST:
				if(is_mesh_adv_cmd_fifo_empty()){
	                fri_ship_proc_lpn.req_tick = clock_time();
					
	                friend_cmd_send_request();

	                // init par
	                mesh_friend_ship_clear_LPN();
	                mesh_lpn_par.LPNAdr = ele_adr_primary;
	                mesh_friend_ship_set_st_lpn(FRI_ST_OFFER);
				}
                break;
            case FRI_ST_OFFER:
                if(bear){
                    if(CMD_CTL_OFFER == p_lt_ctl_unseg->opcode){
                        if(0 != lpn_rx_offer_handle(bear)){
                            break;
                        }
                    }
                }else{
                    if(clock_time_exceed(fri_ship_proc_lpn.req_tick, FRI_ESTABLISH_PERIOD_MS*1000)){
                        if(mesh_lpn_par.FriAdr){
                            mesh_lpn_par.link_ok = 1;
                            mesh_friend_key_update_all_nk(0, 0);
                        }
                        mesh_friend_ship_set_st_lpn(FRI_ST_POLL);
                    }
                }
                break;
            case FRI_ST_POLL:
                if(is_friend_ship_link_ok_lpn()){
					if(is_mesh_adv_cmd_fifo_empty()){
	                    mesh_lpn_par.poll.FSN = 0;   // init
	                    // send poll
	                    fri_ship_proc_lpn.poll_retry = FRI_GET_UPDATE_RETRY_MAX + 1;					
	                    friend_cmd_send_poll();					
	                    t_rec_delay_and_win = mesh_lpn_par.req.RecDelay + mesh_lpn_par.offer.RecWin;
	                    mesh_friend_ship_set_st_lpn(FRI_ST_UPDATE);
					}
                }else{
                    lpn_no_offer_handle();
                }
                break;
            case FRI_ST_UPDATE:
                if(bear){   // current state is establishing friend ship
                    if(CMD_CTL_UPDATE == p_lt_ctl_unseg->opcode){
                        mesh_ctl_fri_update_t *p_update = (mesh_ctl_fri_update_t *)(p_lt_ctl_unseg->data);
						LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)p_update, sizeof(mesh_ctl_fri_update_t),"rcv friend update:",0);
                        memcpy(&mesh_lpn_par.update, p_update, sizeof(mesh_ctl_fri_update_t));
                        //friend ship establish done
                        mesh_lpn_par.req.PreAdr = mesh_lpn_par.FriAdr;
						mesh_cmd_sig_lowpower_heartbeat();
                        iv_update_key_refresh_rx_handle(&p_update->flag, p_update->IVIndex);
                        mesh_friend_ship_proc_init_lpn();

                        friend_ship_establish_ok_cb_lpn();
                    }
                }else{
                    if(clock_time_exceed(fri_ship_proc_lpn.poll_tick, t_rec_delay_and_win*1000)){
                        mesh_friend_ship_retry();
                    }
                }
                break;
            default:
                break;
        }
    }
}







u8 lpn_get_poll_retry_max()
{
    return FRI_POLL_RETRY_MAX;
}

#if LPN_DEBUG_PIN_EN
void lpn_debug_set_current_pin(u8 level)
{
    #define CURRENT_PIN_MESH      GPIO_PC4
    gpio_set_func(CURRENT_PIN_MESH, AS_GPIO);
    gpio_set_output_en(CURRENT_PIN_MESH, 1);
    gpio_write(CURRENT_PIN_MESH, level);
}

void lpn_debug_set_debug_pin(u8 level)
{
    //#define DEBUG_PIN_MESH      GPIO_PB7 // conflic with GPIO_VBAT_DETECT	// TBD
    gpio_set_func(DEBUG_PIN_MESH, AS_GPIO);
    gpio_set_output_en(DEBUG_PIN_MESH, 1);
    gpio_write(DEBUG_PIN_MESH, level);
}

void lpn_debug_set_irq_pin(u8 level)
{
    #define IRQ_PIN_MESH      GPIO_PD2
    gpio_set_func(IRQ_PIN_MESH, AS_GPIO);
    gpio_set_output_en(IRQ_PIN_MESH, 1);
    gpio_write(IRQ_PIN_MESH, level);
}

void lpn_debug_set_event_handle_pin(u8 level)
{
    #define EVENT_PIN_MESH      GPIO_PB5
    gpio_set_func(EVENT_PIN_MESH, AS_GPIO);
    gpio_set_output_en(EVENT_PIN_MESH, 1);
    gpio_write(EVENT_PIN_MESH, level);
}

void lpn_debug_alter_debug_pin(int reset)
{
    static u8 debug_pin_level = 0;
    if(reset){
        debug_pin_level = 0;
    }else{
        debug_pin_level = !debug_pin_level;
    }
    
    lpn_debug_set_debug_pin(debug_pin_level);
}
#endif

#if PTS_TEST_EN
enum{
    TS_LPN_BV04 = 4,
    TS_LPN_BV05 = 5,
    TS_LPN_BV08 = 8,
    TS_LPN_BV05_2 = 0x85,   // auto set
};

void pts_test_case_lpn()
{
    static u32 tick_ts_test;
	static u8 lpn_pts_test_cmd;
	if(lpn_pts_test_cmd){
		if((TS_LPN_BV04 == lpn_pts_test_cmd)){
			subsc_add_rmv_test(1);
		}else if(TS_LPN_BV05 == lpn_pts_test_cmd){
		    tick_ts_test = clock_time()|1;
			subsc_add_rmv_test(1);
		}else if(TS_LPN_BV05_2 == lpn_pts_test_cmd){
			subsc_add_rmv_test(0);
		}else if(TS_LPN_BV08 == lpn_pts_test_cmd){
			friend_cmd_send_fn(0, CMD_CTL_CLEAR);
		}
		lpn_pts_test_cmd = 0;
	}

	if(tick_ts_test && clock_time_exceed(tick_ts_test, 1500*1000)){
	    tick_ts_test = 0;
	    lpn_pts_test_cmd = TS_LPN_BV05_2;
	}
}
#endif

void mesh_feature_set_lpn(){
    #if FRI_SAMPLE_EN
	friend_ship_sample_message_test();
    #else
    mesh_ctl_fri_req_t *p_req = &mesh_lpn_par.req;
    p_req->Criteria.MinCacheSizeLog = LPN_MIN_CACHE_SIZE_LOG; // 3;
    p_req->Criteria.RecWinFac = FRI_REC_WIN_FAC;
    p_req->Criteria.RSSIFac = FRI_REC_RSSI_FAC;
    p_req->RecDelay = FRI_REC_DELAY_MS;
    p_req->PollTimeout = LPN_POLL_TIMEOUT_100MS;  // 0x057e40 = 10*3600 second
    p_req->PreAdr = 0;
    p_req->NumEle = g_ele_cnt;
    p_req->LPNCounter = 0 - 1;  // increase counter later
    #endif
}

void mesh_friend_ship_start_poll()
{
    if(is_in_mesh_friend_st_lpn()){
        friend_cmd_send_poll();
		mesh_lpn_poll_md_pending = 0;
		#if RETRY_POLL_AGAIN_FLIP_FSN_EN
		fri_ship_proc_lpn.poll_retry = PTS_TEST_EN ? 20 : (FRI_POLL_RETRY_MAX * 2);	// *2 means 2 rounds. change FSN to retry the 2nd round.
		#else
		fri_ship_proc_lpn.poll_retry = PTS_TEST_EN ? 20 : (FRI_POLL_RETRY_MAX + 1);
		#endif
    }
}

void mesh_friend_ship_stop_poll()
{
	blt_soft_timer_delete(&mesh_lpn_rcv_delay_wakeup);
	fri_ship_proc_lpn.poll_retry = 0;
}

void subsc_add_rmv_test(int add)
{
    if(!fri_ship_proc_lpn.status){
        if(is_friend_ship_link_ok_lpn()){
            u16 adr[] = {0xc001, 0xc002, 0xc003, 0xc004, 0xc005};
            if(add){
                friend_subsc_add(adr, ARRAY_SIZE(adr));
            }else{
                friend_subsc_rmv(adr, ARRAY_SIZE(adr));
            }
        }
    }
}

void lpn_subsc_list_update_by_sub_set_cmd(u16 op, u16 sub_adr)
{
    if((CFG_MODEL_SUB_ADD == op)||(CFG_MODEL_SUB_VIRTUAL_ADR_ADD == op)){
		mesh_lpn_subsc_pending_add(CMD_CTL_SUBS_LIST_ADD, sub_adr);
    }else if((CFG_MODEL_SUB_DEL == op)||(CFG_MODEL_SUB_VIRTUAL_ADR_DEL == op)){
		mesh_lpn_subsc_pending_add(CMD_CTL_SUBS_LIST_REMOVE, sub_adr);
    }else if(((CFG_MODEL_SUB_OVER_WRITE == op)||(CFG_MODEL_SUB_VIRTUAL_ADR_OVER_WRITE == op))
    	   ||(CFG_MODEL_SUB_DEL_ALL == op)){
    }
}

void friend_send_current_subsc_list()
{
#if MD_SERVER_EN
    u16 adr_list[SUB_LIST_MAX_IN_ONE_MSG];
    int j = 0;
    model_common_t * model_list[] = {
    	&model_sig_g_onoff_level.onoff_srv[0].com,
    	#if MD_MESH_OTA_EN
		&model_mesh_ota.blob_trans_srv.com,
		#endif
    };

    foreach_arr(k, model_list){
	    model_common_t *p_model_com = model_list[k]; 
	    #if MD_MESH_OTA_EN
	    if(p_model_com == &model_mesh_ota.blob_trans_srv.com){
		    if(is_fw_update_start_before()){
		    	// because it may re-establish friend ship during OTA.
		    }else{
		    	// not expected to report,because we don't want to report when establishing friendship after OTA.
		    	continue; // no need to report in this case.
		    }
	    }
	    #endif
	    
	    #if (VIRTUAL_ADDR_STAND_ALONE_SIZE_EN && (0 == VIRTUAL_ADDR_ENABLE))
	    foreach_arr(i, p_model_com->sub_buf.sub_addr)
	    #else
	    foreach_arr(i, p_model_com->sub_list)
	    #endif
	    {
	    	#if (VIRTUAL_ADDR_STAND_ALONE_SIZE_EN)
	        u16 sub_adr = p_model_com->sub_buf.sub_addr[i];
	    	#else
	        u16 sub_adr = p_model_com->sub_list[i];
	        #endif
	        if((sub_adr & 0x8000) && (!is_fixed_group(sub_adr))){
	            adr_list[j++] = sub_adr;
	        }
	        if(j >= SUB_LIST_MAX_IN_ONE_MSG){
	            break;
	        }
	    }
    }
    
    if(j){
        friend_subsc_add(adr_list, j);
    }
#endif
}

void lpn_node_io_init()
{
#if ((!IS_VC_PROJECT) && FEATURE_LOWPOWER_EN)
    lpn_debug_set_current_pin(1);
    #define WAKEUP_LEVEL_LPN    0       // level : 1 (high); 0 (low)
    foreach_arr(i,mesh_lpn_key_map)
    {
        gpio_set_wakeup (mesh_lpn_key_map[i], WAKEUP_LEVEL_LPN, 1);         // level : 1 (high); 0 (low)
        cpu_set_gpio_wakeup (mesh_lpn_key_map[i], WAKEUP_LEVEL_LPN, 1);     // level : 1 (high); 0 (low)
    }
    bls_pm_setWakeupSource(PM_WAKEUP_PAD);
#endif

#if LPN_VENDOR_SENSOR_EN
    i2c_io_init();
#endif
}

void mesh_lpn_sleep_prepare(u16 op)
{
	if(is_lpn_support_and_en && (BLS_LINK_STATE_CONN != get_blt_state())){
		if(CMD_ST_NORMAL_TX != op){  
			ENABLE_SUSPEND_MASK;
			rf_set_tx_rx_off();// disable tx rx in manual mode,must 	
			CLEAR_ALL_RFIRQ_STATUS;
			blt_soft_timer_delete(&mesh_lpn_poll_receive_timeout);
		}
		
		
		if(CMD_CTL_REQUEST == op){
			blt_soft_timer_update(&mesh_lpn_rcv_delay_wakeup, FRI_ESTABLISH_REC_DELAY_MS*1000);
		}
		else if((CMD_CTL_POLL == op) || (CMD_CTL_SUBS_LIST_REMOVE == op) || (CMD_CTL_SUBS_LIST_ADD == op)){
			blt_soft_timer_update(&mesh_lpn_rcv_delay_wakeup, mesh_lpn_par.req.RecDelay*1000);
		}
		else if(CMD_ST_NORMAL_TX == op){
			blt_soft_timer_update(&mesh_lpn_poll_md_wakeup, get_mesh_adv_interval());
		}
		else if(CMD_ST_POLL_MD == op){
			mesh_lpn_poll_md_pending = 1;
			u32 sleep_ms = FRI_POLL_DELAY_FOR_MD_MS;
			#if MD_MESH_OTA_EN
			if(is_blob_chunk_transfer_ready()){
				sleep_ms = 10;	// waiting for chunk message from FN retransmit completed
			}
			#endif
			blt_soft_timer_update(&mesh_lpn_poll_md_wakeup, sleep_ms * 1000);
			#if !WIN32
			blt_rxfifo.rptr = blt_rxfifo.wptr - 1;// clear buf, blt_rxfifo.rptr will ++ in lib
		    #endif
		}
		else if(CMD_CTL_UPDATE == op){
			#if !WIN32
			blt_rxfifo.rptr = blt_rxfifo.wptr - 1;// clear buf, blt_rxfifo.rptr will ++ in lib
		    #endif
		}
		else{//CMD_ST_SLEEP

		}
	}
	else if(CMD_ST_NORMAL_TX == op){
		blt_soft_timer_update(&mesh_lpn_poll_md_wakeup, get_mesh_adv_interval());
	}
}

void suspend_enter(u32 sleep_ms, int deep_retention_flag)
{
#if DEBUG_SUSPEND
    sleep_us(sleep_ms*1000);
#else
    int sleep_mode = 0; // SUSPEND_MODE default
    #if(__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278)||(MCU_CORE_TYPE == MCU_CORE_9518))
    sleep_mode = deep_retention_flag ? ((MCU_CORE_TYPE == MCU_CORE_9518) ? DEEPSLEEP_MODE_RET_SRAM_LOW64K : DEEPSLEEP_MODE_RET_SRAM_LOW32K) : SUSPEND_MODE;
    #endif
	
    cpu_sleep_wakeup(sleep_mode, PM_WAKEUP_TIMER, clock_time() + sleep_ms*CLOCK_SYS_CLOCK_1MS);
	lpn_wakeup_tick = clock_time();
#endif
}

u32 mesh_lpn_wakeup_key_io_get()
{
	#define WAKEUP_TRIGGER_LEVEL	0
	foreach_arr(i,mesh_lpn_key_map){
		if(WAKEUP_TRIGGER_LEVEL == gpio_read (mesh_lpn_key_map[i])){
			key_not_release = 1;	
			return mesh_lpn_key_map[i];
		}
	}

	return 0;
}

int is_lpn_key_cmd_tx_trigger()
{
    int trigger_flag = 0;
    u32 key_val = mesh_lpn_wakeup_key_io_get();
	
    if(MESH_LPN_CMD_KEY == key_val){     
        test_cmd_wakeup_lpn();       
        trigger_flag = 1;
    }
    
    return trigger_flag;
}

void lpn_key_factory_reset_check()
{
    // long press key
    u32 detkey = 0;
    int key_press = 0; // KEY_PRESSED_IDLE
    u32 long_tick = clock_time();

	if(del_node_tick){
		return;
	}
	
    do{
        detkey = mesh_lpn_wakeup_key_io_get();
        if(MESH_LPN_FACTORY_RESET_KEY == detkey){
            if(clock_time_exceed(long_tick, LONG_PRESS_TRIGGER_MS*1000)){
                key_press = KEY_PRESSED_LONG;
				cfg_cmd_reset_node(ele_adr_primary);
                detkey = 0;
                
                break;
            }else{
                key_press = KEY_PRESSED_SHORT;
            }
            
            if(get_blt_state() != BLS_LINK_STATE_CONN){
				#if(MCU_CORE_TYPE == MCU_CORE_8278)
				sleep_ms(1); //8278 can't call cpu_sleep_wakeup within 400us after retention wakeup
				#endif
                cpu_sleep_wakeup(0, PM_WAKEUP_TIMER, clock_time()+KEY_SCAN_INTERVAL_MS*1000*sys_tick_per_us);
            }
        }else{
            break;
        }
    }while(detkey);

    if(KEY_PRESSED_SHORT == key_press){
        if(LPN_MODE_GATT_OTA == lpn_mode){
            lpn_mode_set(LPN_MODE_NORMAL);
        }else{
            lpn_mode_set(LPN_MODE_GATT_OTA);
        }
    }
}

void lpn_proc_keyboard (u8 e, u8 *p, int n)
{
	if(e == BLT_EV_FLAG_GPIO_EARLY_WAKEUP){
	}
	else{
	}
	
	static u32 tick, scan_io_interval_us = 40000;
	if (!clock_time_exceed (tick, scan_io_interval_us))
	{
		return;
	}
	tick = clock_time();
	
	lpn_key_factory_reset_check(); // include mesh_lpn_wakeup_key_io_get_(); 
	
	#if (FRI_ESTABLISH_WIN_MS > 100)
	_attribute_no_retention_bss_ static u8 st_sw2_last;
	u8 st_sw2 = !gpio_read(MESH_LPN_CMD_KEY);
	if(!(st_sw2_last)&&st_sw2){
	    is_lpn_key_cmd_tx_trigger();
	}
	st_sw2_last = st_sw2;
	#endif
}

#if DEBUG_SUSPEND
void cpu_sleep_wakeup_core_debug()
{
    while(1){
		sleep_us(1000);
		mesh_lpn_wakeup_key = mesh_lpn_wakeup_key_io_get();
		if(mesh_lpn_wakeup_key){
			break;
		}
    }
}
#endif

int mesh_friend_offer_is_valid(mesh_ctl_fri_offer_t *p_offer)
{
    #if 0
    if(p_offer->RecWin > FRI_REC_WIN_MS){   // the larger receive window may cost more power when lost messages.
        // return 0;
    }
    
    if(p_offer->SubsListSize < SUB_LIST_MAX_LPN){
        // return 0;
    }
    #endif
    
    return ((p_offer->RecWin >= 1)&&(p_offer->CacheSize >= (1 << mesh_lpn_par.req.Criteria.MinCacheSizeLog))); // must
}

int mesh_is_better_offer(mesh_ctl_fri_offer_t *p_offer_new, mesh_ctl_fri_offer_t *p_offer)
{
    return (p_offer_new->RSSI > p_offer->RSSI);
}

int lpn_rx_offer_handle(u8 *bear)
{
    mesh_cmd_bear_t *p_bear = (mesh_cmd_bear_t *)bear;
    mesh_cmd_nw_t *p_nw = &p_bear->nw;
    mesh_cmd_lt_ctl_unseg_t *p_lt_ctl_unseg = &p_bear->lt_ctl_unseg;
    mesh_ctl_fri_offer_t *p_offer = (mesh_ctl_fri_offer_t *)(p_lt_ctl_unseg->data);
	LOG_MSG_LIB(TL_LOG_FRIEND,(u8 *)p_offer, sizeof(mesh_ctl_fri_offer_t),"rcv friend offer:",0);
    if(0 == mesh_friend_offer_is_valid(p_offer)){
        return -1;
    }
    
    //if(FN_RSSI_INVALID == p_offer->RSSI){
        adv_report_extend_t *p_extend = get_adv_report_extend(&p_bear->len);;
        p_offer->RSSI = p_extend->rssi;    // rssi: measure by LPN self,
    //}
    
    u8 better = (!mesh_lpn_par.FriAdr) || mesh_is_better_offer(p_offer, &mesh_lpn_par.offer);
    if(better){
        memcpy(&mesh_lpn_par.offer, p_offer, sizeof(mesh_ctl_fri_offer_t));
        mesh_lpn_par.FriAdr = p_nw->src;
    }

    return 0;
}

void mesh_friend_ship_retry()
{
    if(fri_ship_proc_lpn.req_retrys++ < FRI_REQ_RETRY_MAX){      
    }else{
        mesh_friend_ship_proc_init_lpn();        
    }
	mesh_lpn_sleep_prepare(CMD_ST_SLEEP);
	mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);
}

void lpn_no_offer_handle()  // only call after send request message during establish friend ship
{
    if(is_lpn_support_and_en){
        if(fri_ship_proc_lpn.req_retrys < FRI_REQ_RETRY_MAX){
        #if WIN32
            u16 rand_ms = 0;
        #else
            //u16 rand_ms = (rand() & 0x7F);
        #endif
            mesh_friend_ship_retry();   // should be before suspend,because of deep retention.
        }else{
            mesh_friend_ship_retry();
        }
    }else{
        // should not happen here
        if(clock_time_exceed(fri_ship_proc_lpn.req_tick, FRI_REQ_TIMEOUT_MS*1000)){
            mesh_friend_ship_retry();
        }
    }
}

void mesh_lpn_proc_suspend ()
{
	if((BLS_LINK_STATE_CONN != get_blt_state())){
		set_blt_busy(0); // triger pm in blt_sdk_main_loop
	}
	
    if(LPN_MODE_GATT_OTA == lpn_mode){
        // use BLE PM flow: BLE_REMOTE_PM_ENABLE
        if(get_blt_state() != BLS_LINK_STATE_CONN){
            if(clock_time_exceed(lpn_mode_tick, LPN_GATT_OTA_ADV_STATE_MAX_TIME_MS * 1000)){
                lpn_mode_set(LPN_MODE_NORMAL);
            }else{
                if(!is_led_busy()){
                    cfg_led_event (LED_EVENT_FLASH_1HZ_1T);
                }
            }
        }
        return ;
    }
    
	if(lpn_provision_ok){
	    
        if(get_blt_state() == BLS_LINK_STATE_CONN){
			ENABLE_SUSPEND_MASK;
            return ;
        }
		#if (!DEBUG_SUSPEND)
		if(clock_time_exceed(lpn_wakeup_tick, LPN_WORKING_TIMEOUT_MS*1000)){
			mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);// prevent abnormal working time.
		}
		#endif
	}else{
	    if(!is_provision_success()){
	        if(!is_provision_working() && !is_tlk_gatt_ota_busy()){   // not being provision
    	        if(clock_time_exceed(0, LPN_SCAN_PROVISION_START_TIMEOUT_MS*1000)){
    	            light_onoff_all(0);
                    cpu_sleep_wakeup(1, PM_WAKEUP_PAD, 0);
                    while(1);   // wait reboot
    	        }
	        }
	    }else{
    	    if((!lpn_provision_ok) && node_binding_tick && clock_time_exceed(node_binding_tick, LPN_START_REQUEST_AFTER_BIND_MS*1000)){
				lpn_provision_ok = 1;// provison and key bind finish
				gatt_adv_send_flag = GATT_LPN_EN;				
				mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);
				if(BLS_LINK_STATE_CONN == get_blt_state()){
					bls_ll_terminateConnection(0x13); // disconnet to establish friendship
				}
    	    }
	    }
	    return ;
	}

	return;
}

void mesh_main_loop_LPN()
{
}

u8 mesh_lpn_quick_tx_flag = 0;
int lpn_quick_tx(u8 is_quick_tx)
{
    u32 r = irq_disable();
	mesh_lpn_quick_tx_flag = is_quick_tx;
	int ret = mesh_send_adv2scan_mode(1);	
	mesh_lpn_quick_tx_flag = 0;
    lpn_debug_alter_debug_pin(0);
    irq_restore(r);
	return ret;

}

u32 get_lpn_poll_interval_ms()
{
	#if (MD_MESH_OTA_EN && MESH_OTA_PULL_MODE_EN)
	extern _align_4_ fw_update_srv_proc_t    fw_update_srv_proc;
	if(fw_update_srv_proc.pull_chunk_tick && (fw_update_srv_proc.busy || fw_update_srv_proc.blob_trans_busy)){
		return min(FRI_POLL_INTERVAL_MS_MESH_OTA, FRI_POLL_INTERVAL_MS);
	}
	#endif
	
	return FRI_POLL_INTERVAL_MS;
}

int mesh_lpn_adv_interval_update(u8 adv_tick_refresh)
{
	u16 interval = (LPN_MODE_GATT_OTA == lpn_mode) ? ADV_INTERVAL_MS:(is_lpn_support_and_en?(is_friend_ship_link_ok_lpn() ? get_lpn_poll_interval_ms() : FRI_REQ_TIMEOUT_MS):GET_ADV_INTERVAL_MS(ADV_INTERVAL_UNIT));
	int ret = bls_ll_setAdvParam_interval(interval, 0);
	if(adv_tick_refresh){
		extern u32 blt_advExpectTime;
		blt_advExpectTime = clock_time() + blta.adv_interval;
	}
	return ret;
}

void lpn_mode_set(int mode)
{
    if(LPN_MODE_GATT_OTA == mode){
        LOG_MSG_LIB(TL_LOG_FRIEND,0, 0,"SW1:enter GATT OTA",0);
        lpn_mode = LPN_MODE_GATT_OTA;
		gatt_adv_send_flag = 1;
		ENABLE_SUSPEND_MASK;
        mesh_friend_ship_proc_init_lpn();
        lpn_mode_tick = clock_time();
    }else if(LPN_MODE_NORMAL == mode){
        LOG_MSG_LIB(TL_LOG_FRIEND,0, 0,"SW1:exit GATT OTA",0);
        lpn_mode = LPN_MODE_NORMAL;
        mesh_friend_ship_set_st_lpn(FRI_ST_REQUEST);    // start to send friend request flow
        lpn_mode_tick = 0;
		gatt_adv_send_flag = 0;
        // cfg_led_event_stop();	// can't stop like this, becasue LED state needs to be recovered.
    }
	mesh_lpn_adv_interval_update(1);
}
#else
int lpn_quick_tx(u8 is_quick_tx){return 0;}
#endif 




