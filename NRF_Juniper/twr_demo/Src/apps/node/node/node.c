/**
 * @file    node.c
 * @brief    Decawave Application level
 *             collection of TWR bare-metal functions for a Node
 *
 * @author Decawave
 *
 * @attention Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
  ==============================================================================
  =                                                                            =
  =                                                                            =
  =                D E C A W A V E    C O N F I D E N T I A L                  =
  =                                                                            =
  =                                                                            =
  ==============================================================================

  This software contains Decawave confidential information and techniques,
  subject to licence and non-disclosure agreements.  No part of this software
  package may be revealed to any third-party without the express permission of
  Decawave Ltd.

 ==============================================================================
 */


/* Includes */
#include <math.h>

#include "node.h"
#include "util.h"
#include "nrf_drv_rtc.h"
#include "errno.h"
#include "deca_device_api.h"
#include "config.h"

#include "assert.h"
#include "deca_dbg.h"



// ----------------------------------------------------------------------------
// implementation-specific: critical section protection
#ifndef TWR_ENTER_CRITICAL
#define TWR_ENTER_CRITICAL    taskENTER_CRITICAL
#endif

#ifndef TWR_EXIT_CRITICAL
#define TWR_EXIT_CRITICAL     taskEXIT_CRITICAL
#endif

#define NODE_MALLOC  pvPortMalloc
#define NODE_FREE    vPortFree

// ----------------------------------------------------------------------------
#ifndef M_PI
#define M_PI    (3.141592654f)
#endif

#ifndef M_PI_2
#define M_PI_2  (1.570796327f)
#endif

#ifndef TWO_PI
#define TWO_PI  (2*M_PI)
#endif

#define FB       (499.2e6f)                    /* Basis frequency */
#define L_M_5    (SPEED_OF_LIGHT /FB /13.0f)   /* Lambda, m, CH5 */
#define L_M_9    (SPEED_OF_LIGHT /FB /16.0f)   /* Lambda, m, CH9 */
#define D_M_5    (0.022777110597040736f)       /* Distance between centers of antennas, ~(L_M/2), m, CH5 */
#define D_M_9    (0.017883104683497245f)       /* Distance between centers of antennas, ~(L_M/2), m, CH9 */


void
node_apply_pdoa_offset_deg(int16_t pdoa_offset_deg)
{
    if(pdoa_offset_deg < 0)
    {
        dwt_setpdoaoffset((int)(-1*(pdoa_offset_deg)*M_PI)*(1<<11)/180);
    }
    else if(pdoa_offset_deg > 0)
    {
        dwt_setpdoaoffset((int)((360 - (pdoa_offset_deg))*M_PI)*(1<<11)/180);
    }
    else
    {
        dwt_setpdoaoffset((int)((0)*M_PI)*(1<<11)/180);
    }
}


//-----------------------------------------------------------------------------
/* Note : for slot period less than 10 ms, more sophisticated TagHW shall be used.
 * STM32L1x & STM32F3x provide RTC with resolution of 61.035us, thus
 * current project can support precise timings.
 * See "default_config.h"
 **/

#define RX_RELAX_TIMEOUT_SY        (50)    /**< relaxed RX Timeout in sequential TWR process exchange */

//-----------------------------------------------------------------------------
// The psNodeInfo structure holds all Node's process parameters
static node_info_t *psNodeInfo = NULL;

//NRF
extern int rtcInitState;
extern uint32_t gRTC_SF_PERIOD;

extern void trilat_SF_cb(void);
static void rtcWakeUpTimerEventCallback_node(void);
static double pdoa2path_diff_ch5(float x);
static double pdoa2path_diff_ch9(float x);
static uint8_t s_node_eui[8] = {0};
static uint16_t s_node_addr16 = 0;

static void node_store_eui(uint32_t lotId, uint32_t partId)
{
    s_node_eui[0] = (uint8_t)(partId);
    s_node_eui[1] = (uint8_t)(partId >> 8);
    s_node_eui[2] = (uint8_t)(partId >> 16);
    s_node_eui[3] = (uint8_t)(partId >> 24);
    s_node_eui[4] = (uint8_t)(lotId);
    s_node_eui[5] = (uint8_t)(lotId >> 8);
    s_node_eui[6] = (uint8_t)(lotId >> 16);
    s_node_eui[7] = (uint8_t)(lotId >> 24);
}

//-----------------------------------------------------------------------------
// Implementation

/*
 * @brief     get pointer to the twrInfo structure
 * */
node_info_t * getNodeInfoPtr(void)
{
    return (psNodeInfo);
}


#if (DIAG_READ_SUPPORT==1)
/**
 * @brief     ISR level (need to be protected if called from APP level)
 *     read full diagnostic data form the received frame from the two DW1000s
 *
 * */
static int
read_full_diagnostics(rx_pckt_t *prxPckt,
                      uint32_t    status )
{
// TODO: rewrite for DW3000
    uint16_t     fpIndex = 0;
    uint16_t     fpIndex;
    diag_v5_t    *p = &prxPckt->diagnostics;

    p->header = DWT_DIAGNOSTIC_LOG_REV_5;

    memcpy(p->r0F, (uint8_t*) &status, 4);                      //copy 4bytes of status (saved on entry to ISR)
    dwt_readfromdevice(RX_FINFO_ID, 4, 5, (uint8_t*)(p+5) );    //read MSB from status and 4byte frame info
    dwt_readfromdevice(RX_FQUAL_ID, 0, 17,(uint8_t *)(p->r12)); //read 17 bytes of diagnostic data from 0x12,13,14

    memcpy((uint8_t*)p->r15, prxPckt->rxTimeStamp, TS_40B_SIZE);//copy TS
    dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 9, (uint8_t *)(p->r15 + 5)); //2FP, 2Diag, 5TSraw

    // Calculate the First Path Index ((LDE0 + LDE1 << 8) / 64)
    fpIndex = (*((uint8_t*)(p+32)) >> 6) + (*((uint8_t*)(p+33)) << 2);

    fpIndex = fpIndex*4 + 1;                 //get location in the accumulator

    //printf("%d FP index %02x %02x %i %i\n", offset, *((uint8_t*)(p+32)), *((uint8_t*)(p+33)), fpIndex, (fpIndex-1)>>2);
    //Read CIR for the First Path + 3 More samples (4*4 = 16)
    dwt_readaccdata(p->r25, 17, fpIndex-1); //read 1 extra as first will be dummy byte
    dwt_readfromdevice(LDE_IF_ID, LDE_PPINDX_OFFSET, 2, p->r2E);
    dwt_readfromdevice(DRX_CONF_ID, 0x28,            4, p->r27);
    dwt_readfromdevice(LDE_IF_ID, LDE_PPAMPL_OFFSET, 2, p->r2E2);

    return (int) fpIndex ;
}
#endif


//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------

/**
 *  @brief function to convert the ToF to range/distance to be output over UART/USB.
 *  if RBC(RANGE_BIAS_CORRECTION) is used to correct the range (then the output value will contain corrected range)
 *  otherwise the it will contain raw range
 *
 *  @param [in]
 *  *p  : pointer to run-time param_block_t;
 *  tofi: time of flight used to calculate the output distance;
 *  @param [out]
 *  *pdist_cm : pointer to output distance result. This also corrected with p->s.rngOffset_mm offset value.
 *              (float)(0XDEADBEEF) if error
 *
 */
error_e
tof2range(param_block_t *p,
          float         *r_m,
          int32_t       tofi)
{
    error_e      ret    = _NO_ERR;
    dwt_config_t *pCfg  = &p->dwt_config;

    *r_m  = (float)tofi* (SPEED_OF_LIGHT/499.2e6f/128.0f);

    if(*r_m >2000.0f)
    {
        ret =  _ERR_Range_Calculation;
    }
    else
    {
        *r_m = (*r_m - ((float)p->s.rngOffset_mm/1000.0f) );
    }

    UNUSED(pCfg);
    return ret;
}


void
pdoa2XY(result_t *pRes, uint8_t channel)
{
    static const float pdoa_interval_shift_ch5 = 15.0f;
    static const float pdoa_interval_shift_ch9 = -22.0f;
    float   pdoa_deg, r_m, x_m, y_m; /* PDOA (deg), range (m), x (m), y (m) */
    double   p_diff_m; /* Path difference between the ports (m). */
    float   pdoa_poll_deg, pdoa_final_deg; /* PDOAs poll and final (deg)  */
    float   pdoa_interval_shift, l_m, d_m;
    static float (*pdoa2path_diff)(float);

    if (channel == 5)
    {
        pdoa_interval_shift = pdoa_interval_shift_ch5;
        l_m = L_M_5;
        d_m = D_M_5;
        //pdoa2path_diff = &pdoa2path_diff_ch5;
    }
    else
    {
        pdoa_interval_shift = pdoa_interval_shift_ch9;
        l_m = L_M_9;
        d_m = D_M_9;
        //pdoa2path_diff = &pdoa2path_diff_ch9;
    }

    r_m = pRes->dist_cm / 100.0f;
    pdoa_final_deg = pRes->pdoa_raw_deg;
    pdoa_poll_deg  = pRes->pdoa_raw_degP;

    /* Shift the range of PDOAs out of the board */
    pdoa_final_deg = fmod(pdoa_final_deg - pdoa_interval_shift + 540.0f, 360.0f) + pdoa_interval_shift - 180.0f;
    pdoa_poll_deg  = fmod(pdoa_poll_deg - pdoa_interval_shift + 540.0f, 360.0f) + pdoa_interval_shift - 180.0f;

    /* If jumping detected do not average. */
    if ((abs(pdoa_final_deg) > 130.0f) && (pdoa_final_deg * pdoa_poll_deg< 0.0f))
    {
        pdoa_deg = pdoa_final_deg;
    }
    else
    {  /*Average PDOA value using poll and final PDOA*/
        pdoa_deg = (pdoa_poll_deg + pdoa_final_deg) / 2.0f;
    }

    /* Path difference (either LUT or just wave propagation theory). */
    if(channel == 5)
    {
        p_diff_m = (app.pConfig->s.phaseCorrEn) ? pdoa2path_diff_ch5(pdoa_deg) : (pdoa_deg / 360.0f * l_m);
    }
    else
    {
        p_diff_m = (app.pConfig->s.phaseCorrEn) ? pdoa2path_diff_ch9(pdoa_deg) : (pdoa_deg / 360.0f * l_m);
    }
    pRes->path_diff = p_diff_m*1e9;
    /* x and y from path difference and range */
    x_m = p_diff_m  / d_m * r_m;
    y_m = (abs(x_m) < r_m) ? sqrt(r_m*r_m - x_m*x_m) : 0.0f;
    /* m -> cm */
    pRes->x_cm = x_m * 100.0f;
    pRes->y_cm = y_m * 100.0f;
}



/*
 * @brief   This is a special function, which starts the SAR sampling.
 *          This shall be started in order to update temperature of the chip.
 *      Note: the reading of temperature will be available ~1mS after start of this function.
 * */
//static void
//start_tempvbat_sar(void)
//{
//   TODO: DW3000
//    uint8_t wr_buf[1];
//    // These writes should be single writes and in sequence
//    wr_buf[0] = 0x80; // Enable TLD Bias
//    dwt_writetodevice(RF_CONF_ID,0x11,1,wr_buf);
//
//    wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
//    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);
//
//    wr_buf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
//    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);    //
//
//    // Reading All SAR inputs
//    wr_buf[0] = 0x00;
//    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
//    wr_buf[0] = 0x01; // Set SAR enable
//    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
//}

/**
 * @brief  ISR level (need to be protected if called from APP level)
 *         Setup the receiver to receive the expected Final message, Final Tx Time + rx_timeout
 *
 *         This function is called from the (TX) interrupt handler.
 *         The Node can range to one tag at a time only.
 */
static void
resp_tx_cb_setup_receiver(node_info_t *pNodeInfo)
{
//------------------------------------------------------------------------------
//
// Description of the ranging protocol.
//
//           |< ------------------------------- tag_pollTxFinalTx_us --------------------------------->|
//TAGTX      |                                                                                         |
//   PollTxSFD   PollTxEnd                                                                    FinalTxSFD   FinalTxEnd
//_i---------|---i___________________________________________________________________________i---------|---i___
//           |
//           |   |<--- tag_replyDly_us --->|                                                 |
//TAGRX         |                   RespRxStart                                                 |
//_______________________________________..i---------------i___________________________________________________
//
//           |                             |                                                 |
//NODERX     |                             |                                                 |
//           |                             |           |<-(2)- dwt_setdelayedtrxtime for M ->|
//           |                             |           |                                     |
//    PollRxTs                             |           |                                      FinalRxSFD
//-----------|---i___________________________________________________________________________i---------|---i___
//           |
//           |<--[1]-- delayedTxTimeH_sy (tmp64) ----->|
//           |                                         |
//NODETX                                        respTxTs
//_________________________________________i-----------|---i___________________________________________________
//
//[1] is calculating in send_response()
//[2] is calculating in TX_IRQ callback
//------------------------------------------------------------------------------

    //[2] startup receiver.
    uint32_t    tmp;
    uint16_t    timeout;
#if 0
    uint64_t    anchorRespTxTime;
    TS2U64_MEMCPY(anchorRespTxTime, pNodeInfo->nodeRespTx_ts);
    tmp  = pNodeInfo->pSfConfig->tag_pollTxFinalTx_us;   //time between Final and Poll transmissions on the tag
    //need to enable receiver prior to start of Final's preamble transmission
    tmp -= pNodeInfo->pSfConfig->tag_replyDly_us;        //configured time when Node was transmitting Response (now)
    tmp -= pNodeInfo->msg_time .poll     .phrAndData_us; //length of data part of the poll
    tmp -= pNodeInfo->msg_time .response .preamble_us;   //length of response's preamble
    tmp -= pNodeInfo->msg_time .final    .preamble_us;   //length of final's preamble

    tmp = (uint32_t) (util_us_to_dev_time(tmp) >>8);

    tmp += (uint32_t)(anchorRespTxTime >>8);
    tmp &= 0xFFFFFFFE;                                              //This is the time when to enable the receiver

    timeout = pNodeInfo->msg_time.final.sy + RX_RELAX_TIMEOUT_SY;  //timeout for reception of Final msg

    dwt_setdelayedtrxtime(tmp);             //set delayed RX : waiting the Final
    dwt_setrxtimeout(timeout);
    dwt_rxenable(DWT_START_RX_DELAYED);     //start delayed RX : if late, then the RX will be enabled immediately.
#else
    //turn on the receiver to receive the Final message from the tag
    //use delayed receive based on the previous receive event (Poll)

    //set rx turn on delay
    tmp  = pNodeInfo->pSfConfig->tag_pollTxFinalTx_us;   //time between Final and Poll transmissions on the tag
    tmp -= pNodeInfo->msg_time .final    .preamble_us;   //length of final's preamble (length of packet until RMARKER)
    tmp = (uint32_t) (util_us_to_dev_time(tmp) >>8);     //convert to device timeunits
    dwt_setdelayedtrxtime(tmp);                          //this time is relative to previous receive time (of the Poll) as we are using  DWT_START_RX_DLY_RS below

    //set timeout
    timeout = pNodeInfo->msg_time.final.sy + RX_RELAX_TIMEOUT_SY;  //timeout for reception of Final msg
    dwt_setrxtimeout(timeout);

    dwt_rxenable(DWT_START_RX_DLY_RS);    //delayed receive based on the previous receive timestamp (Poll's)
#endif
//    start_tempvbat_sar();               //start sampling of a temperature on the Master chip: on reception of Final read the value.

}


//-----------------------------------------------------------------------------
//    DW3000 callbacks section :
//    if RTOS, the preemption priority of the dwt_isr() shall be such, that
//    allows signal to the thread.

/* @brief   ISR level
 *          Real-time TWR application Tx callback
 *          to be called from dwt_isr()
 * */
void twr_tx_node_cb(const dwt_cb_data_t *txd)
{
    node_info_t     *pNodeInfo = getNodeInfoPtr();

    if(!pNodeInfo)
    {
        return;
    }

    uint32_t tmp = nrf_drv_rtc_counter_get(&rtc);

    // Store the Tx Time Stamp of the transmitted packet
    switch(pNodeInfo->txState)
    {
    case Twr_Tx_Ranging_Config_Sent :                //responder (node)
        pNodeInfo->rangeInitRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pNodeInfo->rangeInitTx_ts);
        break;

    case Twr_Tx_Resp_Sent :                        //responder (node)
        pNodeInfo->respRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pNodeInfo->nodeRespTx_ts);

        resp_tx_cb_setup_receiver(pNodeInfo);    //node additional algorithm for receivers
        break;

    default :
        break;
    }
}


/* @brief   ISR level
 *          TWR application Rx callback
 *          to be called from dwt_isr() as an Rx call-back
 * */
void twr_rx_node_cb(const dwt_cb_data_t *rxd)
{
    node_info_t  *pNodeInfo = getNodeInfoPtr();

    if(!pNodeInfo)
    {
        return;
    }

    const int  size = sizeof(pNodeInfo->rxPcktBuf.buf) / sizeof(pNodeInfo->rxPcktBuf.buf[0]);

    int head = pNodeInfo->rxPcktBuf.head;
    int tail = pNodeInfo->rxPcktBuf.tail;

    if(CIRC_SPACE(head,tail,size) <= 0)
    {
        return;    //no space in the fast intermediate circular buffer
    }

    rx_pckt_t *p = &pNodeInfo->rxPcktBuf.buf[head];

    p->rtcTimeStamp = nrf_drv_rtc_counter_get(&rtc);    // MCU RTC timestamp 


    //TODO: DW3000  - should we use STS timestamp when using STS, and check for quality

    dwt_readrxtimestamp(p->timeStamp);      //Raw Rx TimeStamp
    p->status = rxd->status;

    if((pNodeInfo->dw3000ChipType == AOA) && (app.pConfig->dwt_config.pdoaMode != DWT_PDOA_M0))
    {
        p->PDOA.pdoa = dwt_readpdoa();           //Phase difference, signed in [1:-11]
    }
    else
    {
        p->PDOA.pdoa = 0;
        memset(&p->PDOA.mDiag, 0x55, sizeof(p->PDOA.mDiag));
    }
    p->rxDataLen = MIN(rxd->datalength, sizeof(p->msg));

    dwt_readrxdata((uint8_t *)&p->msg, p->rxDataLen, 0); //Raw message

    if(app.rxTask.Handle)         // RTOS : rxTask can be not started yet
    {
        head = (head + 1) & (size-1);
        pNodeInfo->rxPcktBuf.head = head;    // ISR level : do not need to protect

        //Sends the Signal to the application level via OS kernel.
        //This will add a small delay of few us, but
        //this method make sense from a program structure point of view.
        if(osSignalSet(app.rxTask.Handle, app.rxTask.Signal) != osOK)
        {
            error_handler(1, _ERR_Signal_Bad);
        }
    }
}


void twr_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
}

void twr_rx_error_cb(const dwt_cb_data_t *rxd)
{
    twr_rx_timeout_cb(rxd);
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// The most real-time section of TWR algorithm

/* return us - correction of time for a given Tag wrt uTimeStamp
 * */
static int32_t
calc_slot_correction(node_info_t    *pNode,
                     uint16_t        slot,
                     uint32_t       uTimeStamp)

{
    int    tmp;

    tmp = uTimeStamp - pNode->gRtcSFrameZeroCnt;

    if (tmp < 0)
    {
        tmp += RTC_WKUP_CNT_OVFLW;    // RTC Timer overflow - 24 bit counter
    }

    tmp = (int)((tmp * WKUP_RESOLUTION_NS) - (1e6f * slot * pNode->pSfConfig->slotPeriod));
    tmp /= 1e3f;


    return (tmp); //tagSleepCorrection_us
}


/**
 * @brief this function constructs the Ranging Config message,
 *        not including mac header
 *
 */
static void
prepare_ranging_config_msg(rng_cfg_t        *pRcfg,
                           tag_addr_slot_t  *tag,
                           node_info_t       *p,
                           uint32_t           uTimeStamp)
{

    pRcfg->fCode = Twr_Fcode_Rng_Config;          //function code (specifies if message is a rangeInit, poll, response, etc)

    pRcfg->tagAddr[0] = tag->addrShort[0];        //tag short address to be used in TWR
    pRcfg->tagAddr[1] = tag->addrShort[1];

    pRcfg->version    = RC_VERSION_PDOA;

    {   //slot period correction for the Tag wrt to
        int32_t tagSleepCorrection_us = calc_slot_correction(p, tag->slot, uTimeStamp);
        U32TOAR_MEMCPY(pRcfg->slotCorr_us , tagSleepCorrection_us);
    }

    pRcfg->sframePeriod_ms[0] = p->pSfConfig->sfPeriod_ms       &0xff;
    pRcfg->sframePeriod_ms[1] = p->pSfConfig->sfPeriod_ms>>8    &0xff;

    pRcfg->pollTxToFinalTx_us[0] = p->pSfConfig->tag_pollTxFinalTx_us       &0xff;
    pRcfg->pollTxToFinalTx_us[1] = p->pSfConfig->tag_pollTxFinalTx_us>>8    &0xff;

    pRcfg->delayRx_us[0] = p->pSfConfig->tag_replyDly_us    &0xff;
    pRcfg->delayRx_us[1] = p->pSfConfig->tag_replyDly_us>>8 &0xff;

    pRcfg->pollMultFast[0] = tag->multFast      &0xff;  //tag config : multiplier: i.e poll every 1 periods
    pRcfg->pollMultFast[1] = tag->multFast>>8   &0xff;  //if moving

    pRcfg->pollMultSlow[0] = tag->multSlow      &0xff;  //tag config : multiplier: i.e. poll every 10 period
    pRcfg->pollMultSlow[1] = tag->multSlow>>8   &0xff;  //if stationary

    pRcfg->mode[0] = tag->mode     &0xff;      //tag config : i.e. use imu to identify stationary : bit 0;
    pRcfg->mode[1] = tag->mode>>8  &0xff;      //
}


/**
 * @brief   This function constructs the data part of Response Msg.
 *          It is called after node receives a Poll from a tag.
 *          Mac header for the message should be created separately.
 */
static void
prepare_response_msg(resp_tag_t         *pResp,
                     tag_addr_slot_t    *tag,
                     node_info_t         *p,
                     uint32_t             uTimeStamp)
{
    int32_t    tmp;
    pResp->fCode = Twr_Fcode_Resp_Ext;

    {//slot period correction for the Tag
        int32_t tagSleepCorrection_us = calc_slot_correction(p, tag->slot, uTimeStamp);
        U32TOAR_MEMCPY(pResp->slotCorr_us , tagSleepCorrection_us);
    }

    pResp->rNum = p->result[tag->slot].rangeNum;

    /* Send back to the tag the previous X, Y & clockOffset */
    tmp = (int32_t)(p->result[tag->slot].x_cm);
    pResp->x_cm[0] = (uint8_t)(tmp &0xFF);
    pResp->x_cm[1] = (uint8_t)(tmp>>8 &0xFF);

    tmp = (int32_t)(p->result[tag->slot].y_cm);
    pResp->y_cm[0] = (uint8_t)(tmp &0xFF);
    pResp->y_cm[1] = (uint8_t)(tmp>>8 &0xFF);

    tmp = (int32_t)(p->result[tag->slot].clockOffset_pphm);
    pResp->clkOffset_pphm[0] = (uint8_t)(tmp &0xFF);
    pResp->clkOffset_pphm[1] = (uint8_t)(tmp>>8 &0xFF);
}


/* @brief    APP level
 *             part of Real-time TWR algorithm implementation (Responder)
 *
 *             if called from ISR level, then revise/remove
 *             TWR_ENTER_CRITICAL()
 *             TWR_EXIT_CRITICAL()
 *
 *            Note:
 *            Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return     _NO_ERR for no errors / error_e otherwise
 * */
static error_e
node_send_ranging_config(rx_pckt_t  *pRxPckt,
                         node_info_t *p)

{
    error_e         ret;
    tx_pckt_t       TxPckt;     /**< allocate TxPckt */
    tag_addr_slot_t *tag = pRxPckt->tag;

    /* tmp, tmp64 used to calculate the time when to send the delayed Ranging Config (i.e. R-Marker of RangingConfig),
     * using Tag's configuration of when Tag is activating its receiver
     * */
    uint32_t    tmp;
    uint64_t    tmp64;

    /* setup the rest of Ranging Config data */
    rng_cfg_t *pRcfg;

    if(tag->reqUpdatePending == 0)
    {
        /* When the Node receives a blink from a Tag,
         * setup the Ranging Config MAC frame header with Long-Short addressing mode
         * */
        rng_cfg_msg_t   *pTxMsg = &TxPckt.msg.rngCfgMsg;

        /* Construct TX Ranging Config UWB message (LS) */
        TxPckt.psduLen      = sizeof(rng_cfg_msg_t);

        /* See IEEE frame header description */
        pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
        pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_LS;       /**<long address */
        pTxMsg->mac.panID[0]        = p->panID     &0xff;
        pTxMsg->mac.panID[1]        = p->panID>>8  &0xff;
        memcpy(pTxMsg->mac.destAddr, &tag->addr64, sizeof(pTxMsg->mac.destAddr)); /**< tag's address */
        pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];     /**< node short address to be used by the tag in TWR */
        pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
        pTxMsg->mac.seqNum          = p->seqNum;

        /* rest of Ranging Config */
        pRcfg               = &pTxMsg->rngCfg;

        //rcDelay_us is a SYSTEM configuration value of the delay
        //after completion of a Tag's Blink Tx, when the Tag will turn on its receiver
        //and will wait for the Ranging Config Response from the Node.
        //From Node's view this is a delay between end of reception of Blink's data
        //and start of transmission of preamble of Ranging Config.
        tmp  = app.pConfig->s.rcDelay_us;

        /* Adjust the transmission time with respect to the last System TimeStamp, i.e. Timestamp of Blink */
        tmp += p->msg_time.blink.phrAndData_us;          //pre-calculated length of Blink packet (data)
        tmp += p->msg_time.ranging_config.preamble_us;   //pre-calculated length of preamble length of Ranging Config
        tmp += p->msg_time.ranging_config.sts_us;        //pre-calculated length of sts of Ranging Config
        TS2U64_MEMCPY(tmp64, pRxPckt->timeStamp);        //Blink's timestamp
        tmp64 += util_us_to_dev_time(tmp);
        tmp64 &= MASK_TXDTS;
    }
    else
    {
        tag->reqUpdatePending = 0;

        /* When the Node receives a poll and need to update the Tag, it sends Ranging Config instead of response
         * setup the Ranging Config MAC frame header with Short-Short addressing mode
         * */
        rng_cfg_upd_msg_t   *pTxMsg = &TxPckt.msg.rngCfgUpdMsg;

        /* Construct TX Ranging Config Update UWB message */
        TxPckt.psduLen  = sizeof(rng_cfg_upd_msg_t);

        /* See IEEE frame header description */
        pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
        pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
        pTxMsg->mac.panID[0]        = p->panID & 0xff;
        pTxMsg->mac.panID[1]        = (p->panID >> 8) & 0xff;
        pTxMsg->mac.destAddr[0]     = tag->addrShort[0];
        pTxMsg->mac.destAddr[1]     = tag->addrShort[1];
        pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];
        pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
        pTxMsg->mac.seqNum          = p->seqNum;

        /* rest of Ranging Config */
        pRcfg           = &pTxMsg->rngCfg;

        //tag_replyDly_us is a SYSTEM configuration value of the delay
        //after completion of a Tag's Poll Tx, when the Tag will turn on its receiver
        //and will wait for the Response from the Node.
        //From Node's view this is a delay between end of reception of Poll's data
        //and start of transmission of preamble of Reply (Ranging Config).
        tmp  = app.pConfig->s.sfConfig.tag_replyDly_us;

        /* Adjust the transmission time with respect to the last System TimeStamp, i.e. Timestamp of Poll */
        tmp += p->msg_time.poll.phrAndData_us;          //pre-calculated length of Poll packet (data)
        tmp += p->msg_time.ranging_config.preamble_us;  //pre-calculated length of preamble length of Ranging Config
        tmp += p->msg_time.ranging_config.sts_us;       //pre-calculated length of sts of Ranging Config
        TS2U64_MEMCPY(tmp64, pRxPckt->timeStamp);       //Poll's timestamp
        tmp64 += util_us_to_dev_time(tmp);
        tmp64 &= MASK_TXDTS;
    }

    /* write the rest of Ranging Config */
    prepare_ranging_config_msg(pRcfg, tag, p,  pRxPckt->rtcTimeStamp);

    TxPckt.txFlag               = ( DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    TxPckt.delayedTxTimeH_sy    = tmp64 >>8;    //at this time the Node will transmit the Ranging Config TimeStamp (R-Marker)
    TxPckt.delayedRxTime_sy     = 0;            //switch on Rx after RangingConfig transmission immediately
    TxPckt.delayedRxTimeout_sy  = 0;

    p->txState                  = Twr_Tx_Ranging_Config_Sent; //indicate to TX ISR that the RangeInit has been sent
    p->seqNum++;

    TWR_ENTER_CRITICAL();

    ret = tx_start(&TxPckt);

    TWR_EXIT_CRITICAL();

    if( ret != _NO_ERR)
    {
        p->lateTxCount++;
    }

    return (ret);
}


/* @brief   APP level
 *          part of Real-time TWR algorithm implementation (Responder)
 *
 *          if called from ISR level, then remove
 *          TWR_ENTER_CRITICAL() and TWR_EXIT_CRITICAL() around tx_start()
 *
 *          Note:
 *          Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return  _NO_ERR for no errors / error_e otherwise
 * */
static error_e
node_send_response(rx_pckt_t *pRxPckt, node_info_t *p)
{
    error_e     ret;
    uint32_t    tmp;
    uint64_t    tmp64;

    /* Construct the response tx packet to the tag */
    tx_pckt_t       TxPckt;
    resp_pdoa_msg_t  *pTxMsg = &TxPckt.msg.respMsg;

    pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
    pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
    pTxMsg->mac.panID[0]        = p->panID & 0xff;
    pTxMsg->mac.panID[1]        = (p->panID >> 8) & 0xff;
    pTxMsg->mac.destAddr[0]     = pRxPckt->tag->addrShort[0];
    pTxMsg->mac.destAddr[1]     = pRxPckt->tag->addrShort[1];
    pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];
    pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
    pTxMsg->mac.seqNum          = p->seqNum;

    prepare_response_msg(&pTxMsg->resp, pRxPckt->tag, p, pRxPckt->rtcTimeStamp);

//------------------------------------------------------------------------------
//
// Description of the ranging protocol.
//
//           |< ------------------------------- tag_pollTxFinalTx_us --------------------------------->|
//TAGTX      |                                                                                         |
//   PollTxSFD   PollTxEnd                                                                   FinalTxSFD    FinalTxEnd
//_i---------|---i___________________________________________________________________________i---------|---i___
//           |
//           |   |<--- tag_replyDly_us --->|                                                 |
//TAGRX         |                   RespRxStart                                                 |
//_______________________________________..i---------------i___________________________________________________
//
//           |                             |                                                 |
//NODERX     |                             |                                                 |
//CHIP MASTER|                             |           |<-(2)- dwt_setdelayedtrxtime for M ->|
//           |                             |           |                                     |
//    PollRxTs                             |           |                                      FinalRxSFD
//-----------|---i___________________________________________________________________________i---------|---i___
//           |
//           |<--[1]-- delayedTxTimeH_sy (tmp64) ----->|
//           |                                         |
//NODETX                                        respTxTs
//_________________________________________i-----------|---i___________________________________________________
//
//           |                                         |
//           |                                         |
//           |                                         |<-[2]- dwt_setdelayedtrxtime for S ->|
//    PollRxTs                                  RespTxTs                                     |
//           |                                         |
//___________________________________________________________________________________________i---------|---i___
//
//[1] is calculating in node_send_response()
//[2] is calculating in TX_IRQ callback: twr_tx_node_cb()
//------------------------------------------------------------------------------

    /* [1] configure TX devtime when R-marker of Response will be transmitted */

    // DW3000 will adjust the transmission of SFD of the response packet exactly
    // to PollRx + tag_replyDly + length of PLEN of msg

    tmp  = p->pSfConfig->tag_replyDly_us;    //tag_replyDly_us is a SYSTEM configuration value of the delay
                                                    //after completion of a Tag PollTx, when the Tag will turn on its receiver
                                                    //and will wait for the Response from the Node.

    tmp += p->msg_time.poll.phrAndData_us;   //pre-calculated length of Poll packet
    tmp += p->msg_time.response.preamble_us; //pre-calculated length of Response packet
    tmp += p->msg_time.response.sts_us;      //pre-calculated length of Response packet


    /* Adjust the Response transmission time with respect to the last system TimeStamp */
    TS2U64_MEMCPY(tmp64, p->nodePollRx_ts);
    tmp64 += util_us_to_dev_time(tmp);
    tmp64 &= MASK_TXDTS;

    TxPckt.delayedTxTimeH_sy    = tmp64 >>8;    //at this time the Node will transmit the Response's TimeStamp
                                                //in the DWT_START_TX_DELAYED mode.

    TxPckt.psduLen              = sizeof(resp_pdoa_msg_t);
    TxPckt.txFlag               = ( DWT_START_TX_DELAYED ); //DW3000 receiver will be set in the dwt_isr() : twr_tx_node_cb()
    TxPckt.delayedRxTime_sy     = 0;
    TxPckt.delayedRxTimeout_sy  = 0;

    p->seqNum++;
    p->txState                  = Twr_Tx_Resp_Sent;         //indicate to TX_IRQ that the Response was sent

    TWR_ENTER_CRITICAL();

    ret = tx_start(&TxPckt);

    TWR_EXIT_CRITICAL();

    //after a good tx_start(), DW_IC will be configured for reception in the TX_IRQ callback [2].
    //if the tx_start() failed delayed transmit, the DW_IC receiver will be re-enabled in the control application.

    return (ret);
}

/* @brief   APP level
 *          part of Real-time TWR algorithm implementation (Responder)
 *
 *          if called from ISR level, then remove
 *          TWR_ENTER_CRITICAL() and TWR_EXIT_CRITICAL()
 *
 *          Note:
 *          Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return
 * */
static void node_read_final_diagnostics(rx_pckt_t *pRxPckt)
{
    /* read diagnostics from the chips */
    TWR_ENTER_CRITICAL();

    { //Diagnostics data logging for the final message
        if(app.pConfig->s.diagEn == 1)
        {
            dwt_readdiagnostics(&pRxPckt->diag_dw3000);
        }

        if(app.pConfig->s.accEn == 1)
        {
            //TODO: review
            //memset(pRxPckt->acc, 0, sizeof(pRxPckt->acc));
            //dwt_readaccdata(&pRxPckt->acc[0][0], sizeof(pRxPckt->acc[0]), ACC_OFFSET);
        }
    }

    TWR_EXIT_CRITICAL();
}


/* @brief    APP level
 *             Real-time TWR algorithm implementation (Responder)
 *
 *             prefer to be called from application UWB Rx thread, but
 *             can be called bare-metal from ISR, i.e. twr_rx_node_cb() directly.
 *             if called from ISR level, then revise/remove
 *             TWR_ENTER_CRITICAL()
 *             TWR_EXIT_CRITICAL()
 *
 *            Note:
 *            Shall be called with the guarantee that the DWT_IRQ will not happen
 *
 * @return     returning the result of low-level parse as error_e code:
 *             _NO_ERR             : TX sent
 *             _Err_Not_Twr_Frame
 *             _Err_DelayedTX_Late
 *             _Err_Not_Twr_Frame
 *             _Err_Unknown_Tag
 *             _No_Err_New_Tag
 *             _No_Err_Final
 *
 * */
error_e twr_responder_algorithm_rx(rx_pckt_t *pRxPckt, node_info_t *pNodeInfo)
{
    fcode_e     fcode = Twr_Fcode_Not_Defined;
    error_e     ret   = _ERR_Not_Twr_Frame;
    std_msg_t   *pMsg = &pRxPckt->msg.stdMsg;

    if( (pMsg->mac.frameCtrl[0] == Head_Msg_BLINK) && (pRxPckt->rxDataLen == sizeof(blink_msg_t)) )
    {
        fcode = Twr_Fcode_Blink;
        pNodeInfo->seqNum = pMsg->mac.frameCtrl[1]; //for Blink message the second byte is seq. number
    }
    else if((pMsg->mac.frameCtrl[0] == Head_Msg_STD) || (pMsg->mac.frameCtrl[0] == Head_Msg_STD_AR))
    {
        /* Apart of Blinks only 16-bit dest/source addresses (SS) MAC headers supported in current Node application */
        switch (pMsg->mac.frameCtrl[1] & Frame_Ctrl_MASK)
        {
        case Frame_Ctrl_SS:
            if((pRxPckt->rxDataLen == sizeof(poll_msg_t)) ||        /* Poll */
               (pRxPckt->rxDataLen == sizeof(final_msg_accel_t)))   /* Final extended */
            {
                fcode = ((std_msg_ss_t*)pMsg)->messageData[0] ;
            }
            break;
        default:
            fcode = Twr_Fcode_Not_Defined ;
            break;
        }
    }
    else
    {
        fcode = Twr_Fcode_Not_Defined ;
    }


    /* received packet with "fcode" functional code */
    switch (fcode)
    {
        case Twr_Fcode_Blink :
        {
            /* Responder (Node) received Blink message from Tag in discovery process.
             * 1. if Tag addr64 is unknown : report to upper application
             * 2. if Tag addr64 is in the known Tag list, setup Timing parameters and send back to the Tag in the
             *       Ranging Config message;
             * */
            uint64_t        addr64;

            memcpy(&addr64, ((blink_msg_t*)pMsg)->tagID, sizeof(addr64));           //valid only for low endian

            pRxPckt->tag = get_tag64_from_knownTagList(addr64);

            pNodeInfo->pDestTag            = NULL;   /* New Blink: interrupt any range exchange if it was in progress */
            memset(pNodeInfo->nodePollRx_ts, 0, sizeof(pNodeInfo->nodePollRx_ts));    /* received time of poll message */
            memset(pNodeInfo->nodeRespTx_ts, 0, sizeof(pNodeInfo->nodePollRx_ts));    /* anchor's response tx time */

            if (pRxPckt->tag)
            {
                pRxPckt->tag->reqUpdatePending = 0;
                ret = node_send_ranging_config(pRxPckt, pNodeInfo);
            }
            else
            {
                pNodeInfo->newTag_addr64 = addr64;
                ret = _NO_Err_New_Tag;              /* report to the upper application discovery of a new Tag */
            }
            break;
        }

        case  Twr_Fcode_Tag_Poll :
        {
            /* Responder (Node) received the Poll from a Tag.
             * 1. if Tag addr16 is in the known Tag list, perform ranging sequence with this tag.
             * 2. otherwise ignore
             * */
            uint16_t    addr16;

            addr16      = AR2U16(((poll_msg_t*)pMsg)->mac.sourceAddr);

            pRxPckt->tag = get_tag16_from_knownTagList( addr16 );

            if (pRxPckt->tag)
            {
                pNodeInfo->pDestTag = pRxPckt->tag;                         /* current tag we are ranging to*/
                TS2TS_MEMCPY(pNodeInfo->nodePollRx_ts, pRxPckt->timeStamp); /* node's received time of Poll message */
                pNodeInfo->pollRtcTimeStamp = pRxPckt->rtcTimeStamp;


                memcpy(&pNodeInfo->pollPDOA, &pRxPckt->PDOA, sizeof(pNodeInfo->pollPDOA));/* node's received PDOA result */
                memset(&pNodeInfo->finalPDOA, 0, sizeof(pNodeInfo->finalPDOA));

                if(pRxPckt->tag->reqUpdatePending)
                {
                    ret = node_send_ranging_config(pRxPckt, pNodeInfo);  /* send any updates to tag (e.g. change TWR rate) */
                }
                else
                {
                    ret = node_send_response(pRxPckt, pNodeInfo);  /* send a response message - continue with TWR */
                }

                if(ret == _NO_ERR)
                {
                    pNodeInfo->result[pRxPckt->tag->slot].rangeNum           = pRxPckt->msg.pollMsg.poll.rNum;

                    /* Below results will be calculated in the Node on reception of Final from the Tag.
                     * If the calculations was not successful, Node will not report wrong values to usb/uart,
                     * but will send them back to the Tag (in the next Response message) */
                    pNodeInfo->result[pRxPckt->tag->slot].pdoa_raw_deg       = (float)0xDEAD;
                    pNodeInfo->result[pRxPckt->tag->slot].clockOffset_pphm   = (float)0xDEAD;
                    pNodeInfo->result[pRxPckt->tag->slot].dist_cm            = (float)0xDEAD;
                    pNodeInfo->result[pRxPckt->tag->slot].x_cm               = (float)0xDEAD;
                    pNodeInfo->result[pRxPckt->tag->slot].y_cm               = (float)0xDEAD;
                }
                else
                {
                    pNodeInfo->pDestTag            = NULL;    /* no range exchange is in progress */
                    memset(pNodeInfo->nodePollRx_ts, 0, sizeof(pNodeInfo->nodePollRx_ts));    /* clear received time of poll message */
                    memset(pNodeInfo->nodeRespTx_ts, 0, sizeof(pNodeInfo->nodeRespTx_ts));    /* clear node's response tx time */

                    pNodeInfo->lateTxCount++;
                }
            }
            else
            {
                ret = _ERR_Unknown_Tag;
            }
            break;
        }

        case Twr_Fcode_Tag_Accel_Final :
        {
            /* Responder (Node) received the Final from Tag,
             * 1. if Tag is that one the node is currently ranging to :
             *       read registers necessary for ranging & phase difference and report this to upper application
             *       to calculate the result
             * 2. otherwise ignore
             * */
            uint16_t        addr16;
            ret = _ERR_Unknown_Tag;

            addr16 = AR2U16( ((final_msg_accel_t*)pMsg)->mac.sourceAddr );

            pRxPckt->tag = get_tag16_from_knownTagList( addr16 );

            if (pRxPckt->tag && (pRxPckt->tag == pNodeInfo->pDestTag))
            {
                memcpy(&pNodeInfo->finalPDOA, &pRxPckt->PDOA, sizeof(pNodeInfo->finalPDOA));

                node_read_final_diagnostics(pRxPckt);
                ret = _NO_Err_Final;
            }

            break;
        }

        default:
            /* Responder (Node) received unknown data : discard previous measurements and restart the reception
             * */
            pNodeInfo->pDestTag = NULL;

            ret = _ERR_Not_Twr_Frame;
            break;
    }

    return (ret);
}

//-----------------------------------------------------------------------------

void node_get_eui(uint8_t *buf)
{
    if (!buf)
    {
        return;
    }

    memcpy(buf, s_node_eui, 8);
}

uint16_t node_get_addr16(void)
{
    return s_node_addr16;
}


//-----------------------------------------------------------------------------
/*
 * @brief   setup RTC Wakeup timer
 *          period_ms is awaiting time in ms
 * */
void node_configure_rtc_wakeup(uint32_t     period_ms)
{
    ret_code_t           err_code;
    uint32_t             old_cc, new_cc;
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;

    config.prescaler = RTC_WKUP_PRESCALER;   // WKUP_RESOLUTION_US counter period

    gRTC_SF_PERIOD = (period_ms * 1e6) / WKUP_RESOLUTION_NS;

    // the timer was intialized in the rtc_init()
    nrf_drv_rtc_disable(&rtc);

    // rtc_handler will call rtcWakeUpTimerEventCallback_node()
    nrf_drv_rtc_uninit(&rtc);
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Disable tick interrupt
    nrf_drv_rtc_tick_disable(&rtc);

    //Enable Counter Compare interrupt
    old_cc = nrf_drv_rtc_counter_get(&rtc);
    new_cc = old_cc + gRTC_SF_PERIOD;
    new_cc&= RTC_WKUP_CNT_OVFLW_MASK;
    nrfx_rtc_cc_set( &rtc, 0, new_cc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}


/* HAL RTC Wakeup timer callback.
 *         Use the LED_NODE_Pin to see synchronization of tags
 *         on the oscilloscope if needed
 * */
void rtcWakeUpTimerEventCallback_node(void)
{
    uint32_t    new_cc;
    node_info_t *pNodeInfo = getNodeInfoPtr();

    if(!pNodeInfo)
    {
        return;
    }

//    DEBUG_NODE_LED_SF_ON;

    pNodeInfo->gRtcSFrameZeroCnt = nrf_drv_rtc_counter_get(&rtc);
    new_cc  = pNodeInfo->gRtcSFrameZeroCnt + gRTC_SF_PERIOD;
    new_cc &= RTC_WKUP_CNT_OVFLW_MASK;

    nrfx_rtc_cc_set( &rtc, 0, new_cc, true);

//    DEBUG_NODE_LED_SF_OFF;

//    if(app.trilatTask.Handle)
//    {
//        trilat_SF_cb();
//    }
}



//-----------------------------------------------------------------------------

/* @brief     app level
 *     RTOS-independent application level function.
 *     initializing of a TWR Node functionality.
 *
 * */
error_e node_process_init()
{
    if(!psNodeInfo)
    {
        psNodeInfo = NODE_MALLOC(sizeof(node_info_t));
    }

    node_info_t  *pNodeInfo = getNodeInfoPtr();

    if(!pNodeInfo)
    {
        return(_ERR_Cannot_Alloc_NodeMemory);
    }

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pNodeInfo, 0 , sizeof(node_info_t));

    /* Configure non-zero initial variables.1 : from app parameters */

    /* The Node has its configuration in the app->pConfig, see DEFAULT_CONFIG.
     * Tag will receive its configuration, such as
     * panID, tagAddr, node0Addr and TWR delays:
     * pollTx2FinalTxDelay_us and tag_replyDly_us from Range Config message.
     *
     * The reception timeouts calculated based on a known length of
     * RangeInit and Response packets.
     *
     * */
    pNodeInfo->pSfConfig = &app.pConfig->s.sfConfig; /**< Super Frame configuration */
    pNodeInfo->panID     = app.pConfig->s.panID;     /**< panID    */
    pNodeInfo->eui16     = app.pConfig->s.addr;      /**< Node's address */
    s_node_addr16        = pNodeInfo->eui16;

    { //calculate two-way ranging frame timings
        dwt_config_t *pCfg = &app.pConfig->dwt_config;  //dwt_config : holds node's UWB mode

        msg_t msg;

        msg.dataRate        = pCfg->dataRate;       //Deca define: e.g. DWT_BR_6M8
        msg.txPreambLength  = pCfg->txPreambLength; //Deca define: e.g. DWT_PLEN_128
        msg.sfdType         = pCfg->sfdType;
        msg.txPcode         = pCfg->txCode;
        msg.stsLength    = (pCfg->stsMode & DWT_STS_CONFIG_MASK) ? ((1<<(pCfg->stsLength+2))*8) : 0;

        msg.msg_len = sizeof(blink_msg_t);
        calculate_msg_time(&msg, &pNodeInfo->msg_time.blink);

        msg.msg_len = sizeof(rng_cfg_msg_t);
        calculate_msg_time(&msg, &pNodeInfo->msg_time.ranging_config);

        msg.msg_len = sizeof(poll_msg_t);
        calculate_msg_time(&msg, &pNodeInfo->msg_time.poll);

        msg.msg_len = sizeof(resp_pdoa_msg_t);
        calculate_msg_time(&msg, &pNodeInfo->msg_time.response);

        msg.msg_len = sizeof(final_msg_accel_t);
        calculate_msg_time(&msg, &pNodeInfo->msg_time.final);
    }

    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    disable_dw3000_irq();

    TWR_ENTER_CRITICAL();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)  /**< set callbacks to NULL inside dwt_initialise*/
    {
        TWR_EXIT_CRITICAL();
        return (_ERR_INIT);
    }

    set_dw_spi_fast_rate();

    uint32_t dev_id = dwt_readdevid();

    if ((dev_id == DWT_DW3000_PDOA_DEV_ID) || (dev_id == DWT_QM33120_PDOA_DEV_ID))
    {
        pNodeInfo->dw3000ChipType = AOA;

        diag_printf("Found AOA DW3000 chip. PDoA is available.\r\n");
    }
    else if (dev_id == DWT_DW3000_DEV_ID)
    {
        pNodeInfo->dw3000ChipType = NON_AOA;

        app.pConfig->dwt_config.pdoaMode = DWT_PDOA_M0;

        diag_printf("Found non-AOA DW3000 chip. PDoA is not available.\r\n");
    }
    else
    {
        TWR_EXIT_CRITICAL();
        diag_printf("Found unknown chip 0x%08lX. Stop.\r\n", (unsigned long int)dev_id);
        return DWT_ERROR;
    }

    /* read OTP Temperature calibration parameter */

    /* Derive node short address from chip EUI-64 last 2 bytes (last 4 hex digits).
     * Falls back to stored config values if OTP is unprogrammed (ES0 silicon workaround). */
    // {
    //     uint32_t lotId  = dwt_getlotid();
    //     uint32_t partId = dwt_getpartid();

    //     if ((lotId | partId) == 0)
    //     {
    //         if ((app.pConfig->s.partId | app.pConfig->s.lotId) == 0)
    //         {
    //             app.pConfig->s.lotId  = rand();
    //             app.pConfig->s.partId = rand();
    //             save_bssConfig(app.pConfig);
    //         }
    //         lotId = app.pConfig->s.lotId;
    //         partId = app.pConfig->s.partId;
    //     }
    //     node_store_eui(lotId, partId);
    //     pNodeInfo->eui16 = (uint16_t)(partId & 0xFFFF);
    //     s_node_addr16 = pNodeInfo->eui16;

    //     //display the EUI-64 only
    //     diag_printf("Node EUI-64: 0x%08X%08X\r\n", (unsigned int)lotId, (unsigned int)(partId & 0xFFFFFFFF));
    // }

    uint8_t euiLong[8] = {0};
    dwt_geteui(euiLong); //read EUI-64 from the chip (or from stored config if OTP is unprogrammed)
    memcpy(s_node_eui, euiLong, 8);
    pNodeInfo->eui16 = AR2U16(euiLong);

    /* Configure DW IC's UWB mode, sets power and antenna delays for TWR mode */
    rxtx_configure_t p;
    p.pdwCfg      = &app.pConfig->dwt_config;
    /* HW frame filtering: accept data frames addressed to this node (Poll, Final),
     * ACK frames, and blink frames (frame type 5 = multipurpose/reserved, no dest addr). */
    p.frameFilter     = DWT_FF_ENABLE_802_15_4;
    p.frameFilterMode = DWT_FF_DATA_EN       /* Poll & Final addressed to this node */
                      | DWT_FF_ACK_EN        /* ACK frames */
                      | DWT_FF_RSVD_EN       /* blink frame type (0xC5, type=5 reserved) */
                      | DWT_FF_MULTI_EN      /* blink frame type (0xC5, type=5 multipurpose in 802.15.4-2015) */
                      | DWT_FF_IMPBRCAST_EN; /* blinks carry no dest address — implicit broadcast */
    p.txAntDelay  = app.pConfig->s.antTx_a;
    p.rxAntDelay  = app.pConfig->s.antRx_a;
    p.panId       = pNodeInfo->panID;
    p.shortadd    = pNodeInfo->eui16;

    rxtx_configure(&p);
    node_apply_pdoa_offset_deg(app.pConfig->s.pdoaOffset_deg);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_TXRX_EN);  /**< DEBUG I/O 0&1 : configure TX/RX states to output on GPIOs */

    dwt_setcallbacks(twr_tx_node_cb, twr_rx_node_cb, twr_rx_timeout_cb, twr_rx_error_cb, NULL, NULL, NULL);

    dwt_setinterrupt( DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK
                    | (DWT_INT_ARFE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
                    | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFTO_BIT_MASK /*| DWT_INT_RXPTO_BIT_MASK*/), 0, DWT_ENABLE_INT_ONLY);

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);         /* DW3000 */

    /*
     * The dwt_initialize will read the default XTAL TRIM from the OTP or use the DEFAULT_XTAL_TRIM.
     * In this case we would apply the user-configured value.
     *
     * Bit 0x80 can be used to overwrite the OTP settings if any.
     * */
    if((dwt_getxtaltrim() == DEFAULT_XTAL_TRIM) || (app.pConfig->s.xtalTrim & ~XTAL_TRIM_BIT_MASK))
    {
        dwt_setxtaltrim(app.pConfig->s.xtalTrim & XTAL_TRIM_BIT_MASK);
    }

    /* End configuration of DW IC */

    /* Configure non-zero initial variables.2 : dwt_getlotid/dwt_getpartid are valid after dwt_initialise() */
    pNodeInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);

    {
        /* configure the RTC Wakeup timer with a high priority;
         * this timer is saving global Super Frame Timestamp,
         * so we want this timestamp as stable as we can.
         *
         * */
        NVIC_DisableIRQ(RTC0_IRQn);
        NVIC_SetPriority(RTC0_IRQn, PRIO_RTC_WKUP_IRQn);
        juniper_configure_hal_rtc_callback(rtcWakeUpTimerEventCallback_node);
        node_configure_rtc_wakeup(pNodeInfo->pSfConfig->sfPeriod_ms);
    }

    TWR_EXIT_CRITICAL();

    return (_NO_ERR);
}


/*
 * @brief
 *     Enable DW3000 IRQ to start
 * */
void node_process_start(void)
{
    enable_dw3000_irq();

    //start the RTC timer
    NVIC_EnableIRQ(RTC0_IRQn);

    diag_printf("PDOA Node Top Application: Started\r\n");
}


/* @brief     app level
 *     RTOS-independent application level function.
 *     deinitialize the pNodeInfo structure.
 *    This must be executed in protected mode.
 *
 * */
void node_process_terminate(void)
{
    {//stop the RTC timer
        NVIC_DisableIRQ(RTC0_IRQn);
        juniper_configure_hal_rtc_callback(NULL);
    }

    if(psNodeInfo)
    {
        NODE_FREE(psNodeInfo);
        psNodeInfo = NULL;
    }
}


static double pdoa2path_diff_ch5(float x)
{
    static const double xs[] = { -169.52599388379204, -164.55029585798817, -160.03367003367003, -155.23640661938535, -150.56637168141592, -145.09003215434083, -138.81159420289856, -130.99603174603175, -123.18204488778055, -113.84177215189874, -102.67299107142857, -91.66666666666667, -78.92921686746988, -66.17794486215539, -52.489329268292686, -38.47277936962751, -25.19344262295082, -10.773413897280967, 0.9860681114551083, 15.321236559139784, 31.16460396039604, 46.739010989010985, 62.48295454545455, 79.0, 93.02919708029196, 105.17538461538462, 116.59448818897638, 125.65702479338843, 135.40189873417722, 142.97699386503066, 151.34635416666666, 158.2935656836461, 165.31201044386424, 172.39769820971867, 180.0, 188.23993808049536, 198.43213296398892 };
    static const double ys[] = { -0.02277711059704047, -0.022690436814620973, -0.022431075107181914, -0.022000999373923726, -0.0214034827508634, -0.020643072700292697, -0.019725556401844816, -0.018657916708562303, -0.01744827900316916, -0.016105849359003235, -0.014640844476237574, -0.01306441392662416, -0.011388555298520233, -0.009626022887996853, -0.007790230630944395, -0.005895150014920516, -0.003955203747694204, -0.0019851559917306244, 0.0, 0.001985155991730628, 0.0039552037476942034, 0.005895150014920523, 0.007790230630944404, 0.009626022887996888, 0.011388555298520233, 0.013064413926624103, 0.014640844476237608, 0.01610584935900323, 0.017448279003169222, 0.018657916708562265, 0.019725556401844816, 0.02064307270029269, 0.021403482750863533, 0.02200099937392368, 0.02243107510718191, 0.02269043681462098, 0.022777110597040465 };
    static const int count = sizeof(xs)/sizeof(xs[0]);

    int i;
    double dx, dy;

    if (x < xs[0]) {
        return ys[0]; /* return minimum element */
    }

    if (x > xs[count-1]) {
        return ys[count-1]; /* return maximum */
    }

    /* find i, such that xs[i] <= x < xs[i+1] */
    for (i = 0; i < count-1; i++) {
        if (xs[i+1] > x) {
            break;
        }
    }

    /* interpolate */
    dx = xs[i+1] - xs[i];
    dy = ys[i+1] - ys[i];
    return ys[i] + (x - xs[i]) * dy / dx;
}

static double pdoa2path_diff_ch9(float x)
{
// NRF M3 AND M1
    static const double xs[] = { -175.909     , -166.321     , -166.25413889,
       -156.59927778, -152.748     , -146.94441667, -137.28955556,
       -134.172     , -127.63469444, -117.97983333, -108.41      ,
       -108.32497222,  -98.67011111,  -89.01525   ,  -81.162     ,
        -79.36038889,  -69.70552778,  -61.32      ,  -60.05066667,
        -50.39580556,  -45.013     ,  -40.74094444,  -31.08608333,
        -23.548     ,  -21.43122222,  -11.77636111,   -2.1215    ,
          0.0     ,    7.53336111,   17.18822222,   24.565     ,
         26.84308333,   36.49794444,   46.15280556,   47.076     ,
         55.80766667,   65.46252778,   68.196     ,   75.11738889,
         84.77225   ,   85.808     ,   94.42711111,  103.298     ,
        104.08197222,  113.73683333,  119.281     ,  123.39169444,
        133.04655556,  136.455     ,  142.70141667,  152.35627778,
        153.038     ,  162.01113889,  171.666   };

    static const double ys[] = { -0.0178831 , -0.01761142, -0.01760745, -0.01703355,
       -0.01680462, -0.01639303, -0.01570832, -0.01548722, -0.01503351,
       -0.01436343, -0.01369925, -0.01369237, -0.01291135, -0.01213032,
       -0.01149504, -0.01126319, -0.01002069, -0.00894155, -0.00872164,
       -0.00704895, -0.00611638, -0.00551712, -0.00416278, -0.00310537,
       -0.00283591, -0.00160689, -0.00037788,  0.        ,  0.00087544,
        0.00213954,  0.00310537,  0.00341008,  0.00470149,  0.0059929 ,
        0.00611638,  0.0072844 ,  0.0085759 ,  0.00894155,  0.00994505,
        0.01134487,  0.01149504,  0.01258128,  0.01369925,  0.01378695,
        0.01486701,  0.01548722,  0.01580255,  0.01654316,  0.01680462,
        0.01710852,  0.01757825,  0.01761142,  0.01774229,  0.0178831};

    static const int count = sizeof(xs)/sizeof(xs[0]);

    int i;
    double dx, dy;


    if (x < xs[0]) {
        return ys[0]; /* return minimum element */
    }

    if (x > xs[count-1]) {
        return ys[count-1]; /* return maximum */
    }

    /* find i, such that xs[i] <= x < xs[i+1] */
    for (i = 0; i < count-1; i++) {
        if (xs[i+1] > x) {
            break;
        }
    }

    /* interpolate */
    dx = xs[i+1] - xs[i];
    dy = ys[i+1] - ys[i];
    return ys[i] + (x - xs[i]) * dy / dx;
}
//-----------------------------------------------------------------------------
