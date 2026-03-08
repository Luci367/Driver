/******************************************************************************
 *  File: mhal_can.c
 *  Module: CAN MHAL (Hardware Abstraction Layer) — Implementation
 *
 *  AUTOSAR-compatible HAL layer for XCAN IP.
 *  Thin mapping layer: validates parameters, maps AUTOSAR types (HTH/HRH)
 *  to XCAN FIFO IDs via config lookup, converts PDU types, then delegates
 *  all hardware operations to HCL (mhcl_can).
 *
 *  Reference:
 *    - old_project/old_mhal_can.h  (API signatures, CanCtrlStatus)
 *    - mhcl_can.h / mhdl_can.h    (XCAN HCL/HDL layers)
 *    - XCAN User Manual v3.90
 *
 *  DO NOT reference can_driver.c or any other existing project files.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#include "mhal_can.h"
#include "mhal_can_internal.h"
#include "mhcl_can.h"
#include "can_debug.h"
#include <string.h>

/******************************************************************************
 *  AUTOSAR CAN Interface callbacks (provided by CanIf module at link time)
 *****************************************************************************/

extern void CanIf_RxIndication(const Can_HwType *Mailbox,
                               const Can_PduType *PduInfoPtr);
extern void CanIf_TxConfirmation(uint32 swPduHandle);
extern void CanIf_ControllerBusOff(uint8 ControllerId);
extern void CanIf_ControllerModeIndication(uint8 ControllerId,
                                           Can_ControllerStateType ControllerMode);

/******************************************************************************
 *  Private defines
 *****************************************************************************/

/* CAN_BUSY expressed as Std_ReturnType (AUTOSAR convention) */
#define CAN_BUSY_STD   ((Std_ReturnType)0x02U)

#define CAN_MAX_HTH_PER_CTRL  32u
#define CAN_FD_MAX_SDU_BYTES  64u

/******************************************************************************
 *  Global variables (extern'd in mhal_can.h)
 *****************************************************************************/

VAR(CanCtrlStatus, CAN_VAR) can_hd[CAN_CTRL_CONFIG_CNT];

P2CONST(Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) pCanHalCfg = NULL_PTR;

/******************************************************************************
 *  Module-private state
 *****************************************************************************/

static Mhcl_Can_ControllerType mhcl_ctrl[CAN_CTRL_CONFIG_CNT];

static uint32 tx_pdu_handle[CAN_CTRL_CONFIG_CNT][CAN_MAX_HTH_PER_CTRL];

/******************************************************************************
 *  Internal helpers
 *****************************************************************************/

static uint8 hal_hth_local_idx(Can_HwHandleType Hth)
{
    return (uint8)(Hth - pCanHalCfg->HwObjTxstartIdx);
}

/* Copy SDU byte array into 32-bit word array (little-endian packing) */
static void hal_sdu_to_data_words(const uint8 *sdu, uint32_t *data_word,
                                  uint32 length)
{
    uint32 num_words = (length + 3u) / 4u;
    uint32 i, j;
    for (i = 0u; i < num_words; i++) {
        uint32_t word = 0u;
        for (j = 0u; (j < 4u) && ((i * 4u + j) < length); j++) {
            word |= ((uint32_t)sdu[i * 4u + j]) << (j * 8u);
        }
        data_word[i] = word;
    }
}

/* Copy 32-bit word array back to SDU byte array (little-endian) */
static void hal_data_words_to_sdu(const uint32_t *data_word, uint8 *sdu,
                                  uint32 length)
{
    uint32 num_words = (length + 3u) / 4u;
    uint32 i, j;
    for (i = 0u; i < num_words; i++) {
        for (j = 0u; (j < 4u) && ((i * 4u + j) < length); j++) {
            sdu[i * 4u + j] = (uint8)(data_word[i] >> (j * 8u));
        }
    }
}

/* Map HCL error state to AUTOSAR error state */
static Can_ErrorStateType hal_map_error_state(Mhcl_Can_ErrorStateType hcl_err)
{
    switch (hcl_err) {
    case MHCL_CAN_ERR_PASSIVE: return CAN_ERRORSTATE_PASSIVE;
    case MHCL_CAN_ERR_BUSOFF:  return CAN_ERRORSTATE_BUSOFF;
    case MHCL_CAN_ERR_ACTIVE:
    default:                    return CAN_ERRORSTATE_ACTIVE;
    }
}

/* Determine CC vs FD from HW object config and the CAN-ID FD mask */
static Mhcl_Can_FrameFormatType hal_determine_ff(
    const Can_HardwareObjectType *hwObj, uint32 canId)
{
    if (IS_FD_FRAME(canId) || (hwObj->CanObjectPayloadLength > CAN_PL_8)) {
        return MHCL_CAN_FF_FD;
    }
    return MHCL_CAN_FF_CC;
}

/* Build a masked CAN ID + encode the FD / EXT flags for RX PDU output */
static uint32 hal_build_rx_can_id(const Mhcl_Can_MsgType *msg)
{
    uint32 canId;

    if (msg->id_type == MHCL_CAN_ID_EXTENDED) {
        canId = (msg->frame_id & EXT_ID_MASK) | EXT_ID_DATA_MASK;
    } else {
        canId = msg->frame_id & STD_ID_MASK;
    }

    if (msg->frame_format == MHCL_CAN_FF_FD) {
        canId |= (msg->id_type == MHCL_CAN_ID_EXTENDED)
                     ? FD_EXT_MASK : FD_STD_MASK;
    }

    return canId;
}

/******************************************************************************
 *  HCL callbacks  (registered in can_hal_init, invoked by HCL IRQ processing)
 *****************************************************************************/

static void hal_cb_tx_confirmation(uint8_t controller, uint32_t fifo_or_slot,
                                   bool is_pq)
{
    uint16 hoh;

    (void)is_pq;

    for (hoh = pCanHalCfg->HwObjTxstartIdx;
         hoh < pCanHalCfg->HwObjCfgNum; hoh++) {

        const Can_HardwareObjectType *hwObj =
            &pCanHalCfg->CanHardwareObject[hoh];

        if (hwObj->CanControllerRef->ControllerId != controller) { continue; }
        if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT)    { continue; }
        if ((uint32_t)hwObj->HwFifoId != fifo_or_slot)           { continue; }

        uint8 hth_idx = hal_hth_local_idx((Can_HwHandleType)hoh);
        uint32 hth_bit = (uint32)1u << hth_idx;

        if ((can_hd[controller].HthObjBusy & hth_bit) != 0u) {
            can_hd[controller].HthObjBusy &= ~hth_bit;
            CanIf_TxConfirmation(tx_pdu_handle[controller][hth_idx]);
        }
        break;
    }
}

static void hal_cb_rx_indication(uint8_t controller, uint32_t fifo)
{
    Mhcl_Can_MsgType msg;
    Can_PduType      pdu;
    uint8            sdu_buf[CAN_FD_MAX_SDU_BYTES];
    Can_HwType       hw;

    while (mhcl_can_rx_fifo_dequeue(&mhcl_ctrl[controller], fifo, &msg) != 0) {

        uint32 data_bytes = mhcl_can_dlc_to_bytes(msg.frame_format, msg.dlc);
        if (data_bytes > CAN_FD_MAX_SDU_BYTES) {
            data_bytes = CAN_FD_MAX_SDU_BYTES;
        }

        hal_data_words_to_sdu(msg.data_word, sdu_buf, data_bytes);

        pdu.id          = hal_build_rx_can_id(&msg);
        pdu.length      = (uint8)data_bytes;
        pdu.sdu         = sdu_buf;
        pdu.swPduHandle = 0u;

        hw.ControllerId = controller;
        hw.CanId        = pdu.id;
        hw.Hoh          = 0u;

        /* Find the matching HRH for this controller + RX FIFO */
        for (uint16 hoh = 0u; hoh < pCanHalCfg->HwObjTxstartIdx; hoh++) {
            const Can_HardwareObjectType *hwObj =
                &pCanHalCfg->CanHardwareObject[hoh];
            if (hwObj->CanControllerRef->ControllerId != controller) { continue; }
            if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE)     { continue; }
            if ((uint32_t)hwObj->HwFifoId != fifo)                   { continue; }
            hw.Hoh = (Can_HwHandleType)hoh;
            break;
        }

        CanIf_RxIndication(&hw, &pdu);
    }
}

static void hal_cb_busoff(uint8_t controller)
{
    can_hd[controller].CtrlState = CAN_CS_STOPPED;
    can_hd[controller].HthObjBusy = 0u;
    CanIf_ControllerBusOff(controller);
}

static void hal_cb_error_passive(uint8_t controller)
{
    (void)controller;
}

static void hal_cb_error_active(uint8_t controller)
{
    (void)controller;
}

/******************************************************************************
 *  Config setup
 *****************************************************************************/

FUNC(void, CAN_CODE)
can_hal_set_config(P2CONST(Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) Config)
{
    CAN_DBG_INFO(DBG_HAL, "set_config cfg=%p", (const void *)Config);
    pCanHalCfg = Config;
}

/******************************************************************************
 *  Init / Deinit
 *****************************************************************************/

FUNC(void, CAN_CODE) can_hal_init(VAR(uint8, AUTOMATIC) cid)
{
    CAN_DBG_INFO(DBG_HAL, "init cid=%u", cid);
    if (pCanHalCfg == NULL_PTR) {
        CAN_DBG_ERR(DBG_HAL, "init failed: config NULL");
        return;
    }
    if (cid >= pCanHalCfg->ControllerCfgNum) {
        CAN_DBG_ERR(DBG_HAL, "init failed: cid=%u out of range", cid);
        return;
    }

    const Can_ControllerType *ctrlCfg = GET_CTRL_CFG(cid);
    CanCtrlStatus            *ctrlData = GET_CTRL_DATA(cid);
    Mhcl_Can_ControllerType  *hclCtrl = &mhcl_ctrl[cid];

    /* ---- Build HCL config from AUTOSAR post-build config ---- */
    Mhcl_Can_ConfigType hcl_cfg;
    (void)memset(&hcl_cfg, 0, sizeof(hcl_cfg));

    hcl_cfg.xcan_base_addr  = ctrlCfg->BaseAddress;
    hcl_cfg.lmem_base       = ctrlCfg->LmemBaseAddress;
    hcl_cfg.lmem_size_words = ctrlCfg->LmemSizeWords;
    hcl_cfg.instance_id     = ctrlCfg->ControllerId;

    /* --- Nominal bit timing (from default baudrate entry) --- */
    const Can_ControllerBaudrateCfgType *nomCfg =
        &ctrlCfg->BaudrateCfg[ctrlCfg->DefaultBaudrateIdx];
    hcl_cfg.brp                = nomCfg->Prescaler;
    hcl_cfg.nominal.prop_seg   = nomCfg->PropSeg;
    hcl_cfg.nominal.phase_seg1 = nomCfg->PhaseSeg1;
    hcl_cfg.nominal.phase_seg2 = nomCfg->PhaseSeg2;
    hcl_cfg.nominal.sjw        = nomCfg->SyncJumpWidth;
    hcl_cfg.nominal.tdc_offset = 0u;

    /* --- FD data bit timing --- */
    if (ctrlCfg->CanControllerFdBaudrateConfig != NULL_PTR) {
        const Can_ControllerFdBaudrateCfgType *fdCfg =
            ctrlCfg->CanControllerFdBaudrateConfig;
        hcl_cfg.data_fd.prop_seg   = fdCfg->CanControllerPropSeg;
        hcl_cfg.data_fd.phase_seg1 = fdCfg->CanControllerSeg1;
        hcl_cfg.data_fd.phase_seg2 = fdCfg->CanControllerSeg2;
        hcl_cfg.data_fd.sjw        = fdCfg->CanControllerSyncJumpWidth;
        hcl_cfg.data_fd.tdc_offset = 0u;
        hcl_cfg.prt_mode.fd_ena    = true;
    }

    /* --- XL data bit timing --- */
    if ((ctrlCfg->XlEnable == TRUE) &&
        (ctrlCfg->CanControllerXlBaudrateConfig != NULL_PTR)) {
        const Can_ControllerXlBaudrateCfgType *xlCfg =
            ctrlCfg->CanControllerXlBaudrateConfig;
        hcl_cfg.data_xl.prop_seg   = xlCfg->CanControllerXlPropSeg;
        hcl_cfg.data_xl.phase_seg1 = xlCfg->CanControllerXlSeg1;
        hcl_cfg.data_xl.phase_seg2 = xlCfg->CanControllerXlSeg2;
        hcl_cfg.data_xl.sjw        = xlCfg->CanControllerXlSyncJumpWidth;
        hcl_cfg.data_xl.tdc_offset = xlCfg->CanControllerXlTdcOffset;
        hcl_cfg.prt_mode.xl_ena    = true;
    }

    /* --- PWME (XL pulse-width modulation) --- */
    if (ctrlCfg->CanControllerPwmeConfig != NULL_PTR) {
        hcl_cfg.pwme.pwmo = ctrlCfg->CanControllerPwmeConfig->Pwmo;
        hcl_cfg.pwme.pwms = ctrlCfg->CanControllerPwmeConfig->Pwms;
        hcl_cfg.pwme.pwml = ctrlCfg->CanControllerPwmeConfig->Pwml;
    }

    /* --- MH global config --- */
    hcl_cfg.retrans_max       = ctrlCfg->RetransMax;
    hcl_cfg.rx_continuous_mode = (ctrlCfg->RxContinuousMode == TRUE);

    /* --- LMEM layout offsets --- */
    hcl_cfg.lmem_fq_base        = ctrlCfg->LmemFqBase;
    hcl_cfg.lmem_pq_base        = ctrlCfg->LmemPqBase;
    hcl_cfg.lmem_rx_filter_base = ctrlCfg->LmemRxFilterBase;

    /* --- TX FIFO config: scan TX HW objects for this controller --- */
    {
        uint16 hoh;
        for (hoh = pCanHalCfg->HwObjTxstartIdx;
             hoh < pCanHalCfg->HwObjCfgNum; hoh++) {

            const Can_HardwareObjectType *hwObj =
                &pCanHalCfg->CanHardwareObject[hoh];
            if (hwObj->CanControllerRef->ControllerId != cid) { continue; }
            if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT) { continue; }

            uint8 fid = hwObj->HwFifoId;
            if (fid < MHCL_CAN_MAX_TX_FIFO) {
                hcl_cfg.tx_fifo[fid].enabled        = true;
                hcl_cfg.tx_fifo[fid].fifo_size      = hwObj->FifoSize;
                hcl_cfg.tx_fifo[fid].dc_size_word   = hwObj->DcSizeWord;
                hcl_cfg.tx_fifo[fid].desc_array_addr = hwObj->DescArrayAddr;
                hcl_cfg.tx_fifo[fid].dc_start_addr  = hwObj->DcStartAddr;
            }
        }
    }

    /* --- TX Priority Queue --- */
    hcl_cfg.tx_pq.num_slots      = ctrlCfg->TxPqNumSlots;
    hcl_cfg.tx_pq.desc_array_addr = ctrlCfg->TxPqDescArrayAddr;

    /* --- RX FIFO config: scan RX HW objects for this controller --- */
    {
        uint16 hoh;
        for (hoh = 0u; hoh < pCanHalCfg->HwObjTxstartIdx; hoh++) {

            const Can_HardwareObjectType *hwObj =
                &pCanHalCfg->CanHardwareObject[hoh];
            if (hwObj->CanControllerRef->ControllerId != cid) { continue; }
            if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { continue; }

            uint8 fid = hwObj->HwFifoId;
            if (fid < MHCL_CAN_MAX_RX_FIFO) {
                hcl_cfg.rx_fifo[fid].enabled        = true;
                hcl_cfg.rx_fifo[fid].fifo_size      = hwObj->FifoSize;
                hcl_cfg.rx_fifo[fid].dc_size_word   = hwObj->DcSizeWord;
                hcl_cfg.rx_fifo[fid].desc_array_addr = hwObj->DescArrayAddr;
                hcl_cfg.rx_fifo[fid].dc_start_addr  = hwObj->DcStartAddr;
            }
        }
    }

    /* --- RX filter config ---
     * Build reference pairs from HW object filter configuration.
     * If no filters are configured, default to accept-all. */
    {
        static Mhcl_Can_RxFilterRefPairType ref_pairs_buf[MHCL_CAN_MAX_RX_FIFO * 8u];
        uint32 rp_count = 0u;
        uint16 hoh;

        for (hoh = 0u; hoh < pCanHalCfg->HwObjTxstartIdx; hoh++) {
            const Can_HardwareObjectType *hwObj =
                &pCanHalCfg->CanHardwareObject[hoh];
            if (hwObj->CanControllerRef->ControllerId != cid) { continue; }
            if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { continue; }
            if (hwObj->CanHwFilter == NULL_PTR) { continue; }

            uint8 fi;
            for (fi = 0u; fi < hwObj->HwFilterCount; fi++) {
                if (rp_count < (uint32)(sizeof(ref_pairs_buf) / sizeof(ref_pairs_buf[0]))) {
                    ref_pairs_buf[rp_count].value = hwObj->CanHwFilter[fi].CanHwFilterCode;
                    ref_pairs_buf[rp_count].mask  = hwObj->CanHwFilter[fi].CanHwFilterMask;
                    rp_count++;
                }
            }
        }

        hcl_cfg.rx_filter.num_elements         = 0u;
        hcl_cfg.rx_filter.elements             = NULL;
        hcl_cfg.rx_filter.accept_non_matching  = true;
        hcl_cfg.rx_filter.non_matching_fifo    = 0u;
        hcl_cfg.rx_filter.accept_non_filtered  = true;
        hcl_cfg.rx_filter.non_filtered_threshold = 0u;

        if (rp_count > 0u) {
            hcl_cfg.rx_filter.num_ref_pairs = rp_count;
            hcl_cfg.rx_filter.ref_pairs     = ref_pairs_buf;
        } else {
            hcl_cfg.rx_filter.num_ref_pairs = 0u;
            hcl_cfg.rx_filter.ref_pairs     = NULL;
        }
    }

    /* --- IRC --- */
    hcl_cfg.irc.func_ena_mask   = ctrlCfg->IrcFuncEnaMask;
    hcl_cfg.irc.err_ena_mask    = ctrlCfg->IrcErrEnaMask;
    hcl_cfg.irc.safety_ena_mask = ctrlCfg->IrcSafetyEnaMask;

    /* ---- Call HCL init ---- */
    Mhcl_Can_ReturnType ret = mhcl_can_init(hclCtrl, &hcl_cfg);
    if (ret != MHCL_CAN_OK) {
        CAN_DBG_ERR(DBG_HAL, "init cid=%u HCL failed", cid);
        ctrlData->CtrlState = CAN_CS_UNINIT;
        return;
    }

    /* Register HAL callbacks into HCL controller */
    hclCtrl->cb_tx_confirmation = hal_cb_tx_confirmation;
    hclCtrl->cb_rx_indication   = hal_cb_rx_indication;
    hclCtrl->cb_busoff          = hal_cb_busoff;
    hclCtrl->cb_error_passive   = hal_cb_error_passive;
    hclCtrl->cb_error_active    = hal_cb_error_active;

    /* Init HAL runtime state */
    ctrlData->CtrlState  = CAN_CS_STOPPED;
    ctrlData->RefCounter = 0u;
    ctrlData->HthObjBusy = 0u;
    (void)memset(tx_pdu_handle[cid], 0, sizeof(tx_pdu_handle[cid]));
    CAN_DBG_INFO(DBG_HAL, "init cid=%u OK, state=STOPPED", cid);
}

FUNC(void, CAN_CODE) can_hal_deinit(VAR(uint8, AUTOMATIC) cid)
{
    CAN_DBG_INFO(DBG_HAL, "deinit cid=%u", cid);
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }

    (void)mhcl_can_deinit(&mhcl_ctrl[cid]);

    can_hd[cid].CtrlState  = CAN_CS_UNINIT;
    can_hd[cid].RefCounter = 0u;
    can_hd[cid].HthObjBusy = 0u;
}

/******************************************************************************
 *  Mode control
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            VAR(Can_StateTransitionType, AUTOMATIC) Transition)
{
    CAN_DBG_INFO(DBG_HAL, "set_mode cid=%u transition=%u", cid, Transition);
    if (pCanHalCfg == NULL_PTR)              { return E_NOT_OK; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return E_NOT_OK; }

    CanCtrlStatus       *ctrlData = GET_CTRL_DATA(cid);
    Mhcl_Can_ReturnType  ret;

    switch (Transition) {
    case CAN_T_START:
        if (ctrlData->CtrlState != CAN_CS_STOPPED) { return E_NOT_OK; }
        ret = mhcl_can_start(&mhcl_ctrl[cid]);
        if (ret != MHCL_CAN_OK) { return E_NOT_OK; }
        ctrlData->CtrlState = CAN_CS_STARTED;
        CanIf_ControllerModeIndication(cid, CAN_CS_STARTED);
        break;

    case CAN_T_STOP:
        if (ctrlData->CtrlState != CAN_CS_STARTED) { return E_NOT_OK; }
        ret = mhcl_can_stop(&mhcl_ctrl[cid]);
        if (ret != MHCL_CAN_OK) { return E_NOT_OK; }
        ctrlData->CtrlState  = CAN_CS_STOPPED;
        ctrlData->HthObjBusy = 0u;
        CanIf_ControllerModeIndication(cid, CAN_CS_STOPPED);
        break;

    default:
        return E_NOT_OK;
    }

    return E_OK;
}

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            P2VAR(Can_ControllerStateType, AUTOMATIC,
                                  CAN_APPL_DATA) ControllerModePtr)
{
    if (pCanHalCfg == NULL_PTR)              { return E_NOT_OK; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return E_NOT_OK; }
    if (ControllerModePtr == NULL_PTR)       { return E_NOT_OK; }

    *ControllerModePtr = can_hd[cid].CtrlState;
    return E_OK;
}

/******************************************************************************
 *  CC / FD Write
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
              P2CONST(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo)
{
    if (pCanHalCfg == NULL_PTR)           { return E_NOT_OK; }
    if (Hth >= pCanHalCfg->HwObjCfgNum)  { return E_NOT_OK; }
    if (PduInfo == NULL_PTR)              { return E_NOT_OK; }
    if (PduInfo->sdu == NULL_PTR)         { return E_NOT_OK; }

    CAN_DBG_VERB(DBG_HAL, "write Hth=%u id=0x%08X len=%u",
                 Hth, PduInfo->id, PduInfo->length);

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hth];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT) { return E_NOT_OK; }

    uint8 cid = hwObj->CanControllerRef->ControllerId;
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return E_NOT_OK; }

    /* Check if HTH is already occupied */
    uint8  hth_idx = hal_hth_local_idx(Hth);
    uint32 hth_bit = (uint32)1u << hth_idx;
    if ((can_hd[cid].HthObjBusy & hth_bit) != 0u) {
        CAN_DBG_WARN(DBG_HAL, "write Hth=%u BUSY", Hth);
        return CAN_BUSY_STD;
    }

    Mhcl_Can_FrameFormatType ff = hal_determine_ff(hwObj, PduInfo->id);

    /* Build HCL TX message */
    Mhcl_Can_MsgType msg;
    (void)memset(&msg, 0, sizeof(msg));

    msg.direction    = MHCL_CAN_DIR_TX;
    msg.frame_format = ff;
    msg.id_type      = IS_EXT_ID(PduInfo->id) ? MHCL_CAN_ID_EXTENDED
                                               : MHCL_CAN_ID_BASE;
    msg.frame_id     = (msg.id_type == MHCL_CAN_ID_EXTENDED)
                           ? (PduInfo->id & EXT_ID_MASK)
                           : (PduInfo->id & STD_ID_MASK);

    msg.dlc = mhcl_can_bytes_to_dlc(ff, (uint32)PduInfo->length);

    if (ff == MHCL_CAN_FF_FD) {
        const Can_ControllerType *ctrlCfg = hwObj->CanControllerRef;
        if ((ctrlCfg->CanControllerFdBaudrateConfig != NULL_PTR) &&
            (ctrlCfg->CanControllerFdBaudrateConfig
                 ->CanControllerTxBitRateSwitch == TRUE)) {
            msg.brs = 1u;
        }
    }

    hal_sdu_to_data_words(PduInfo->sdu, msg.data_word,
                          (uint32)PduInfo->length);

    /* Enqueue via HCL */
    int result = mhcl_can_tx_fifo_enqueue(&mhcl_ctrl[cid],
                                          (uint32_t)hwObj->HwFifoId, &msg);
    if (result == 0) {
        return CAN_BUSY_STD;
    }

    can_hd[cid].HthObjBusy |= hth_bit;
    tx_pdu_handle[cid][hth_idx] = PduInfo->swPduHandle;

    return E_OK;
}

/******************************************************************************
 *  CC / FD Read
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
             P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo)
{
    CAN_DBG_VERB(DBG_HAL, "read Hrh=%u", Hrh);
    if (pCanHalCfg == NULL_PTR)           { return E_NOT_OK; }
    if (Hrh >= pCanHalCfg->HwObjCfgNum)  { return E_NOT_OK; }
    if (PduInfo == NULL_PTR)              { return E_NOT_OK; }
    if (PduInfo->sdu == NULL_PTR)         { return E_NOT_OK; }

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hrh];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { return E_NOT_OK; }

    uint8 cid = hwObj->CanControllerRef->ControllerId;
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return E_NOT_OK; }

    Mhcl_Can_MsgType msg;
    (void)memset(&msg, 0, sizeof(msg));

    int result = mhcl_can_rx_fifo_dequeue(&mhcl_ctrl[cid],
                                          (uint32_t)hwObj->HwFifoId, &msg);
    if (result == 0) {
        return E_NOT_OK;
    }

    uint32 data_bytes = mhcl_can_dlc_to_bytes(msg.frame_format, msg.dlc);
    if (data_bytes > CAN_FD_MAX_SDU_BYTES) {
        data_bytes = CAN_FD_MAX_SDU_BYTES;
    }

    PduInfo->id          = hal_build_rx_can_id(&msg);
    PduInfo->length      = (uint8)data_bytes;
    PduInfo->swPduHandle = 0u;

    hal_data_words_to_sdu(msg.data_word, PduInfo->sdu, data_bytes);

    return E_OK;
}

/******************************************************************************
 *  Interrupt management  (SWS_Can_00202 nesting)
 *****************************************************************************/

FUNC(void, CAN_CODE)
can_hal_enable_controller_interrupts(VAR(uint8, AUTOMATIC) cid)
{
    CAN_DBG_INFO(DBG_HAL, "enable_irq cid=%u", cid);
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }

    CanCtrlStatus *ctrlData = GET_CTRL_DATA(cid);

    if (ctrlData->RefCounter > 0u) {
        ctrlData->RefCounter--;
    }

    if (ctrlData->RefCounter == 0u) {
        const Can_ControllerType *ctrlCfg = GET_CTRL_CFG(cid);
        mhcl_can_enable_interrupts(&mhcl_ctrl[cid],
                                   ctrlCfg->IrcFuncEnaMask,
                                   ctrlCfg->IrcErrEnaMask,
                                   ctrlCfg->IrcSafetyEnaMask);
    }
}

FUNC(void, CAN_CODE)
can_hal_disable_controller_interrupts(VAR(uint8, AUTOMATIC) cid)
{
    CAN_DBG_INFO(DBG_HAL, "disable_irq cid=%u", cid);
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }

    CanCtrlStatus *ctrlData = GET_CTRL_DATA(cid);
    ctrlData->RefCounter++;

    if (ctrlData->RefCounter == 1u) {
        mhcl_can_disable_interrupts(&mhcl_ctrl[cid]);
    }
}

/******************************************************************************
 *  Error / status
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_controller_error_state(VAR(uint8, AUTOMATIC) cid,
                                   P2VAR(Can_ErrorStateType, AUTOMATIC,
                                         CAN_APPL_DATA) ErrorStatePtr)
{
    if (pCanHalCfg == NULL_PTR)              { return E_NOT_OK; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return E_NOT_OK; }
    if (ErrorStatePtr == NULL_PTR)           { return E_NOT_OK; }

    *ErrorStatePtr = hal_map_error_state(
        mhcl_can_get_error_state(&mhcl_ctrl[cid]));
    return E_OK;
}

FUNC(uint8, CAN_CODE)
can_hal_get_rx_error_count(VAR(uint8, AUTOMATIC) cid)
{
    if (pCanHalCfg == NULL_PTR)              { return 0u; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return 0u; }

    uint32_t rec = mhcl_can_get_rec(&mhcl_ctrl[cid]);
    return (rec > 255u) ? 255u : (uint8)rec;
}

FUNC(uint8, CAN_CODE)
can_hal_get_tx_error_count(VAR(uint8, AUTOMATIC) cid)
{
    if (pCanHalCfg == NULL_PTR)              { return 0u; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return 0u; }

    uint32_t tec = mhcl_can_get_tec(&mhcl_ctrl[cid]);
    return (tec > 255u) ? 255u : (uint8)tec;
}

/******************************************************************************
 *  Baudrate
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_baudrate(VAR(uint8, AUTOMATIC) cid,
                     VAR(uint16, AUTOMATIC) arb_baudrate)
{
    CAN_DBG_INFO(DBG_HAL, "set_baudrate cid=%u br_id=%u", cid, arb_baudrate);
    if (pCanHalCfg == NULL_PTR)              { return E_NOT_OK; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return E_NOT_OK; }
    if (can_hd[cid].CtrlState != CAN_CS_STOPPED) { return E_NOT_OK; }

    const Can_ControllerType *ctrlCfg = GET_CTRL_CFG(cid);

    const Can_ControllerBaudrateCfgType *nomCfg = NULL_PTR;
    uint8 i;
    for (i = 0u; i < ctrlCfg->BaudrateCfgCount; i++) {
        if (ctrlCfg->BaudrateCfg[i].BaudRateConfigId == arb_baudrate) {
            nomCfg = &ctrlCfg->BaudrateCfg[i];
            break;
        }
    }
    if (nomCfg == NULL_PTR) { return E_NOT_OK; }

    /* Nominal */
    Mhcl_Can_BitTimingType nominal_bt;
    nominal_bt.prop_seg   = nomCfg->PropSeg;
    nominal_bt.phase_seg1 = nomCfg->PhaseSeg1;
    nominal_bt.phase_seg2 = nomCfg->PhaseSeg2;
    nominal_bt.sjw        = nomCfg->SyncJumpWidth;
    nominal_bt.tdc_offset = 0u;

    /* FD data */
    Mhcl_Can_BitTimingType  fd_bt;
    Mhcl_Can_BitTimingType *fd_bt_ptr = NULL;
    if (ctrlCfg->CanControllerFdBaudrateConfig != NULL_PTR) {
        const Can_ControllerFdBaudrateCfgType *fdCfg =
            ctrlCfg->CanControllerFdBaudrateConfig;
        fd_bt.prop_seg   = fdCfg->CanControllerPropSeg;
        fd_bt.phase_seg1 = fdCfg->CanControllerSeg1;
        fd_bt.phase_seg2 = fdCfg->CanControllerSeg2;
        fd_bt.sjw        = fdCfg->CanControllerSyncJumpWidth;
        fd_bt.tdc_offset = 0u;
        fd_bt_ptr        = &fd_bt;
    }

    /* XL data */
    Mhcl_Can_BitTimingType  xl_bt;
    Mhcl_Can_BitTimingType *xl_bt_ptr = NULL;
    if ((ctrlCfg->XlEnable == TRUE) &&
        (ctrlCfg->CanControllerXlBaudrateConfig != NULL_PTR)) {
        const Can_ControllerXlBaudrateCfgType *xlCfg =
            ctrlCfg->CanControllerXlBaudrateConfig;
        xl_bt.prop_seg   = xlCfg->CanControllerXlPropSeg;
        xl_bt.phase_seg1 = xlCfg->CanControllerXlSeg1;
        xl_bt.phase_seg2 = xlCfg->CanControllerXlSeg2;
        xl_bt.sjw        = xlCfg->CanControllerXlSyncJumpWidth;
        xl_bt.tdc_offset = xlCfg->CanControllerXlTdcOffset;
        xl_bt_ptr        = &xl_bt;
    }

    /* PWME */
    Mhcl_Can_PwmeType  pwme_local;
    Mhcl_Can_PwmeType *pwme_ptr = NULL;
    if (ctrlCfg->CanControllerPwmeConfig != NULL_PTR) {
        pwme_local.pwmo = ctrlCfg->CanControllerPwmeConfig->Pwmo;
        pwme_local.pwms = ctrlCfg->CanControllerPwmeConfig->Pwms;
        pwme_local.pwml = ctrlCfg->CanControllerPwmeConfig->Pwml;
        pwme_ptr        = &pwme_local;
    }

    /* PRT mode flags */
    Mhcl_Can_PrtModeType mode_local;
    (void)memset(&mode_local, 0, sizeof(mode_local));
    mode_local.fd_ena = (ctrlCfg->CanControllerFdBaudrateConfig != NULL_PTR);
    mode_local.xl_ena = (ctrlCfg->XlEnable == TRUE);

    Mhcl_Can_ReturnType br_ret = mhcl_can_set_baudrate(&mhcl_ctrl[cid],
                                    nomCfg->Prescaler,
                                    &nominal_bt, fd_bt_ptr, xl_bt_ptr,
                                    pwme_ptr, &mode_local);

    return (br_ret == MHCL_CAN_OK) ? E_OK : E_NOT_OK;
}

/******************************************************************************
 *  Main functions (polling)
 *****************************************************************************/

FUNC(void, CAN_CODE) can_hal_main_function_read(VAR(uint8, AUTOMATIC) cid)
{
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return; }

    uint16 hoh;
    for (hoh = 0u; hoh < pCanHalCfg->HwObjTxstartIdx; hoh++) {

        const Can_HardwareObjectType *hwObj =
            &pCanHalCfg->CanHardwareObject[hoh];

        if (hwObj->CanControllerRef->ControllerId != cid) { continue; }
        if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { continue; }
        if (hwObj->PollingMode != TRUE)                      { continue; }

        Mhcl_Can_MsgType msg;
        Can_PduType       pdu;
        uint8             sdu_buf[CAN_FD_MAX_SDU_BYTES];

        while (mhcl_can_rx_fifo_dequeue(&mhcl_ctrl[cid],
                                        (uint32_t)hwObj->HwFifoId,
                                        &msg) != 0) {

            uint32 data_bytes =
                mhcl_can_dlc_to_bytes(msg.frame_format, msg.dlc);
            if (data_bytes > CAN_FD_MAX_SDU_BYTES) {
                data_bytes = CAN_FD_MAX_SDU_BYTES;
            }

            hal_data_words_to_sdu(msg.data_word, sdu_buf, data_bytes);

            pdu.id          = hal_build_rx_can_id(&msg);
            pdu.length      = (uint8)data_bytes;
            pdu.sdu         = sdu_buf;
            pdu.swPduHandle = 0u;

            Can_HwType hw;
            hw.Hoh          = (Can_HwHandleType)hoh;
            hw.ControllerId = cid;
            hw.CanId        = pdu.id;

            CanIf_RxIndication(&hw, &pdu);
        }
    }
}

FUNC(void, CAN_CODE) can_hal_main_function_write(VAR(uint8, AUTOMATIC) cid)
{
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return; }
    if (can_hd[cid].HthObjBusy == 0u)       { return; }

    uint16 hoh;
    for (hoh = pCanHalCfg->HwObjTxstartIdx;
         hoh < pCanHalCfg->HwObjCfgNum; hoh++) {

        const Can_HardwareObjectType *hwObj =
            &pCanHalCfg->CanHardwareObject[hoh];

        if (hwObj->CanControllerRef->ControllerId != cid) { continue; }
        if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT) { continue; }
        if (hwObj->PollingMode != TRUE)                       { continue; }

        uint8  hth_idx = hal_hth_local_idx((Can_HwHandleType)hoh);
        uint32 hth_bit = (uint32)1u << hth_idx;

        if ((can_hd[cid].HthObjBusy & hth_bit) == 0u) { continue; }

        if (mhcl_can_tx_fifo_check_sent(&mhcl_ctrl[cid],
                                         (uint32_t)hwObj->HwFifoId)) {
            can_hd[cid].HthObjBusy &= ~hth_bit;
            CanIf_TxConfirmation(tx_pdu_handle[cid][hth_idx]);
        }
    }
}

FUNC(void, CAN_CODE) can_hal_main_function_busoff(VAR(uint8, AUTOMATIC) cid)
{
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return; }

    Mhcl_Can_ErrorStateType err = mhcl_can_get_error_state(&mhcl_ctrl[cid]);
    if (err == MHCL_CAN_ERR_BUSOFF) {
        CAN_DBG_ERR(DBG_HAL, "busoff detected cid=%u", cid);
        can_hd[cid].CtrlState  = CAN_CS_STOPPED;
        can_hd[cid].HthObjBusy = 0u;
        CanIf_ControllerBusOff(cid);
    }
}

/******************************************************************************
 *  CAN XL APIs
 *****************************************************************************/

FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                P2CONST(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo)
{
    CAN_DBG_VERB(DBG_HAL, "xl_write Hth=%u", Hth);
    if (pCanHalCfg == NULL_PTR)          { return E_NOT_OK; }
    if (Hth >= pCanHalCfg->HwObjCfgNum) { return E_NOT_OK; }
    if (PduInfo == NULL_PTR)             { return E_NOT_OK; }
    if (PduInfo->sdu == NULL_PTR)        { return E_NOT_OK; }

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hth];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT) { return E_NOT_OK; }

    uint8 cid = hwObj->CanControllerRef->ControllerId;
    if (can_hd[cid].CtrlState != CAN_CS_STARTED)         { return E_NOT_OK; }
    if (hwObj->CanControllerRef->XlEnable != TRUE)        { return E_NOT_OK; }

    uint8  hth_idx = hal_hth_local_idx(Hth);
    uint32 hth_bit = (uint32)1u << hth_idx;
    if ((can_hd[cid].HthObjBusy & hth_bit) != 0u) { return CAN_BUSY_STD; }

    /* Build HCL message for XL */
    Mhcl_Can_MsgType msg;
    (void)memset(&msg, 0, sizeof(msg));

    msg.direction    = MHCL_CAN_DIR_TX;
    msg.frame_format = MHCL_CAN_FF_XL;
    msg.id_type      = MHCL_CAN_ID_BASE;
    msg.frame_id     = PduInfo->priorityId & STD_ID_MASK;

    uint16 xl_len = PduInfo->length;
    if (xl_len > (uint16)MHCL_CAN_MAX_XL_PAYLOAD_BYTE) {
        xl_len = (uint16)MHCL_CAN_MAX_XL_PAYLOAD_BYTE;
    }
    msg.dlc  = (xl_len > 0u) ? (uint32_t)(xl_len - 1u) : 0u;
    msg.vcid = PduInfo->vcid;
    msg.sdt  = PduInfo->sdt;
    msg.sec  = (PduInfo->sec == TRUE) ? 1u : 0u;

    hal_sdu_to_data_words(PduInfo->sdu, msg.data_word, (uint32)xl_len);

    int result = mhcl_can_tx_fifo_enqueue(&mhcl_ctrl[cid],
                                          (uint32_t)hwObj->HwFifoId, &msg);
    if (result == 0) {
        return CAN_BUSY_STD;
    }

    can_hd[cid].HthObjBusy |= hth_bit;
    tx_pdu_handle[cid][hth_idx] = PduInfo->swPduHandle;

    return E_OK;
}

FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_get_controller_mode(VAR(uint8, AUTOMATIC) cid,
                              P2VAR(Can_ControllerStateType, AUTOMATIC,
                                    CANXL_APPL_CONST) CtrlModePtr)
{
    return can_hal_get_controller_mode(cid, CtrlModePtr);
}

FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
               P2VAR(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo)
{
    CAN_DBG_VERB(DBG_HAL, "xl_read Hrh=%u", Hrh);
    if (pCanHalCfg == NULL_PTR)           { return E_NOT_OK; }
    if (Hrh >= pCanHalCfg->HwObjCfgNum)  { return E_NOT_OK; }
    if (PduInfo == NULL_PTR)              { return E_NOT_OK; }
    if (PduInfo->sdu == NULL_PTR)         { return E_NOT_OK; }

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hrh];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { return E_NOT_OK; }

    uint8 cid = hwObj->CanControllerRef->ControllerId;
    if (can_hd[cid].CtrlState != CAN_CS_STARTED) { return E_NOT_OK; }

    Mhcl_Can_MsgType msg;
    (void)memset(&msg, 0, sizeof(msg));

    int result = mhcl_can_rx_fifo_dequeue(&mhcl_ctrl[cid],
                                          (uint32_t)hwObj->HwFifoId, &msg);
    if (result == 0) { return E_NOT_OK; }

    if (msg.frame_format != MHCL_CAN_FF_XL) { return E_NOT_OK; }

    uint16 data_bytes = (uint16)(msg.dlc + 1u);
    if (data_bytes > (uint16)MHCL_CAN_MAX_XL_PAYLOAD_BYTE) {
        data_bytes = (uint16)MHCL_CAN_MAX_XL_PAYLOAD_BYTE;
    }

    PduInfo->priorityId  = msg.frame_id;
    PduInfo->vcid        = (uint8)msg.vcid;
    PduInfo->sdt         = (uint8)msg.sdt;
    PduInfo->length      = data_bytes;
    PduInfo->sec         = (msg.sec == 1u) ? TRUE : FALSE;
    PduInfo->swPduHandle = 0u;

    hal_data_words_to_sdu(msg.data_word, PduInfo->sdu, (uint32)data_bytes);

    return E_OK;
}

/******************************************************************************
 *  Helper functions (FD info queries)
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hth_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsFd,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) Brs,
                        P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) PaddingValue)
{
    if (pCanHalCfg == NULL_PTR)          { return E_NOT_OK; }
    if (Hth >= pCanHalCfg->HwObjCfgNum) { return E_NOT_OK; }

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hth];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_TRANSMIT) { return E_NOT_OK; }

    *IsFd = (hwObj->CanObjectPayloadLength > CAN_PL_8) ? TRUE : FALSE;

    if (*IsFd == TRUE) {
        const Can_ControllerType *ctrlCfg = hwObj->CanControllerRef;
        *Brs = ((ctrlCfg->CanControllerFdBaudrateConfig != NULL_PTR) &&
                (ctrlCfg->CanControllerFdBaudrateConfig
                     ->CanControllerTxBitRateSwitch == TRUE))
                   ? TRUE : FALSE;
    } else {
        *Brs = FALSE;
    }

    *PaddingValue = hwObj->CanFdPaddingValue;
    return E_OK;
}

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hrh_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
                        P2VAR(Can_ObjectPLType, AUTOMATIC, CAN_APPL_DATA)
                            MaxPayload)
{
    if (pCanHalCfg == NULL_PTR)          { return E_NOT_OK; }
    if (Hrh >= pCanHalCfg->HwObjCfgNum) { return E_NOT_OK; }

    const Can_HardwareObjectType *hwObj = &pCanHalCfg->CanHardwareObject[Hrh];
    if (hwObj->CanObjectType != CAN_OBJECT_TYPE_RECEIVE) { return E_NOT_OK; }

    *MaxPayload = hwObj->CanObjectPayloadLength;
    return E_OK;
}

/******************************************************************************
 *  Interrupt handler
 *****************************************************************************/

FUNC(void, CAN_CODE) can_hal_irq_handler(VAR(uint8, AUTOMATIC) cid)
{
    CAN_DBG_VERB(DBG_HAL, "irq_handler cid=%u", cid);
    if (pCanHalCfg == NULL_PTR)              { return; }
    if (cid >= pCanHalCfg->ControllerCfgNum) { return; }

    mhcl_can_process_irq_func(&mhcl_ctrl[cid]);
    mhcl_can_process_irq_err(&mhcl_ctrl[cid]);
    mhcl_can_process_irq_safety(&mhcl_ctrl[cid]);
}
