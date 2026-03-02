/*
 * XCAN Hardware Common-interface Layer (HCL) — Implementation
 *
 * Each function implements one logical operation using multiple HDL calls.
 * Manages FIFO queue state, TX descriptor building, RX descriptor parsing,
 * multi-step init/stop sequences, and interrupt processing.
 *
 * Reference:
 *   - xcand.c / xcand_mh.c / xcan_prt.c  (xcan_sw_example, Bosch)
 *   - XCAN User Manual v3.90
 *
 * DO NOT use any existing project files (mhal_can.h, can_driver.c, etc.).
 */

#include "mhcl_can.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

static uint32_t mhcl_dlc_to_bytes_internal(Mhcl_Can_FrameFormatType ff,
                                            uint32_t dlc, bool rtr)
{
    uint32_t bytes;

    switch (ff) {
    case MHCL_CAN_FF_XL:
        bytes = dlc + 1u;
        break;

    case MHCL_CAN_FF_FD:
        if (dlc <= 8u) {
            bytes = dlc;
        } else {
            switch (dlc) {
            case  9u: bytes = 12u; break;
            case 10u: bytes = 16u; break;
            case 11u: bytes = 20u; break;
            case 12u: bytes = 24u; break;
            case 13u: bytes = 32u; break;
            case 14u: bytes = 48u; break;
            case 15u: bytes = 64u; break;
            default:  bytes = 0u;  break;
            }
        }
        break;

    case MHCL_CAN_FF_CC:
    default:
        if (rtr) {
            bytes = 0u;
        } else {
            bytes = (dlc <= MHCL_CAN_CC_MAX_DATA_BYTES) ? dlc
                                                         : MHCL_CAN_CC_MAX_DATA_BYTES;
        }
        break;
    }
    return bytes;
}

static uint32_t mhcl_bytes_to_words_roundup(uint32_t bytes)
{
    if ((bytes & 0x3u) != 0u) {
        return MHCL_CAN_BYTE_TO_WORD(bytes) + 1u;
    }
    return MHCL_CAN_BYTE_TO_WORD(bytes);
}

static uint32_t mhcl_dlc_to_data_words(Mhcl_Can_FrameFormatType ff,
                                        uint32_t dlc, bool rtr)
{
    return mhcl_bytes_to_words_roundup(
        mhcl_dlc_to_bytes_internal(ff, dlc, rtr));
}

/* Build TX descriptor elem4 (T0): Frame format + ID + XL fields */
static void mhcl_build_tx_t0(const Mhcl_Can_MsgType *msg,
                              XCAND_MH_TX_W0_union *t0)
{
    t0->as_uint32 = 0u;

    if (msg->frame_format == MHCL_CAN_FF_XL) {
        t0->as_xl.FDFu1     = 1u;
        t0->as_xl.XLFu1     = 1u;
        t0->as_xl.IDPRIOu11 = msg->frame_id;
        t0->as_xl.RRSu1     = msg->rrs;
        t0->as_xl.SECu1     = msg->sec;
        t0->as_xl.VCIDu8    = msg->vcid;
        t0->as_xl.SDTu8     = msg->sdt;
    } else {
        if (msg->id_type == MHCL_CAN_ID_BASE) {
            t0->as_ccfd_id_base.IDBASEu11 = msg->frame_id;
            if (msg->frame_format == MHCL_CAN_FF_FD) {
                t0->as_ccfd_id_base.FDFu1 = 1u;
            }
        } else {
            t0->as_ccfd_id_ext.XTDu1     = 1u;
            t0->as_ccfd_id_ext.IDEXTu29  = msg->frame_id;
            if (msg->frame_format == MHCL_CAN_FF_FD) {
                t0->as_ccfd_id_ext.FDFu1 = 1u;
            }
        }
    }
}

/* Build TX descriptor elem5 (T1): DLC, BRS, ESI, RTR, FIR */
static void mhcl_build_tx_t1(const Mhcl_Can_MsgType *msg,
                              XCAND_MH_TX_W1_union *t1)
{
    t1->as_uint32 = 0u;

    if (msg->frame_format == MHCL_CAN_FF_XL) {
        t1->as_xl.DLCu11 = msg->dlc;
        t1->as_xl.FIRu1  = msg->fir;
    } else {
        t1->as_ccfd.DLCu4 = msg->dlc;
        t1->as_xl.FIRu1   = msg->fir;

        if (msg->frame_format == MHCL_CAN_FF_FD) {
            t1->as_ccfd.BRSu1 = msg->brs;
            t1->as_ccfd.ESIu1 = msg->esi;
        } else {
            t1->as_ccfd.RTRu1 = msg->rtr;
        }
    }
}

/* =========================================================================
 * Init / Deinit
 * ========================================================================= */

/*
 * Full init sequence (based on xcand_config_and_start + xcand_mh_init):
 *   1. Compute sub-module base addresses
 *   2. LMEM zero-init
 *   3. MH global config
 *   4. TX FIFO setup (invalidate descriptors, config registers, enable)
 *   5. TX PQ setup
 *   6. RX FIFO setup (validate descriptors, config registers, enable)
 *   7. RX filter config (write elements + reference pairs to LMEM)
 *   8. IRC config
 *   9. MH start
 *  10. RX FIFO start
 *  11. PRT config (bit timing + mode)
 *  12. PRT start
 */
Mhcl_Can_ReturnType mhcl_can_init(Mhcl_Can_ControllerType *ctrl,
                                    const Mhcl_Can_ConfigType *config)
{
    uint32_t i, j;

    /* --- Step 1: Compute sub-module base addresses --- */
    ctrl->mh_base   = config->xcan_base_addr + XCAN_MH_OFFSET;
    ctrl->prt_base  = config->xcan_base_addr + XCAN_PRT_OFFSET;
    ctrl->irc_base  = config->xcan_base_addr + XCAN_IRC_OFFSET;
    ctrl->lmem_base = config->lmem_base;
    ctrl->instance_id = config->instance_id;
    ctrl->rx_continuous_mode = config->rx_continuous_mode;

    ctrl->cb_tx_confirmation = NULL;
    ctrl->cb_rx_indication   = NULL;
    ctrl->cb_busoff          = NULL;
    ctrl->cb_error_passive   = NULL;
    ctrl->cb_error_active    = NULL;

    /* --- Step 2: LMEM zero-init --- */
    for (i = 0u; i < config->lmem_size_words; i++) {
        mhdl_can_mh_write_lmem(ctrl->lmem_base, i * 4u, 0u);
    }

    /* --- Step 3: MH global config --- */
    mhdl_can_mh_set_global_cfg(ctrl->mh_base, config->instance_id,
                                config->retrans_max,
                                config->rx_continuous_mode ? 1u : 0u);
    mhdl_can_mh_set_axi_params(ctrl->mh_base, 0u, 0u);
    mhdl_can_mh_set_mem_addresses(ctrl->mh_base,
                                   config->lmem_fq_base,
                                   config->lmem_pq_base,
                                   config->lmem_rx_filter_base);

    /* --- Step 4: TX FIFO setup --- */
    for (i = 0u; i < MHCL_CAN_MAX_TX_FIFO; i++) {
        const Mhcl_Can_TxFifoCfgType *tcfg = &config->tx_fifo[i];
        Mhcl_Can_TxFifoState *ts = &ctrl->tx_fifo[i];

        ts->enabled = tcfg->enabled;
        if (!tcfg->enabled) {
            continue;
        }

        ts->fifo_size       = tcfg->fifo_size;
        ts->dc_size_word    = tcfg->dc_size_word;
        ts->desc_array_addr = tcfg->desc_array_addr;
        ts->dc_start_addr   = tcfg->dc_start_addr;
        ts->put_index       = 0u;
        ts->rolling_counter = 0u;

        /* Invalidate all TX descriptors (VALID=0) */
        volatile XCAND_MH_TX_DESC_struct *desc_arr =
            (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)ts->desc_array_addr;
        for (j = 0u; j < ts->fifo_size; j++) {
            desc_arr[j].elem0.as_uint32 = 0u;
        }

        mhdl_can_mh_set_tx_fifo_config(ctrl->mh_base, i,
                                        ts->desc_array_addr, ts->fifo_size);
        mhdl_can_mh_tx_fifo_enable(ctrl->mh_base, i);
    }

    /* --- Step 5: TX PQ setup --- */
    ctrl->tx_pq_num_slots      = config->tx_pq.num_slots;
    ctrl->tx_pq_desc_array_addr = config->tx_pq.desc_array_addr;

    if (config->tx_pq.num_slots > 0u) {
        uint32_t enable_mask = 0u;

        /* Invalidate all PQ descriptors */
        volatile XCAND_MH_TX_DESC_struct *pq_desc_arr =
            (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)config->tx_pq.desc_array_addr;
        for (i = 0u; i < config->tx_pq.num_slots; i++) {
            pq_desc_arr[i].elem0.as_uint32 = 0u;
        }

        for (i = 0u; i < config->tx_pq.num_slots; i++) {
            ctrl->tx_pq_slot[i].enabled       = config->tx_pq.slot[i].enabled;
            ctrl->tx_pq_slot[i].dc_size_word  = config->tx_pq.slot[i].dc_size_word;
            ctrl->tx_pq_slot[i].dc_start_addr = config->tx_pq.slot[i].dc_start_addr;

            if (config->tx_pq.slot[i].enabled) {
                enable_mask |= MHDL_BIT(i);
            }
        }

        mhdl_can_mh_set_tx_pq_config(ctrl->mh_base,
                                       config->tx_pq.desc_array_addr,
                                       enable_mask);
    }

    /* --- Step 6: RX FIFO setup --- */
    for (i = 0u; i < MHCL_CAN_MAX_RX_FIFO; i++) {
        const Mhcl_Can_RxFifoCfgType *rcfg = &config->rx_fifo[i];
        Mhcl_Can_RxFifoState *rs = &ctrl->rx_fifo[i];

        rs->enabled = rcfg->enabled;
        if (!rcfg->enabled) {
            continue;
        }

        rs->fifo_size       = rcfg->fifo_size;
        rs->dc_size_word    = rcfg->dc_size_word;
        rs->desc_array_addr = rcfg->desc_array_addr;
        rs->dc_start_addr   = rcfg->dc_start_addr;
        rs->get_index       = 0u;

        /* Initialize RX descriptors: VALID=0 (meaning "available for MH"),
         * set FQN, IN, RC, and elem1_rx_ap for normal mode.
         * (Per XCAN convention: RX VALID=0 means descriptor is available) */
        volatile XCAND_MH_RX_DESC_struct *rx_desc_arr =
            (volatile XCAND_MH_RX_DESC_struct *)(uintptr_t)rs->desc_array_addr;

        for (j = 0u; j < rs->fifo_size; j++) {
            XCAND_MH_RX_DESC_ELEM0_union elem0;
            elem0.as_uint32 = 0u;
            elem0.as_RX_FIFO_Queue.VALIDu1 = 0u;
            elem0.as_RX_FIFO_Queue.FQNu4   = (uint32_t)i;
            elem0.as_RX_FIFO_Queue.INu3    = config->instance_id;
            elem0.as_RX_FIFO_Queue.RCu5    = j & MHCL_CAN_RC_MASK;

            rx_desc_arr[j].elem0.as_uint32 = elem0.as_uint32;

            if (!config->rx_continuous_mode) {
                rx_desc_arr[j].elem1_rx_ap =
                    rs->dc_start_addr +
                    j * MHCL_CAN_WORD_TO_BYTE(rs->dc_size_word);
            } else {
                rx_desc_arr[j].elem1_rx_ap = 0u;
            }
        }

        /* RC init for get_index tracking: last RC was (fifo_size-1),
         * so next expected = fifo_size (masked) */
        rs->rolling_counter = rs->fifo_size & MHCL_CAN_RC_MASK;

        mhdl_can_mh_set_rx_fifo_config(ctrl->mh_base, i,
                                        rs->desc_array_addr,
                                        rs->fifo_size,
                                        rs->dc_size_word,
                                        rs->dc_start_addr,
                                        config->rx_continuous_mode);
        mhdl_can_mh_rx_fifo_enable(ctrl->mh_base, i);
    }

    /* --- Step 7: RX filter config --- */
    if (config->rx_filter.num_elements > 0u) {
        mhdl_can_mh_set_rx_filter_ctrl(ctrl->mh_base,
                                        config->rx_filter.num_elements,
                                        config->rx_filter.accept_non_matching ? 1u : 0u,
                                        config->rx_filter.accept_non_filtered ? 1u : 0u,
                                        config->rx_filter.non_matching_fifo,
                                        config->rx_filter.non_filtered_threshold);

        /* Write filter elements to LMEM */
        for (i = 0u; i < config->rx_filter.num_elements; i++) {
            const Mhcl_Can_RxFilterElementType *fe = &config->rx_filter.elements[i];
            XCAND_MH_RX_FILTER_ELEMENT_UNION feu;
            feu.as_uint32 = 0u;
            feu.as_s.CREFI0u8 = fe->compare0_ref_index;
            feu.as_s.WI0u2    = fe->compare0_word_index;
            feu.as_s.AR0u1    = fe->compare0_reject_on_match ? 1u : 0u;
            feu.as_s.CREFI1u8 = fe->compare1_ref_index;
            feu.as_s.WI1u2    = fe->compare1_word_index;
            feu.as_s.AR1u1    = fe->compare1_reject_on_match ? 1u : 0u;
            feu.as_s.BLKu1    = fe->blacklist ? 1u : 0u;
            feu.as_s.IRQu1    = fe->interrupt_enable ? 1u : 0u;
            feu.as_s.FIFOu4   = fe->default_rx_fifo;

            mhdl_can_mh_write_lmem(ctrl->lmem_base,
                                    config->lmem_rx_filter_base * 4u + i * 4u,
                                    feu.as_uint32);
        }

        /* Write reference pairs (value + mask) to LMEM after filter elements */
        uint32_t ref_base_offset =
            config->lmem_rx_filter_base * 4u +
            config->rx_filter.num_elements * 4u;
        for (i = 0u; i < config->rx_filter.num_ref_pairs; i++) {
            mhdl_can_mh_write_lmem(ctrl->lmem_base,
                                    ref_base_offset + i * 8u,
                                    config->rx_filter.ref_pairs[i].value);
            mhdl_can_mh_write_lmem(ctrl->lmem_base,
                                    ref_base_offset + i * 8u + 4u,
                                    config->rx_filter.ref_pairs[i].mask);
        }
    }

    /* --- Step 8: IRC config --- */
    mhdl_can_irc_set_func_ena(ctrl->irc_base, config->irc.func_ena_mask);
    mhdl_can_irc_set_err_ena(ctrl->irc_base, config->irc.err_ena_mask);
    mhdl_can_irc_set_safety_ena(ctrl->irc_base, config->irc.safety_ena_mask);

    /* --- Step 9: MH start --- */
    mhdl_can_mh_start(ctrl->mh_base);

    /* --- Step 10: RX FIFO start --- */
    for (i = 0u; i < MHCL_CAN_MAX_RX_FIFO; i++) {
        if (ctrl->rx_fifo[i].enabled) {
            mhdl_can_mh_rx_fifo_start(ctrl->mh_base, i);
        }
    }

    /* --- Step 11: PRT config (bit timing + mode) --- */
    {
        uint32_t tseg1_nom = config->nominal.prop_seg + config->nominal.phase_seg1 - 1u;
        uint32_t tseg2_nom = config->nominal.phase_seg2 - 1u;
        uint32_t sjw_nom   = config->nominal.sjw - 1u;
        uint32_t brp_nom   = config->brp - 1u;

        mhdl_can_prt_set_nbtp(ctrl->prt_base, brp_nom, tseg1_nom,
                               tseg2_nom, sjw_nom);
    }

    if (config->prt_mode.fd_ena) {
        uint32_t tseg1_fd = config->data_fd.prop_seg + config->data_fd.phase_seg1 - 1u;
        uint32_t tseg2_fd = config->data_fd.phase_seg2 - 1u;
        uint32_t sjw_fd   = config->data_fd.sjw - 1u;
        uint32_t tdco_fd  = config->data_fd.tdc_offset;

        mhdl_can_prt_set_dbtp(ctrl->prt_base, tseg1_fd, tseg2_fd,
                               sjw_fd, tdco_fd);
    }

    if (config->prt_mode.xl_ena) {
        uint32_t tseg1_xl = config->data_xl.prop_seg + config->data_xl.phase_seg1 - 1u;
        uint32_t tseg2_xl = config->data_xl.phase_seg2 - 1u;
        uint32_t sjw_xl   = config->data_xl.sjw - 1u;
        uint32_t tdco_xl  = config->data_xl.tdc_offset;

        mhdl_can_prt_set_xbtp(ctrl->prt_base, tseg1_xl, tseg2_xl,
                               sjw_xl, tdco_xl);
    }

    if (config->prt_mode.xl_tc_mode_switching_ena) {
        mhdl_can_prt_set_pwme(ctrl->prt_base,
                               config->pwme.pwms - 1u,
                               config->pwme.pwml - 1u,
                               config->pwme.pwmo);
    }

    mhdl_can_prt_set_mode(ctrl->prt_base,
                           config->prt_mode.fd_ena ? 1u : 0u,
                           config->prt_mode.xl_ena ? 1u : 0u,
                           config->prt_mode.tdc_ena ? 1u : 0u,
                           config->prt_mode.pxh_disable ? 1u : 0u,
                           0u,  /* efbi: not exposed in HCL config */
                           config->prt_mode.tx_pause ? 1u : 0u,
                           0u,  /* mon: monitor mode — off by default */
                           0u,  /* rstr: restricted mode — off by default */
                           config->prt_mode.timestamp_sof ? 1u : 0u,
                           config->prt_mode.xl_tc_mode_switching_ena ? 1u : 0u,
                           config->prt_mode.error_signaling_disable ? 1u : 0u,
                           config->prt_mode.fault_inject_ena ? 1u : 0u);

    /* --- Step 12: PRT start --- */
    mhdl_can_prt_start(ctrl->prt_base);

    ctrl->initialized = true;
    return MHCL_CAN_OK;
}

Mhcl_Can_ReturnType mhcl_can_deinit(Mhcl_Can_ControllerType *ctrl)
{
    uint32_t i;

    /* Step 1: PRT stop (normal) */
    mhdl_can_prt_stop(ctrl->prt_base, false);

    /* Wait until PRT is actually stopped */
    while (mhdl_can_prt_is_started(ctrl->prt_base)) {
        /* busy-wait; TODO: add timeout in production */
    }

    /* Step 2: Abort all TX PQ slots */
    if (ctrl->tx_pq_num_slots > 0u) {
        for (i = 0u; i < (uint32_t)ctrl->tx_pq_num_slots; i++) {
            if (ctrl->tx_pq_slot[i].enabled) {
                mhdl_can_mh_tx_pq_abort(ctrl->mh_base, i);
            }
        }
    }

    /* Step 3: Abort all TX FIFOs */
    for (i = 0u; i < MHCL_CAN_MAX_TX_FIFO; i++) {
        if (ctrl->tx_fifo[i].enabled) {
            mhdl_can_mh_tx_fifo_abort(ctrl->mh_base, i);
        }
    }

    /* Step 4: Abort all RX FIFOs */
    for (i = 0u; i < MHCL_CAN_MAX_RX_FIFO; i++) {
        if (ctrl->rx_fifo[i].enabled) {
            mhdl_can_mh_rx_fifo_abort(ctrl->mh_base, i);
        }
    }

    /* Step 5: MH stop */
    mhdl_can_mh_stop(ctrl->mh_base);

    ctrl->initialized = false;
    return MHCL_CAN_OK;
}

/* =========================================================================
 * Mode Control
 * ========================================================================= */

Mhcl_Can_ReturnType mhcl_can_start(Mhcl_Can_ControllerType *ctrl)
{
    mhdl_can_prt_start(ctrl->prt_base);

    /* Wait for bus integration (ACT != INACTIVE) */
    while (!mhdl_can_prt_is_integrated(ctrl->prt_base)) {
        /* busy-wait; TODO: add timeout in production */
    }

    return MHCL_CAN_OK;
}

Mhcl_Can_ReturnType mhcl_can_stop(Mhcl_Can_ControllerType *ctrl)
{
    uint32_t i;

    /* PRT stop (normal) */
    mhdl_can_prt_stop(ctrl->prt_base, false);
    while (mhdl_can_prt_is_started(ctrl->prt_base)) {
        /* busy-wait; TODO: add timeout in production */
    }

    /* Abort all TX PQ slots */
    if (ctrl->tx_pq_num_slots > 0u) {
        for (i = 0u; i < (uint32_t)ctrl->tx_pq_num_slots; i++) {
            if (ctrl->tx_pq_slot[i].enabled) {
                mhdl_can_mh_tx_pq_abort(ctrl->mh_base, i);
            }
        }
    }

    /* Abort all TX FIFOs */
    for (i = 0u; i < MHCL_CAN_MAX_TX_FIFO; i++) {
        if (ctrl->tx_fifo[i].enabled) {
            mhdl_can_mh_tx_fifo_abort(ctrl->mh_base, i);
        }
    }

    /* Abort all RX FIFOs */
    for (i = 0u; i < MHCL_CAN_MAX_RX_FIFO; i++) {
        if (ctrl->rx_fifo[i].enabled) {
            mhdl_can_mh_rx_fifo_abort(ctrl->mh_base, i);
        }
    }

    /* MH stop */
    mhdl_can_mh_stop(ctrl->mh_base);

    return MHCL_CAN_OK;
}

bool mhcl_can_is_started(const Mhcl_Can_ControllerType *ctrl)
{
    return mhdl_can_prt_is_started(ctrl->prt_base);
}

/* =========================================================================
 * TX Path — FIFO Enqueue
 *
 * Based on xcand_mh_tx_fifo_enqueue_msg():
 *   1. Check full (VALID bit of put-index descriptor)
 *   2. Build descriptor elements 0-7
 *   3. Write payload to Data Container
 *   4. Write elem0 LAST (atomically validates the descriptor)
 *   5. Start FIFO
 *   6. Update put_index + rolling_counter
 * ========================================================================= */

int mhcl_can_tx_fifo_enqueue(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id,
                              const Mhcl_Can_MsgType *msg)
{
    Mhcl_Can_TxFifoState *ts = &ctrl->tx_fifo[fifo_id];

    /* Step 1: Check if full */
    if (mhcl_can_tx_fifo_is_full(ctrl, fifo_id)) {
        return 0;
    }

    uint32_t data_words = mhcl_dlc_to_data_words(msg->frame_format,
                                                   msg->dlc, msg->rtr);

    volatile XCAND_MH_TX_DESC_struct *desc_arr =
        (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)ts->desc_array_addr;
    volatile XCAND_MH_TX_DESC_struct *tx_desc = &desc_arr[ts->put_index];

    /* --- ELEM0 (prepare locally, write last) --- */
    XCAND_MH_TX_DESC_ELEM0_union elem0_local;
    elem0_local.as_uint32 = 0u;
    elem0_local.as_TX_FIFO_Queue.VALIDu1 = 1u;
    elem0_local.as_TX_FIFO_Queue.HDu1    = 1u;
    elem0_local.as_TX_FIFO_Queue.FQNu4   = fifo_id;
    elem0_local.as_TX_FIFO_Queue.RCu5    = ts->rolling_counter;

    /* --- ELEM1 --- */
    XCAND_MH_TX_DESC_ELEM1_union elem1_local;
    elem1_local.as_uint32 = 0u;

    if ((msg->frame_format == MHCL_CAN_FF_XL) ||
        ((msg->frame_format == MHCL_CAN_FF_FD) && (data_words > 1u))) {
        elem1_local.as_TX_FIFO_Queue.PLSRCu1 = 1u;
    }
    elem1_local.as_TX_FIFO_Queue.SIZEu10 = data_words;
    elem1_local.as_TX_FIFO_Queue.INu3    = ctrl->instance_id;
    elem1_local.as_TX_FIFO_Queue.NHDOu10 = 1u;

    tx_desc->elem1.as_uint32 = elem1_local.as_uint32;

    /* ELEM2 / ELEM3: timestamps — written by MH on TX completion */

    /* --- ELEM4 (T0) --- */
    XCAND_MH_TX_W0_union t0;
    mhcl_build_tx_t0(msg, &t0);
    tx_desc->elem4_T0.as_uint32 = t0.as_uint32;

    /* --- ELEM5 (T1) --- */
    XCAND_MH_TX_W1_union t1;
    mhcl_build_tx_t1(msg, &t1);
    tx_desc->elem5_T1.as_uint32 = t1.as_uint32;

    /* --- ELEM6: XL -> AF, CC/FD -> TD0 --- */
    if (msg->frame_format == MHCL_CAN_FF_XL) {
        tx_desc->elem6.as_xl.T2 = msg->af;
    } else {
        tx_desc->elem6.as_ccfd.TD0 = msg->data_word[0];
    }

    /* --- ELEM7: CC -> TD1, FD/XL -> Address Pointer to Data Container --- */
    uint32_t dc_sa = 0u;
    if (msg->frame_format == MHCL_CAN_FF_CC) {
        tx_desc->elem7.as_cc.TD1 = msg->data_word[1];
    } else {
        dc_sa = ts->dc_start_addr +
                (MHCL_CAN_WORD_TO_BYTE(ts->dc_size_word) * ts->put_index);
        tx_desc->elem7.as_fdxl.TX_AP = dc_sa;
    }

    /* --- Write payload to Data Container --- */
    if ((msg->frame_format == MHCL_CAN_FF_XL) ||
        ((msg->frame_format == MHCL_CAN_FF_FD) && (data_words > 0u))) {
        for (uint32_t w = 0u; w < data_words; w++) {
            mhdl_can_mh_write_lmem(dc_sa, w * 4u, msg->data_word[w]);
        }
    }

    /* --- Step 4: Write ELEM0 last to validate the descriptor --- */
    tx_desc->elem0.as_uint32 = elem0_local.as_uint32;

    /* --- Step 5: Start TX FIFO --- */
    mhdl_can_mh_tx_fifo_start(ctrl->mh_base, fifo_id);

    /* --- Step 6: Update put_index and rolling_counter --- */
    ts->put_index++;
    if (ts->put_index >= ts->fifo_size) {
        ts->put_index = 0u;
    }
    ts->rolling_counter++;
    ts->rolling_counter &= MHCL_CAN_RC_MASK;

    return 1;
}

/* =========================================================================
 * TX Path — PQ Enqueue
 *
 * Based on xcand_mh_tx_priority_queue_enqueue_msg():
 *   1. Check busy (register-based)
 *   2. Build descriptor
 *   3. Write payload
 *   4. Validate
 *   5. Start slot
 * ========================================================================= */

int mhcl_can_tx_pq_enqueue(Mhcl_Can_ControllerType *ctrl, uint32_t slot_id,
                             const Mhcl_Can_MsgType *msg)
{
    /* Step 1: Check busy */
    if (mhcl_can_tx_pq_is_busy(ctrl, slot_id)) {
        return 0;
    }

    uint32_t data_words = mhcl_dlc_to_data_words(msg->frame_format,
                                                   msg->dlc, msg->rtr);

    volatile XCAND_MH_TX_DESC_struct *pq_desc_arr =
        (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)ctrl->tx_pq_desc_array_addr;
    volatile XCAND_MH_TX_DESC_struct *tx_desc = &pq_desc_arr[slot_id];

    /* --- ELEM0 (PQ variant) --- */
    XCAND_MH_TX_DESC_ELEM0_union elem0_local;
    elem0_local.as_uint32 = 0u;
    elem0_local.as_TX_PRIO_Queue.VALIDu1 = 1u;
    elem0_local.as_TX_PRIO_Queue.HDu1    = 1u;
    elem0_local.as_TX_PRIO_Queue.PQu1    = 1u;
    elem0_local.as_TX_PRIO_Queue.PQSNu5  = slot_id;

    /* --- ELEM1 (PQ variant) --- */
    XCAND_MH_TX_DESC_ELEM1_union elem1_local;
    elem1_local.as_uint32 = 0u;

    if ((msg->frame_format == MHCL_CAN_FF_XL) ||
        ((msg->frame_format == MHCL_CAN_FF_FD) && (data_words > 1u))) {
        elem1_local.as_TX_PRIO_Queue.PLSRCu1 = 1u;
    }
    elem1_local.as_TX_PRIO_Queue.SIZEu10 = data_words;
    elem1_local.as_TX_PRIO_Queue.INu3    = ctrl->instance_id;

    tx_desc->elem1.as_uint32 = elem1_local.as_uint32;

    /* ELEM4 (T0) */
    XCAND_MH_TX_W0_union t0;
    mhcl_build_tx_t0(msg, &t0);
    tx_desc->elem4_T0.as_uint32 = t0.as_uint32;

    /* ELEM5 (T1) */
    XCAND_MH_TX_W1_union t1;
    mhcl_build_tx_t1(msg, &t1);
    tx_desc->elem5_T1.as_uint32 = t1.as_uint32;

    /* ELEM6 */
    if (msg->frame_format == MHCL_CAN_FF_XL) {
        tx_desc->elem6.as_xl.T2 = msg->af;
    } else {
        tx_desc->elem6.as_ccfd.TD0 = msg->data_word[0];
    }

    /* ELEM7 */
    uint32_t dc_sa = 0u;
    if (msg->frame_format == MHCL_CAN_FF_CC) {
        tx_desc->elem7.as_cc.TD1 = msg->data_word[1];
    } else {
        dc_sa = ctrl->tx_pq_slot[slot_id].dc_start_addr;
        tx_desc->elem7.as_fdxl.TX_AP = dc_sa;
    }

    /* Write payload to Data Container */
    if ((msg->frame_format == MHCL_CAN_FF_XL) ||
        ((msg->frame_format == MHCL_CAN_FF_FD) && (data_words > 0u))) {
        for (uint32_t w = 0u; w < data_words; w++) {
            mhdl_can_mh_write_lmem(dc_sa, w * 4u, msg->data_word[w]);
        }
    }

    /* Validate descriptor */
    tx_desc->elem0.as_uint32 = elem0_local.as_uint32;

    /* Start PQ slot */
    mhdl_can_mh_tx_pq_start(ctrl->mh_base, slot_id);

    return 1;
}

bool mhcl_can_tx_fifo_is_full(const Mhcl_Can_ControllerType *ctrl,
                               uint32_t fifo_id)
{
    const Mhcl_Can_TxFifoState *ts = &ctrl->tx_fifo[fifo_id];
    volatile XCAND_MH_TX_DESC_struct *desc_arr =
        (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)ts->desc_array_addr;

    return (desc_arr[ts->put_index].elem0.as_TX_FIFO_Queue.VALIDu1 == 1u);
}

bool mhcl_can_tx_pq_is_busy(const Mhcl_Can_ControllerType *ctrl,
                              uint32_t slot_id)
{
    uint32_t sts0 = mhdl_can_mh_tx_pq_get_sts0(ctrl->mh_base);
    return ((sts0 & MHDL_BIT(slot_id)) != 0u);
}

void mhcl_can_tx_fifo_abort(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id)
{
    mhdl_can_mh_tx_fifo_abort(ctrl->mh_base, fifo_id);
    ctrl->tx_fifo[fifo_id].enabled = false;
}

/* =========================================================================
 * RX Path — FIFO Dequeue
 *
 * Based on xcand_mh_rx_fifo_dequeue_msg():
 *   1. Check empty (VALID bit == 0 means descriptor available = no msg)
 *      (VALID bit == 1 means descriptor has been invalidated by MH after write)
 *      NOTE: In XCAN, RX VALID=0 = descriptor available for MH to write;
 *            VALID=1 = MH has written (invalidated for SW to read).
 *      Actually, per the example: if VALID==0, RX FIFO is empty (no msg).
 *   2. Read R0/R1 from Data Container -> parse frame format, ID, DLC
 *   3. Read payload from Data Container
 *   4. Update RX descriptor (re-validate for MH reuse)
 *   5. Start FIFO
 *   6. Update get_index + rolling_counter
 * ========================================================================= */

int mhcl_can_rx_fifo_dequeue(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id,
                              Mhcl_Can_MsgType *msg)
{
    Mhcl_Can_RxFifoState *rs = &ctrl->rx_fifo[fifo_id];

    /* Step 1: Check empty */
    if (mhcl_can_rx_fifo_is_empty(ctrl, fifo_id)) {
        return 0;
    }

    /* Clear output structure */
    memset(msg, 0, sizeof(Mhcl_Can_MsgType));

    volatile XCAND_MH_RX_DESC_struct *rx_desc_arr =
        (volatile XCAND_MH_RX_DESC_struct *)(uintptr_t)rs->desc_array_addr;
    volatile XCAND_MH_RX_DESC_struct *rx_desc = &rx_desc_arr[rs->get_index];

    msg->direction       = MHCL_CAN_DIR_RX;
    msg->rx_fifo_number  = fifo_id;

    /* ELEM0: RX status */
    msg->rx_status = rx_desc->elem0.as_RX_FIFO_Queue.STSu4;

    /* ELEM1: Data container address pointer */
    uint32_t dc_sa = rx_desc->elem1_rx_ap;

    /* ELEM2 / ELEM3: Timestamps */
    msg->rx_timestamp_lsw = rx_desc->elem2_ts_lsw;
    msg->rx_timestamp_msw = rx_desc->elem3_ts_msw;

    /* --- Read R0 and R1 from Data Container --- */
    XCAND_MH_RX_R0_union r0;
    XCAND_MH_RX_R1_union r1;
    r0.as_uint32 = mhdl_can_mh_read_lmem(dc_sa, 0u);
    r1.as_uint32 = mhdl_can_mh_read_lmem(dc_sa, 4u);

    /* --- Decode R0: frame format + ID --- */
    if (r0.as_xl.FDFu1 == 1u) {
        if (r0.as_xl.XLFu1 == 1u) {
            msg->frame_format = MHCL_CAN_FF_XL;
        } else {
            msg->frame_format = MHCL_CAN_FF_FD;
        }
    } else {
        msg->frame_format = MHCL_CAN_FF_CC;
    }

    if (msg->frame_format == MHCL_CAN_FF_XL) {
        msg->id_type  = MHCL_CAN_ID_BASE;
        msg->frame_id = r0.as_xl.IDPRIOu11;
        msg->sdt      = r0.as_xl.SDTu8;
        msg->vcid     = r0.as_xl.VCIDu8;
        msg->sec      = r0.as_xl.SECu1;
        msg->rrs      = r0.as_xl.RRSu1;
    } else {
        if (r0.as_ccfd_id_base.XTDu1 == 0u) {
            msg->id_type  = MHCL_CAN_ID_BASE;
            msg->frame_id = r0.as_ccfd_id_base.IDBASEu11;
        } else {
            msg->id_type  = MHCL_CAN_ID_EXTENDED;
            msg->frame_id = r0.as_ccfd_id_ext.IDEXTu29;
        }
    }

    /* --- Decode R1: DLC, BRS, ESI, RTR, filtering info --- */
    if (msg->frame_format == MHCL_CAN_FF_XL) {
        msg->dlc = r1.as_xl.DLCu11;
    } else {
        msg->dlc = r1.as_ccfd.DLCu4;
        msg->esi = r1.as_ccfd.ESIu1;
        msg->brs = r1.as_ccfd.BRSu1;
        msg->rtr = r1.as_ccfd.RTRu1;
    }

    /* Filtering metadata (same bit positions for CC/FD/XL in R1) */
    msg->rx_filter_fab  = (r1.as_ccfd.FABu1 == 1u) ? 1u : 0u;
    msg->rx_filter_blk  = (r1.as_ccfd.BLKu1 == 1u) ? 1u : 0u;
    msg->rx_filter_fm   = (r1.as_ccfd.FMu1  == 1u) ? 1u : 0u;
    msg->rx_filter_fidx = (uint8_t)r1.as_ccfd.FIDXu8;

    /* --- Read payload from Data Container --- */
    uint32_t data_field_start;
    if (msg->frame_format == MHCL_CAN_FF_XL) {
        /* AF is at word 2 in DC */
        msg->af = mhdl_can_mh_read_lmem(dc_sa, 2u * 4u);
        data_field_start = dc_sa + 3u * 4u; /* after R0, R1, AF */
    } else {
        data_field_start = dc_sa + 2u * 4u; /* after R0, R1 */
    }

    uint32_t data_words = mhcl_dlc_to_data_words(msg->frame_format,
                                                   msg->dlc, msg->rtr);
    for (uint32_t w = 0u; w < data_words; w++) {
        msg->data_word[w] = mhdl_can_mh_read_lmem(data_field_start, w * 4u);
    }

    /* --- Continuous mode: update read address pointer --- */
    if (ctrl->rx_continuous_mode) {
        uint32_t reg_block_offset = MHDL_MH_RX_FIFO_CFG_REG_BLOCK * fifo_id;
        uint32_t last_word_addr = data_field_start +
                                  MHCL_CAN_WORD_TO_BYTE(data_words - 1u);
        mhdl_can_reg_write_verify(ctrl->mh_base,
            XCAND_MH_CREG_RX_FQ_RD_ADD_PT0 + reg_block_offset,
            last_word_addr);
    }

    /* --- Step 4: Re-validate RX descriptor for MH reuse --- */
    XCAND_MH_RX_DESC_ELEM0_union elem0_new;
    elem0_new.as_uint32 = 0u;
    elem0_new.as_RX_FIFO_Queue.VALIDu1 = 0u;  /* 0 = available for MH */
    elem0_new.as_RX_FIFO_Queue.FQNu4   = fifo_id;
    elem0_new.as_RX_FIFO_Queue.INu3    = ctrl->instance_id;
    elem0_new.as_RX_FIFO_Queue.RCu5    = rs->rolling_counter;

    rx_desc->elem0.as_uint32 = elem0_new.as_uint32;

    /* --- Step 5: Restart RX FIFO --- */
    mhdl_can_mh_rx_fifo_start(ctrl->mh_base, fifo_id);

    /* --- Step 6: Update get_index + rolling_counter --- */
    rs->get_index++;
    if (rs->get_index >= rs->fifo_size) {
        rs->get_index = 0u;
    }
    rs->rolling_counter++;
    rs->rolling_counter &= MHCL_CAN_RC_MASK;

    return 1;
}

bool mhcl_can_rx_fifo_is_empty(const Mhcl_Can_ControllerType *ctrl,
                                uint32_t fifo_id)
{
    const Mhcl_Can_RxFifoState *rs = &ctrl->rx_fifo[fifo_id];
    volatile XCAND_MH_RX_DESC_struct *rx_desc_arr =
        (volatile XCAND_MH_RX_DESC_struct *)(uintptr_t)rs->desc_array_addr;

    /* XCAN RX convention: VALID=0 means "no received message" (descriptor
     * available for MH). VALID=1 means MH has invalidated it (msg present). */
    return (rx_desc_arr[rs->get_index].elem0.as_RX_FIFO_Queue.VALIDu1 == 0u);
}

void mhcl_can_rx_fifo_abort(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id)
{
    mhdl_can_mh_rx_fifo_abort(ctrl->mh_base, fifo_id);
    ctrl->rx_fifo[fifo_id].enabled = false;
}

/* =========================================================================
 * Baudrate
 * ========================================================================= */

Mhcl_Can_ReturnType mhcl_can_set_baudrate(Mhcl_Can_ControllerType *ctrl,
                                            uint16_t brp,
                                            const Mhcl_Can_BitTimingType *nominal,
                                            const Mhcl_Can_BitTimingType *data_fd,
                                            const Mhcl_Can_BitTimingType *data_xl,
                                            const Mhcl_Can_PwmeType *pwme,
                                            const Mhcl_Can_PrtModeType *mode)
{
    bool was_started = mhdl_can_prt_is_started(ctrl->prt_base);

    if (was_started) {
        mhdl_can_prt_stop(ctrl->prt_base, false);
        while (mhdl_can_prt_is_started(ctrl->prt_base)) {
            /* busy-wait */
        }
    }

    /* Nominal */
    {
        uint32_t tseg1 = nominal->prop_seg + nominal->phase_seg1 - 1u;
        uint32_t tseg2 = nominal->phase_seg2 - 1u;
        uint32_t sjw   = nominal->sjw - 1u;
        mhdl_can_prt_set_nbtp(ctrl->prt_base, (uint32_t)brp - 1u,
                               tseg1, tseg2, sjw);
    }

    /* FD Data */
    if (data_fd != NULL) {
        uint32_t tseg1 = data_fd->prop_seg + data_fd->phase_seg1 - 1u;
        uint32_t tseg2 = data_fd->phase_seg2 - 1u;
        uint32_t sjw   = data_fd->sjw - 1u;
        mhdl_can_prt_set_dbtp(ctrl->prt_base, tseg1, tseg2, sjw,
                               data_fd->tdc_offset);
    }

    /* XL Data */
    if (data_xl != NULL) {
        uint32_t tseg1 = data_xl->prop_seg + data_xl->phase_seg1 - 1u;
        uint32_t tseg2 = data_xl->phase_seg2 - 1u;
        uint32_t sjw   = data_xl->sjw - 1u;
        mhdl_can_prt_set_xbtp(ctrl->prt_base, tseg1, tseg2, sjw,
                               data_xl->tdc_offset);
    }

    /* PWME */
    if (pwme != NULL) {
        mhdl_can_prt_set_pwme(ctrl->prt_base,
                               pwme->pwms - 1u, pwme->pwml - 1u, pwme->pwmo);
    }

    /* MODE */
    if (mode != NULL) {
        mhdl_can_prt_set_mode(ctrl->prt_base,
                               mode->fd_ena ? 1u : 0u,
                               mode->xl_ena ? 1u : 0u,
                               mode->tdc_ena ? 1u : 0u,
                               mode->pxh_disable ? 1u : 0u,
                               0u,
                               mode->tx_pause ? 1u : 0u,
                               0u, 0u,
                               mode->timestamp_sof ? 1u : 0u,
                               mode->xl_tc_mode_switching_ena ? 1u : 0u,
                               mode->error_signaling_disable ? 1u : 0u,
                               mode->fault_inject_ena ? 1u : 0u);
    }

    if (was_started) {
        mhdl_can_prt_start(ctrl->prt_base);
    }

    return MHCL_CAN_OK;
}

/* =========================================================================
 * Error / Status
 * ========================================================================= */

Mhcl_Can_ErrorStateType mhcl_can_get_error_state(
    const Mhcl_Can_ControllerType *ctrl)
{
    if (mhdl_can_prt_is_busoff(ctrl->prt_base)) {
        return MHCL_CAN_ERR_BUSOFF;
    }
    if (mhdl_can_prt_is_error_passive(ctrl->prt_base)) {
        return MHCL_CAN_ERR_PASSIVE;
    }
    return MHCL_CAN_ERR_ACTIVE;
}

uint32_t mhcl_can_get_tec(const Mhcl_Can_ControllerType *ctrl)
{
    return mhdl_can_prt_get_tec(ctrl->prt_base);
}

uint32_t mhcl_can_get_rec(const Mhcl_Can_ControllerType *ctrl)
{
    return mhdl_can_prt_get_rec(ctrl->prt_base);
}

/* =========================================================================
 * Interrupt Management
 * ========================================================================= */

void mhcl_can_enable_interrupts(Mhcl_Can_ControllerType *ctrl,
                                 uint32_t func_mask, uint32_t err_mask,
                                 uint32_t safety_mask)
{
    mhdl_can_irc_set_func_ena(ctrl->irc_base, func_mask);
    mhdl_can_irc_set_err_ena(ctrl->irc_base, err_mask);
    mhdl_can_irc_set_safety_ena(ctrl->irc_base, safety_mask);
}

void mhcl_can_disable_interrupts(Mhcl_Can_ControllerType *ctrl)
{
    mhdl_can_irc_set_func_ena(ctrl->irc_base, 0u);
    mhdl_can_irc_set_err_ena(ctrl->irc_base, 0u);
    mhdl_can_irc_set_safety_ena(ctrl->irc_base, 0u);
}

/*
 * Process Functional IRQ line.
 * Based on xcand_process_irq_func():
 *   - Strategy: (1) clear flag, then (2) process event.
 *   - Handles TX FIFO sent/unvalid, RX filter, PRT bus-on, error-active,
 *     PRT RX/TX events.
 */
void mhcl_can_process_irq_func(Mhcl_Can_ControllerType *ctrl)
{
    uint32_t func_raw = mhdl_can_irc_get_func_raw(ctrl->irc_base);
    uint32_t func_ena = mhdl_can_reg_read(ctrl->irc_base, CONTROL_FUNC_ENA);
    uint32_t active   = func_raw & func_ena;

    if (active == 0u) {
        return;
    }

    /* --- TX FIFO Queue interrupts (bits 0-7) --- */
    for (uint32_t fq = 0u; fq < MHCL_CAN_MAX_TX_FIFO; fq++) {
        if ((active & MHDL_BIT(fq)) != 0u) {
            mhdl_can_irc_clear_func(ctrl->irc_base, MHDL_BIT(fq));
            active &= ~MHDL_BIT(fq);

            /* Check if TX FIFO is enabled */
            uint32_t ctrl2 = mhdl_can_reg_read(ctrl->mh_base,
                                                XCAND_MH_CREG_TX_FQ_CTRL2);
            if ((ctrl2 & MHDL_BIT(fq)) != 0u) {
                uint32_t int_sts = mhdl_can_mh_tx_fifo_get_int_status(
                    ctrl->mh_base);

                /* UNVALID interrupt: MH stopped at a descriptor that
                 * may have been validated too late by SW.
                 * Re-trigger start if VALID is now set. */
                if ((int_sts & (MHDL_BIT(fq) << XCAND_MH_CREG_TX_FQ_INT_STS_UNVALID_SHIFT)) != 0u) {
                    mhdl_can_mh_tx_fifo_clear_int(ctrl->mh_base,
                        MHDL_BIT(fq) << XCAND_MH_CREG_TX_FQ_INT_STS_UNVALID_SHIFT);

                    uint32_t addr_pt_offset =
                        XCAND_MH_CREG_TX_FQ_ADD_PT0 +
                        (fq * (XCAND_MH_CREG_TX_FQ_ADD_PT1 -
                               XCAND_MH_CREG_TX_FQ_ADD_PT0));
                    uint32_t cur_desc_addr = mhdl_can_reg_read(
                        ctrl->mh_base, addr_pt_offset);

                    volatile XCAND_MH_TX_DESC_struct *cur_desc =
                        (volatile XCAND_MH_TX_DESC_struct *)(uintptr_t)cur_desc_addr;
                    if (cur_desc->elem0.as_TX_FIFO_Queue.VALIDu1 == 1u) {
                        mhdl_can_mh_tx_fifo_start(ctrl->mh_base, fq);
                    }
                }

                /* SENT interrupt */
                if ((int_sts & (MHDL_BIT(fq) << XCAND_MH_CREG_TX_FQ_INT_STS_SENT_SHIFT)) != 0u) {
                    mhdl_can_mh_tx_fifo_clear_int(ctrl->mh_base,
                        MHDL_BIT(fq) << XCAND_MH_CREG_TX_FQ_INT_STS_SENT_SHIFT);

                    if (ctrl->cb_tx_confirmation != NULL) {
                        ctrl->cb_tx_confirmation(ctrl->instance_id, fq, false);
                    }
                }
            }
        }
    }

    /* --- RX FIFO interrupts (bits 8-15) --- */
    for (uint32_t rq = 0u; rq < MHCL_CAN_MAX_RX_FIFO; rq++) {
        uint32_t rx_bit = MHDL_BIT(rq + 8u);
        if ((active & rx_bit) != 0u) {
            mhdl_can_irc_clear_func(ctrl->irc_base, rx_bit);
            active &= ~rx_bit;

            if (ctrl->cb_rx_indication != NULL) {
                ctrl->cb_rx_indication(ctrl->instance_id, rq);
            }
        }
    }

    /* --- RX Filter IRQ --- */
    if ((active & CONTROL_FUNC_CLR_MH_RX_FILTER_IRQ_MASK) != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base,
                                 CONTROL_FUNC_CLR_MH_RX_FILTER_IRQ_MASK);
        active &= ~CONTROL_FUNC_CLR_MH_RX_FILTER_IRQ_MASK;
    }

    /* --- PRT Bus On --- */
    if ((active & CONTROL_FUNC_CLR_PRT_BUS_ON_MASK) != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base,
                                 CONTROL_FUNC_CLR_PRT_BUS_ON_MASK);
        active &= ~CONTROL_FUNC_CLR_PRT_BUS_ON_MASK;
        /* No specific callback; bus-on is the normal recovered state. */
    }

    /* --- PRT Error Active --- */
    if ((active & CONTROL_FUNC_CLR_PRT_E_ACTIVE_MASK) != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base,
                                 CONTROL_FUNC_CLR_PRT_E_ACTIVE_MASK);
        active &= ~CONTROL_FUNC_CLR_PRT_E_ACTIVE_MASK;

        if (ctrl->cb_error_active != NULL) {
            ctrl->cb_error_active(ctrl->instance_id);
        }
    }

    /* --- PRT RX Event --- */
    if ((active & CONTROL_FUNC_CLR_PRT_RX_EVT_MASK) != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base,
                                 CONTROL_FUNC_CLR_PRT_RX_EVT_MASK);
        active &= ~CONTROL_FUNC_CLR_PRT_RX_EVT_MASK;
    }

    /* --- PRT TX Event --- */
    if ((active & CONTROL_FUNC_CLR_PRT_TX_EVT_MASK) != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base,
                                 CONTROL_FUNC_CLR_PRT_TX_EVT_MASK);
        active &= ~CONTROL_FUNC_CLR_PRT_TX_EVT_MASK;
    }

    /* Clear any remaining unprocessed flags */
    if (active != 0u) {
        mhdl_can_irc_clear_func(ctrl->irc_base, active);
    }
}

/*
 * Process Error IRQ line.
 * Based on xcand_process_irq_err():
 *   - MH descriptor error, PRT bus error, error passive, bus off.
 */
void mhcl_can_process_irq_err(Mhcl_Can_ControllerType *ctrl)
{
    uint32_t err_raw = mhdl_can_irc_get_err_raw(ctrl->irc_base);
    uint32_t err_ena = mhdl_can_reg_read(ctrl->irc_base, CONTROL_ERR_ENA);
    uint32_t active  = err_raw & err_ena;

    if (active == 0u) {
        return;
    }

    /* MH Descriptor Error */
    if ((active & CONTROL_ERR_CLR_MH_DESC_ERR_MASK) != 0u) {
        mhdl_can_irc_clear_err(ctrl->irc_base, CONTROL_ERR_CLR_MH_DESC_ERR_MASK);
        active &= ~CONTROL_ERR_CLR_MH_DESC_ERR_MASK;
    }

    /* PRT Bus Error */
    if ((active & CONTROL_ERR_CLR_PRT_BUS_ERR_MASK) != 0u) {
        mhdl_can_irc_clear_err(ctrl->irc_base, CONTROL_ERR_CLR_PRT_BUS_ERR_MASK);
        active &= ~CONTROL_ERR_CLR_PRT_BUS_ERR_MASK;

        /* Check for Protocol Exception Event (PXE) */
        uint32_t evnt = mhdl_can_prt_get_event(ctrl->prt_base);
        if ((evnt & EVENT_EVNT_PXE_MASK) != 0u) {
            mhdl_can_reg_write(ctrl->prt_base, EVENT_EVNT, EVENT_EVNT_PXE_MASK);
        }
    }

    /* PRT Error Passive */
    if ((active & CONTROL_ERR_CLR_PRT_E_PASSIVE_MASK) != 0u) {
        mhdl_can_irc_clear_err(ctrl->irc_base, CONTROL_ERR_CLR_PRT_E_PASSIVE_MASK);
        active &= ~CONTROL_ERR_CLR_PRT_E_PASSIVE_MASK;

        if (ctrl->cb_error_passive != NULL) {
            ctrl->cb_error_passive(ctrl->instance_id);
        }
    }

    /* PRT Bus Off */
    if ((active & CONTROL_ERR_CLR_PRT_BUS_OFF_MASK) != 0u) {
        mhdl_can_irc_clear_err(ctrl->irc_base, CONTROL_ERR_CLR_PRT_BUS_OFF_MASK);
        active &= ~CONTROL_ERR_CLR_PRT_BUS_OFF_MASK;

        if (ctrl->cb_busoff != NULL) {
            ctrl->cb_busoff(ctrl->instance_id);
        }
    }

    /* Clear remaining */
    if (active != 0u) {
        mhdl_can_irc_clear_err(ctrl->irc_base, active);
    }
}

/*
 * Process Safety IRQ line.
 * Based on xcand_process_irq_safety():
 *   - MH memory safety error, etc.
 */
void mhcl_can_process_irq_safety(Mhcl_Can_ControllerType *ctrl)
{
    uint32_t safety_raw = mhdl_can_irc_get_safety_raw(ctrl->irc_base);
    uint32_t safety_ena = mhdl_can_reg_read(ctrl->irc_base, CONTROL_SAFETY_ENA);
    uint32_t active     = safety_raw & safety_ena;

    if (active == 0u) {
        return;
    }

    /* MH Memory Safety Error */
    if ((active & CONTROL_SAFETY_CLR_MH_MEM_SFTY_ERR_MASK) != 0u) {
        mhdl_can_irc_clear_safety(ctrl->irc_base,
                                   CONTROL_SAFETY_CLR_MH_MEM_SFTY_ERR_MASK);
        active &= ~CONTROL_SAFETY_CLR_MH_MEM_SFTY_ERR_MASK;
    }

    /* Clear remaining */
    if (active != 0u) {
        mhdl_can_irc_clear_safety(ctrl->irc_base, active);
    }
}

/* =========================================================================
 * DLC Conversion — Public API
 * ========================================================================= */

uint32_t mhcl_can_dlc_to_bytes(Mhcl_Can_FrameFormatType ff, uint32_t dlc)
{
    return mhcl_dlc_to_bytes_internal(ff, dlc, false);
}

uint32_t mhcl_can_bytes_to_dlc(Mhcl_Can_FrameFormatType ff, uint32_t bytes)
{
    switch (ff) {
    case MHCL_CAN_FF_XL:
        return (bytes > 0u) ? (bytes - 1u) : 0u;

    case MHCL_CAN_FF_FD:
        if (bytes <= 8u)  return bytes;
        if (bytes <= 12u) return 9u;
        if (bytes <= 16u) return 10u;
        if (bytes <= 20u) return 11u;
        if (bytes <= 24u) return 12u;
        if (bytes <= 32u) return 13u;
        if (bytes <= 48u) return 14u;
        return 15u;

    case MHCL_CAN_FF_CC:
    default:
        return (bytes <= MHCL_CAN_CC_MAX_DATA_BYTES) ? bytes
                                                      : MHCL_CAN_CC_MAX_DATA_BYTES;
    }
}
