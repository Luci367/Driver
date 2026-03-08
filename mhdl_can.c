/*
 * XCAN Hardware Driving Layer (HDL) — Implementation
 *
 * Each function performs ONE atomic register operation or a tightly-coupled
 * multi-step sequence (e.g. unlock + stop). All register accesses use the
 * platform macros from regdef/xcan_platform.h via the inline helpers in
 * mhdl_can.h.
 *
 * Reference:
 *   - xcan_prt.c / xcand_mh.c / xcand_irc.c  (xcan_sw_example, Bosch)
 *   - XCAN User Manual v3.90
 *
 * DO NOT use any existing project files (mhal_can.h, can_driver.c, etc.).
 */

#include "mhdl_can.h"
#include "can_debug.h"

/* *************************************************************************
 * PRT — Protocol Controller register operations
 * ************************************************************************* */

void mhdl_can_prt_set_nbtp(uint32_t base, uint32_t brp, uint32_t tseg1,
                            uint32_t tseg2, uint32_t sjw)
{
    CAN_DBG_VERB(DBG_HDL, "NBTP brp=%u tseg1=%u tseg2=%u sjw=%u",
                 brp, tseg1, tseg2, sjw);
    ConfigurationNbtpUt reg;
    reg.as_uint32   = 0u;
    reg.as_s.BrpU5  = brp;
    reg.as_s.Ntseg1U9 = tseg1;
    reg.as_s.Ntseg2U7 = tseg2;
    reg.as_s.NsjwU7   = sjw;
    mhdl_can_reg_write_verify(base, CONFIGURATION_NBTP, reg.as_uint32);
}

void mhdl_can_prt_set_dbtp(uint32_t base, uint32_t tseg1, uint32_t tseg2,
                            uint32_t sjw, uint32_t tdco)
{
    CAN_DBG_VERB(DBG_HDL, "DBTP tseg1=%u tseg2=%u sjw=%u tdco=%u",
                 tseg1, tseg2, sjw, tdco);
    ConfigurationDbtpUt reg;
    reg.as_uint32     = 0u;
    reg.as_s.Dtseg1U8 = tseg1;
    reg.as_s.Dtseg2U7 = tseg2;
    reg.as_s.DsjwU7   = sjw;
    reg.as_s.DtdcoU8  = tdco;
    mhdl_can_reg_write_verify(base, CONFIGURATION_DBTP, reg.as_uint32);
}

void mhdl_can_prt_set_xbtp(uint32_t base, uint32_t tseg1, uint32_t tseg2,
                            uint32_t sjw, uint32_t tdco)
{
    CAN_DBG_VERB(DBG_HDL, "XBTP tseg1=%u tseg2=%u sjw=%u tdco=%u",
                 tseg1, tseg2, sjw, tdco);
    ConfigurationXbtpUt reg;
    reg.as_uint32     = 0u;
    reg.as_s.Xtseg1U8 = tseg1;
    reg.as_s.Xtseg2U7 = tseg2;
    reg.as_s.XsjwU7   = sjw;
    reg.as_s.XtdcoU8  = tdco;
    mhdl_can_reg_write_verify(base, CONFIGURATION_XBTP, reg.as_uint32);
}

void mhdl_can_prt_set_mode(uint32_t base, uint32_t fdoe, uint32_t xloe,
                            uint32_t tdce, uint32_t pxhd, uint32_t efbi,
                            uint32_t txp, uint32_t mon, uint32_t rstr,
                            uint32_t sfs, uint32_t xltr, uint32_t efdi,
                            uint32_t fime)
{
    CAN_DBG_VERB(DBG_HDL, "MODE fdoe=%u xloe=%u tdce=%u pxhd=%u txp=%u sfs=%u",
                 fdoe, xloe, tdce, pxhd, txp, sfs);
    ConfigurationModeUt reg;
    reg.as_uint32    = 0u;
    reg.as_s.FdoeU1  = fdoe;
    reg.as_s.XloeU1  = xloe;
    reg.as_s.TdceU1  = tdce;
    reg.as_s.PxhdU1  = pxhd;
    reg.as_s.EfbiU1  = efbi;
    reg.as_s.TxpU1   = txp;
    reg.as_s.MonU1   = mon;
    reg.as_s.RstrU1  = rstr;
    reg.as_s.SfsU1   = sfs;
    reg.as_s.XltrU1  = xltr;
    reg.as_s.EfdiU1  = efdi;
    reg.as_s.FimeU1  = fime;
    mhdl_can_reg_write_verify(base, CONFIGURATION_MODE, reg.as_uint32);
}

void mhdl_can_prt_set_pwme(uint32_t base, uint32_t pwms, uint32_t pwml,
                            uint32_t pwmo)
{
    ConfigurationPcfgUt reg;
    reg.as_uint32    = 0u;
    reg.as_s.PwmsU6  = pwms;
    reg.as_s.PwmlU6  = pwml;
    reg.as_s.PwmoU6  = pwmo;
    mhdl_can_reg_write_verify(base, CONFIGURATION_PCFG, reg.as_uint32);
}

void mhdl_can_prt_start(uint32_t base)
{
    CAN_DBG_INFO(DBG_HDL, "PRT start base=0x%08X", base);
    mhdl_can_reg_write(base, CONTROL_CTRL, CONTROL_CTRL_STRT_MASK);
}

void mhdl_can_prt_stop(uint32_t base, bool immediate)
{
    CAN_DBG_INFO(DBG_HDL, "PRT stop base=0x%08X immd=%u", base, immediate);
    /* Unlock sequence required before writing STOP to CTRL */
    mhdl_can_reg_write(base, CONTROL_LOCK, MHDL_PRT_LOCK_ULK_STOP_W1);
    mhdl_can_reg_write(base, CONTROL_LOCK, MHDL_PRT_LOCK_ULK_STOP_W2);

    if (!immediate) {
        mhdl_can_reg_write(base, CONTROL_CTRL, CONTROL_CTRL_STOP_MASK);
    } else {
        mhdl_can_reg_write(base, CONTROL_CTRL,
                           CONTROL_CTRL_STOP_MASK | CONTROL_CTRL_IMMD_MASK);
        /* After immediate stop the PRT state machines may be in an
         * unrecoverable state — issue a SW reset (per User Manual). */
        mhdl_can_prt_sw_reset(base);
    }
}

void mhdl_can_prt_sw_reset(uint32_t base)
{
    CAN_DBG_INFO(DBG_HDL, "PRT sw_reset base=0x%08X", base);
    mhdl_can_reg_write(base, CONTROL_CTRL, CONTROL_CTRL_SRES_MASK);
}

uint32_t mhdl_can_prt_get_stat(uint32_t base)
{
    return mhdl_can_reg_read(base, STATUS_STAT);
}

bool mhdl_can_prt_is_started(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);

    /* PRT is STARTED when ACT != INACTIVE or INT (Integration) bit is set.
     * Matches xcan_prt_check_if_started(). */
    if ((reg.as_s.ActU2 == MHDL_PRT_STAT_ACT_INACTIVE) &&
        (reg.as_s.IntU1 == 0u)) {
        return false;
    }
    return true;
}

bool mhdl_can_prt_is_busoff(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);

    /* Bus-Off (not recovering): BO==1 and INT==0.
     * Matches xcan_prt_check_if_busoff(). */
    if ((reg.as_s.BoU1 == 1u) && (reg.as_s.IntU1 == 0u)) {
        return true;
    }
    return false;
}

uint32_t mhdl_can_prt_get_tec(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);
    return reg.as_s.TecU8;
}

uint32_t mhdl_can_prt_get_rec(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);
    return reg.as_s.RecU7;
}

bool mhdl_can_prt_is_error_passive(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);
    return (reg.as_s.EpU1 == 1u);
}

uint32_t mhdl_can_prt_get_version(uint32_t base)
{
    return mhdl_can_reg_read(base, STATUS_PREL);
}

bool mhdl_can_prt_is_integrated(uint32_t base)
{
    StatusStatUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, STATUS_STAT);

    /* Bus integration finished when ACT != INACTIVE (i.e. Idle, Rx, or Tx).
     * Matches xcan_prt_check_if_started_and_integrated(). */
    if (reg.as_s.ActU2 == MHDL_PRT_STAT_ACT_INACTIVE) {
        return false;
    }
    return true;
}

uint32_t mhdl_can_prt_get_event(uint32_t base)
{
    return mhdl_can_reg_read(base, EVENT_EVNT);
}

/* *************************************************************************
 * MH — Message Handler register operations
 * ************************************************************************* */

void mhdl_can_mh_start(uint32_t base)
{
    CAN_DBG_INFO(DBG_HDL, "MH start base=0x%08X", base);
    XcandmhcregMhctrlUt reg;
    reg.as_uint32    = 0u;
    reg.as_s.StartU1 = 1u;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_MH_CTRL, reg.as_uint32);
}

void mhdl_can_mh_stop(uint32_t base)
{
    CAN_DBG_INFO(DBG_HDL, "MH stop base=0x%08X", base);
    XcandmhcregMhctrlUt reg;
    reg.as_uint32    = 0u;
    reg.as_s.StartU1 = 0u;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_MH_CTRL, reg.as_uint32);
}

bool mhdl_can_mh_is_started(uint32_t base)
{
    XcandmhcregMhctrlUt reg;
    reg.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_MH_CTRL);
    return (reg.as_s.StartU1 == 1u);
}

void mhdl_can_mh_set_global_cfg(uint32_t base, uint32_t inst_num,
                                 uint32_t retrans_max, uint32_t rx_cont_mode)
{
    CAN_DBG_VERB(DBG_HDL, "MH cfg inst=%u retrans=%u rxcont=%u",
                 inst_num, retrans_max, rx_cont_mode);
    XcandmhcregMhcfgUt reg;
    reg.as_uint32         = 0u;
    reg.as_s.InstnumU3    = inst_num;
    reg.as_s.MaxretransU3 = retrans_max;
    reg.as_s.RxcontdcU1   = rx_cont_mode;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_MH_CFG, reg.as_uint32);
}

void mhdl_can_mh_set_axi_params(uint32_t base, uint32_t ar_max, uint32_t aw_max)
{
    XcandmhcregAxiparamsUt reg;
    reg.as_uint32          = 0u;
    reg.as_s.ArmaxpendU2   = ar_max;
    reg.as_s.AwmaxpendU2   = aw_max;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_AXI_PARAMS, reg.as_uint32);
}

void mhdl_can_mh_set_mem_addresses(uint32_t base, uint32_t fq_base,
                                    uint32_t pq_base, uint32_t rx_filter_base)
{
    /* TX_DESC_MEM_ADD — FQ and PQ base addresses in LMEM */
    XcandmhcregTxdescmemaddUt tx_mem;
    tx_mem.as_uint32            = 0u;
    tx_mem.as_s.FqbaseaddrU16   = fq_base;
    tx_mem.as_s.PqbaseaddrU16   = pq_base;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_DESC_MEM_ADD, tx_mem.as_uint32);

    /* RX_FILTER_MEM_ADD */
    XcandmhcregRxfiltermemaddUt rx_mem;
    rx_mem.as_uint32            = 0u;
    rx_mem.as_s.BaseaddrU16     = rx_filter_base;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FILTER_MEM_ADD, rx_mem.as_uint32);
}

void mhdl_can_mh_set_tx_fifo_config(uint32_t base, uint32_t fifo,
                                     uint32_t start_addr, uint32_t max_desc)
{
    CAN_DBG_VERB(DBG_HDL, "TX FIFO%u cfg addr=0x%08X max=%u",
                 fifo, start_addr, max_desc);
    uint32_t reg_block_offset = MHDL_MH_TX_FIFO_CFG_REG_BLOCK * fifo;

    /* TX_FQ_START_ADDn */
    mhdl_can_reg_write_verify(base,
        XCAND_MH_CREG_TX_FQ_START_ADD0 + reg_block_offset, start_addr);

    /* TX_FQ_SIZEn */
    XcandmhcregTxfqsize0Ut size_reg;
    size_reg.as_uint32      = 0u;
    size_reg.as_s.MaxdescU10 = max_desc;
    mhdl_can_reg_write_verify(base,
        XCAND_MH_CREG_TX_FQ_SIZE0 + reg_block_offset, size_reg.as_uint32);
}

void mhdl_can_mh_tx_fifo_enable(uint32_t base, uint32_t fifo)
{
    CAN_DBG_VERB(DBG_HDL, "TX FIFO%u enable", fifo);
    XcandmhcregTxfqctrl2Ut reg;
    reg.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_CTRL2);
    reg.as_s.EnableU8 |= MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_FQ_CTRL2, reg.as_uint32);
}

void mhdl_can_mh_tx_fifo_start(uint32_t base, uint32_t fifo)
{
    XcandmhcregTxfqctrl0Ut reg;
    reg.as_uint32    = 0u;
    reg.as_s.StartU8 = MHDL_BIT(fifo);
    mhdl_can_reg_write(base, XCAND_MH_CREG_TX_FQ_CTRL0, reg.as_uint32);
}

void mhdl_can_mh_tx_fifo_abort(uint32_t base, uint32_t fifo)
{
    CAN_DBG_INFO(DBG_HDL, "TX FIFO%u abort", fifo);
    XcandmhcregTxfqctrl1Ut ctrl1;
    XcandmhcregTxfqsts0Ut  sts0;
    XcandmhcregTxfqctrl2Ut ctrl2;

    /* Step 1: Unlock + set ABORT bit */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_uint32    = 0u;
    ctrl1.as_s.AbortU8 = MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_FQ_CTRL1, ctrl1.as_uint32);

    /* Step 2: Wait for BUSY and STOP bits to clear for this FIFO */
    do {
        sts0.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_STS0);
    } while (((sts0.as_s.BusyU8 & MHDL_BIT(fifo)) != 0u) ||
             ((sts0.as_s.StopU8 & MHDL_BIT(fifo)) != 0u));

    /* Step 3: Unlock + clear ABORT bit */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_s.AbortU8 = 0u;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_FQ_CTRL1, ctrl1.as_uint32);

    /* Step 4: Disable this FIFO */
    ctrl2.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_CTRL2);
    ctrl2.as_s.EnableU8 &= ~MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_FQ_CTRL2, ctrl2.as_uint32);
}

uint32_t mhdl_can_mh_tx_fifo_get_sts0(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_STS0);
}

uint32_t mhdl_can_mh_tx_fifo_get_sts1(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_STS1);
}

uint32_t mhdl_can_mh_tx_fifo_get_int_status(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_TX_FQ_INT_STS);
}

void mhdl_can_mh_tx_fifo_clear_int(uint32_t base, uint32_t mask)
{
    mhdl_can_reg_write(base, XCAND_MH_CREG_TX_FQ_INT_STS, mask);
}

void mhdl_can_mh_set_tx_pq_config(uint32_t base, uint32_t start_addr,
                                    uint32_t enable_mask)
{
    /* TX_PQ_START_ADD */
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_PQ_START_ADD, start_addr);

    /* TX_PQ_CTRL2 — enable mask (one bit per slot) */
    XcandmhcregTxpqctrl2Ut reg;
    reg.as_uint32        = 0u;
    reg.as_s.EnableU32   = enable_mask;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_PQ_CTRL2, reg.as_uint32);
}

void mhdl_can_mh_tx_pq_start(uint32_t base, uint32_t slot)
{
    XcandmhcregTxpqctrl0Ut reg;
    reg.as_uint32     = 0u;
    reg.as_s.StartU32 = MHDL_BIT(slot);
    mhdl_can_reg_write(base, XCAND_MH_CREG_TX_PQ_CTRL0, reg.as_uint32);
}

void mhdl_can_mh_tx_pq_abort(uint32_t base, uint32_t slot)
{
    CAN_DBG_INFO(DBG_HDL, "TX PQ slot%u abort", slot);
    XcandmhcregTxpqctrl1Ut ctrl1;
    XcandmhcregTxpqsts0Ut  sts0;
    XcandmhcregTxpqctrl2Ut ctrl2;

    /* Step 1: Unlock + set ABORT bit for this slot */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_uint32     = 0u;
    ctrl1.as_s.AbortU32 = MHDL_BIT(slot);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_PQ_CTRL1, ctrl1.as_uint32);

    /* Step 2: Wait for BUSY bit to clear for this slot */
    do {
        sts0.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_TX_PQ_STS0);
    } while ((sts0.as_s.BusyU32 & MHDL_BIT(slot)) != 0u);

    /* Step 3: Unlock + clear ABORT bit */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_s.AbortU32 = 0u;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_PQ_CTRL1, ctrl1.as_uint32);

    /* Step 4: Disable this slot */
    ctrl2.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_TX_PQ_CTRL2);
    ctrl2.as_s.EnableU32 &= ~MHDL_BIT(slot);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_TX_PQ_CTRL2, ctrl2.as_uint32);
}

uint32_t mhdl_can_mh_tx_pq_get_sts0(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_TX_PQ_STS0);
}

void mhdl_can_mh_set_rx_fifo_config(uint32_t base, uint32_t fifo,
                                     uint32_t start_addr, uint32_t max_desc,
                                     uint32_t dc_size_word, uint32_t dc_start,
                                     bool continuous_mode)
{
    CAN_DBG_VERB(DBG_HDL, "RX FIFO%u cfg addr=0x%08X max=%u dc_sz=%u cont=%u",
                 fifo, start_addr, max_desc, dc_size_word, continuous_mode);
    uint32_t reg_block_offset = MHDL_MH_RX_FIFO_CFG_REG_BLOCK * fifo;

    /* RX_FQ_SIZEn — max descriptors + data container size (in 32-byte granularity) */
    XcandmhcregRxfqsize0Ut size_reg;
    size_reg.as_uint32       = 0u;
    size_reg.as_s.MaxdescU10 = max_desc;
    size_reg.as_s.DcsizeU12  = dc_size_word >> 3u;  /* 32-byte granularity: word / 8 */
    mhdl_can_reg_write_verify(base,
        XCAND_MH_CREG_RX_FQ_SIZE0 + reg_block_offset, size_reg.as_uint32);

    /* RX_FQ_START_ADDn — descriptor list start address in system memory */
    mhdl_can_reg_write_verify(base,
        XCAND_MH_CREG_RX_FQ_START_ADD0 + reg_block_offset, start_addr);

    if (continuous_mode) {
        /* RX_FQ_DC_START_ADDn — data container start in system memory */
        mhdl_can_reg_write_verify(base,
            XCAND_MH_CREG_RX_FQ_DC_START_ADD0 + reg_block_offset, dc_start);

        /* RX_FQ_RD_ADD_PTn — initial read pointer; VAL=0b11 for initial start */
        mhdl_can_reg_write_verify(base,
            XCAND_MH_CREG_RX_FQ_RD_ADD_PT0 + reg_block_offset, dc_start + 3u);
    }
}

void mhdl_can_mh_rx_fifo_enable(uint32_t base, uint32_t fifo)
{
    XcandmhcregRxfqctrl2Ut reg;
    reg.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_RX_FQ_CTRL2);
    reg.as_s.EnableU8 |= MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FQ_CTRL2, reg.as_uint32);
}

void mhdl_can_mh_rx_fifo_start(uint32_t base, uint32_t fifo)
{
    XcandmhcregRxfqctrl0Ut reg;
    reg.as_uint32    = 0u;
    reg.as_s.StartU8 = MHDL_BIT(fifo);
    mhdl_can_reg_write(base, XCAND_MH_CREG_RX_FQ_CTRL0, reg.as_uint32);
}

void mhdl_can_mh_rx_fifo_abort(uint32_t base, uint32_t fifo)
{
    CAN_DBG_INFO(DBG_HDL, "RX FIFO%u abort", fifo);
    XcandmhcregRxfqctrl1Ut ctrl1;
    XcandmhcregRxfqsts0Ut  sts0;
    XcandmhcregRxfqctrl2Ut ctrl2;

    /* Step 1: Unlock + set ABORT bit */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_uint32    = 0u;
    ctrl1.as_s.AbortU8 = MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FQ_CTRL1, ctrl1.as_uint32);

    /* Step 2: Wait for BUSY and STOP bits to clear for this FIFO */
    do {
        sts0.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_RX_FQ_STS0);
    } while (((sts0.as_s.BusyU8 & MHDL_BIT(fifo)) != 0u) ||
             ((sts0.as_s.StopU8 & MHDL_BIT(fifo)) != 0u));

    /* Step 3: Unlock + clear ABORT bit */
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W1);
    mhdl_can_reg_write(base, XCAND_MH_CREG_MH_LOCK, MHDL_MH_LOCK_ULK_W2);
    ctrl1.as_s.AbortU8 = 0u;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FQ_CTRL1, ctrl1.as_uint32);

    /* Step 4: Disable this FIFO */
    ctrl2.as_uint32 = mhdl_can_reg_read(base, XCAND_MH_CREG_RX_FQ_CTRL2);
    ctrl2.as_s.EnableU8 &= ~MHDL_BIT(fifo);
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FQ_CTRL2, ctrl2.as_uint32);
}

uint32_t mhdl_can_mh_rx_fifo_get_sts0(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_RX_FQ_STS0);
}

uint32_t mhdl_can_mh_rx_fifo_get_int_status(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_RX_FQ_INT_STS);
}

void mhdl_can_mh_set_rx_filter_ctrl(uint32_t base, uint32_t num_elements,
                                     uint32_t anmf, uint32_t anff,
                                     uint32_t anmf_fifo, uint32_t threshold)
{
    XcandmhcregRxfilterctrlUt reg;
    reg.as_uint32        = 0u;
    reg.as_s.NbfeU8      = num_elements;
    reg.as_s.AnmfU1      = anmf;
    reg.as_s.AnffU1      = anff;
    reg.as_s.AnmffqU3    = anmf_fifo;
    reg.as_s.ThresholdU5 = threshold;
    mhdl_can_reg_write_verify(base, XCAND_MH_CREG_RX_FILTER_CTRL, reg.as_uint32);
}

void mhdl_can_mh_write_lmem(uint32_t lmem_base, uint32_t offset, uint32_t value)
{
    mhdl_can_reg_write(lmem_base, offset, value);
}

uint32_t mhdl_can_mh_read_lmem(uint32_t lmem_base, uint32_t offset)
{
    return mhdl_can_reg_read(lmem_base, offset);
}

uint32_t mhdl_can_mh_get_version(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_VERSION);
}

uint32_t mhdl_can_mh_get_status(uint32_t base)
{
    return mhdl_can_reg_read(base, XCAND_MH_CREG_MH_STS);
}

/* *************************************************************************
 * IRC — Interrupt Controller register operations
 * ************************************************************************* */

void mhdl_can_irc_set_func_ena(uint32_t base, uint32_t mask)
{
    CAN_DBG_VERB(DBG_HDL, "IRC func_ena=0x%08X", mask);
    mhdl_can_reg_write_verify(base, CONTROL_FUNC_ENA, mask);
}

void mhdl_can_irc_set_err_ena(uint32_t base, uint32_t mask)
{
    CAN_DBG_VERB(DBG_HDL, "IRC err_ena=0x%08X", mask);
    mhdl_can_reg_write_verify(base, CONTROL_ERR_ENA, mask);
}

void mhdl_can_irc_set_safety_ena(uint32_t base, uint32_t mask)
{
    CAN_DBG_VERB(DBG_HDL, "IRC safety_ena=0x%08X", mask);
    mhdl_can_reg_write_verify(base, CONTROL_SAFETY_ENA, mask);
}

uint32_t mhdl_can_irc_get_func_raw(uint32_t base)
{
    return mhdl_can_reg_read(base, EVENT_FUNC_RAW);
}

uint32_t mhdl_can_irc_get_err_raw(uint32_t base)
{
    return mhdl_can_reg_read(base, EVENT_ERR_RAW);
}

uint32_t mhdl_can_irc_get_safety_raw(uint32_t base)
{
    return mhdl_can_reg_read(base, EVENT_SAFETY_RAW);
}

void mhdl_can_irc_clear_func(uint32_t base, uint32_t mask)
{
    mhdl_can_reg_write(base, CONTROL_FUNC_CLR, mask);
}

void mhdl_can_irc_clear_err(uint32_t base, uint32_t mask)
{
    mhdl_can_reg_write(base, CONTROL_ERR_CLR, mask);
}

void mhdl_can_irc_clear_safety(uint32_t base, uint32_t mask)
{
    mhdl_can_reg_write(base, CONTROL_SAFETY_CLR, mask);
}
