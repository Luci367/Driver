/*
 * XCAN Hardware Driving Layer (HDL)
 *
 * Lowest layer of the XCAN AUTOSAR driver. Each function performs ONE
 * atomic register operation (read or write) or a tightly-coupled
 * sequence (e.g. unlock + stop). All register accesses go through the
 * platform macros defined in regdef/xcan_platform.h.
 *
 * Reference:
 *   - xcan_sw_example  (Robert Bosch GmbH)
 *   - XCAN User Manual v3.90
 *
 * DO NOT use any existing project files (mhal_can.h, can_driver.c, etc.).
 */

#ifndef MHDL_CAN_H
#define MHDL_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "regdef/xcan_platform.h"
#include "regdef/xcan_mh_regdef.h"
#include "regdef/xcan_prt_regdef.h"
#include "regdef/xcan_irc_regdef.h"

/* =========================================================================
 * Sub-module base address offsets from XCAN instance base
 * (from xcand.h in xcan_sw_example)
 * ========================================================================= */
#define XCAN_MH_OFFSET   0x000u
#define XCAN_PRT_OFFSET  0x900u
#define XCAN_IRC_OFFSET  0xA00u

/* =========================================================================
 * PRT Lock register unlock sequences
 * (from xcan_prt.h in xcan_sw_example)
 * ========================================================================= */
#define MHDL_PRT_LOCK_ULK_STOP_W1   0x00001234u
#define MHDL_PRT_LOCK_ULK_STOP_W2   0x00004321u
#define MHDL_PRT_LOCK_TMK_W1        0x67890000u
#define MHDL_PRT_LOCK_TMK_W2        0x98760000u

/* PRT STAT.ACT field values (from xcan_prt.h) */
#define MHDL_PRT_STAT_ACT_INACTIVE     (0x0u)
#define MHDL_PRT_STAT_ACT_IDLE         (0x1u)
#define MHDL_PRT_STAT_ACT_RECEIVER     (0x2u)
#define MHDL_PRT_STAT_ACT_TRANSMITTER  (0x3u)

/* =========================================================================
 * MH Lock register unlock sequences
 * (from xcand_mh.h in xcan_sw_example)
 * ========================================================================= */
#define MHDL_MH_LOCK_ULK_W1   0x00001234u
#define MHDL_MH_LOCK_ULK_W2   0x00004321u
#define MHDL_MH_LOCK_TMK_W1   0x67890000u
#define MHDL_MH_LOCK_TMK_W2   0x98760000u

/* =========================================================================
 * MH Limits
 * (from xcand_mh.h in xcan_sw_example)
 * ========================================================================= */
#define MHDL_MH_MAX_TX_FIFO_NUMBER       8u
#define MHDL_MH_MAX_TX_PQ_SLOTS          32u
#define MHDL_MH_MAX_RX_FIFO_NUMBER       8u
#define MHDL_MH_TX_FIFO_CFG_REG_BLOCK    (XCAND_MH_CREG_TX_FQ_START_ADD1 - XCAND_MH_CREG_TX_FQ_START_ADD0)
#define MHDL_MH_RX_FIFO_CFG_REG_BLOCK    (XCAND_MH_CREG_RX_FQ_START_ADD1 - XCAND_MH_CREG_RX_FQ_START_ADD0)

/* =========================================================================
 * Generic register helpers (inline)
 * Equivalent to xcan_reg_set / xcan_reg_get / xcan_reg_set_and_check
 * from xcan_common_func.h in xcan_sw_example.
 * ========================================================================= */
static inline void mhdl_can_reg_write(uint32_t base, uint32_t offset, uint32_t value)
{
    XCAN_REG_WRITE32(base + offset, value);
}

static inline uint32_t mhdl_can_reg_read(uint32_t base, uint32_t offset)
{
    return XCAN_REG_READ32(base + offset);
}

static inline bool mhdl_can_reg_write_verify(uint32_t base, uint32_t offset, uint32_t value)
{
    XCAN_REG_WRITE32(base + offset, value);
    return (XCAN_REG_READ32(base + offset) == value);
}

/* =========================================================================
 * BIT macro (same semantics as example's BIT())
 * ========================================================================= */
#ifndef MHDL_BIT
#define MHDL_BIT(n)  (1u << (n))
#endif

/* *************************************************************************
 * PRT — Protocol Controller register operations
 * (Reference: xcan_prt.c)
 * ************************************************************************* */

/* Write CONFIGURATION_NBTP register (Nominal / Arbitration bit timing).
 * brp, tseg1, tseg2, sjw are the raw register-field values (already -1 adjusted). */
void mhdl_can_prt_set_nbtp(uint32_t base, uint32_t brp, uint32_t tseg1,
                            uint32_t tseg2, uint32_t sjw);

/* Write CONFIGURATION_DBTP register (CAN FD Data phase bit timing).
 * tseg1, tseg2, sjw, tdco are raw register-field values. */
void mhdl_can_prt_set_dbtp(uint32_t base, uint32_t tseg1, uint32_t tseg2,
                            uint32_t sjw, uint32_t tdco);

/* Write CONFIGURATION_XBTP register (CAN XL Data phase bit timing).
 * tseg1, tseg2, sjw, tdco are raw register-field values. */
void mhdl_can_prt_set_xbtp(uint32_t base, uint32_t tseg1, uint32_t tseg2,
                            uint32_t sjw, uint32_t tdco);

/* Write CONFIGURATION_MODE register.
 * Each parameter is 0 or 1, directly mapped to the MODE bit-field.
 * All 12 writable mode bits are exposed for full control. */
void mhdl_can_prt_set_mode(uint32_t base, uint32_t fdoe, uint32_t xloe,
                            uint32_t tdce, uint32_t pxhd, uint32_t efbi,
                            uint32_t txp, uint32_t mon, uint32_t rstr,
                            uint32_t sfs, uint32_t xltr, uint32_t efdi,
                            uint32_t fime);

/* Write CONFIGURATION_PCFG register (PWME — PWM Encoding for XL). */
void mhdl_can_prt_set_pwme(uint32_t base, uint32_t pwms, uint32_t pwml,
                            uint32_t pwmo);

/* Set CONTROL_CTRL.STRT to start PRT (connect to bus). */
void mhdl_can_prt_start(uint32_t base);

/* Perform PRT stop: unlock sequence + STOP (+IMMD if immediate).
 * If immediate==true, also triggers SW reset after stop. */
void mhdl_can_prt_stop(uint32_t base, bool immediate);

/* Perform PRT software reset (CTRL.SRES). Must be called when PRT is stopped. */
void mhdl_can_prt_sw_reset(uint32_t base);

/* Read STATUS_STAT register (raw uint32). */
uint32_t mhdl_can_prt_get_stat(uint32_t base);

/* Check if PRT is started: ACT != INACTIVE or INT == 1. */
bool mhdl_can_prt_is_started(uint32_t base);

/* Check if PRT is in Bus-Off: STAT.BO==1 && STAT.INT==0. */
bool mhdl_can_prt_is_busoff(uint32_t base);

/* Read Transmit Error Counter from STAT.TEC. */
uint32_t mhdl_can_prt_get_tec(uint32_t base);

/* Read Receive Error Counter from STAT.REC. */
uint32_t mhdl_can_prt_get_rec(uint32_t base);

/* Check if PRT is in Error-Passive state: STAT.EP==1. */
bool mhdl_can_prt_is_error_passive(uint32_t base);

/* Read PRT release (version) register STATUS_PREL. */
uint32_t mhdl_can_prt_get_version(uint32_t base);

/* Check if PRT is started AND bus integration is finished (ACT != INACTIVE). */
bool mhdl_can_prt_is_integrated(uint32_t base);

/* Read PRT event register EVENT_EVNT (read-clear). */
uint32_t mhdl_can_prt_get_event(uint32_t base);

/* *************************************************************************
 * MH — Message Handler register operations
 * (Reference: xcand_mh.c)
 * ************************************************************************* */

/* Write MH_CTRL.START = 1 to start MH. */
void mhdl_can_mh_start(uint32_t base);

/* Write MH_CTRL.START = 0 to stop MH. */
void mhdl_can_mh_stop(uint32_t base);

/* Read MH_CTRL.START bit — returns true if MH is operating. */
bool mhdl_can_mh_is_started(uint32_t base);

/* Write MH_CFG register (instance number, retrans_max, rx_cont_mode). */
void mhdl_can_mh_set_global_cfg(uint32_t base, uint32_t inst_num,
                                 uint32_t retrans_max, uint32_t rx_cont_mode);

/* Write AXI_PARAMS register (ar_max_pend, aw_max_pend). */
void mhdl_can_mh_set_axi_params(uint32_t base, uint32_t ar_max, uint32_t aw_max);

/* Write TX_DESC_MEM_ADD (FQ + PQ base addresses in LMEM) and
 * RX_FILTER_MEM_ADD (RX filter base in LMEM). */
void mhdl_can_mh_set_mem_addresses(uint32_t base, uint32_t fq_base,
                                    uint32_t pq_base, uint32_t rx_filter_base);

/* Configure a TX FIFO queue: start address + max descriptors.
 * Writes TX_FQ_START_ADDn and TX_FQ_SIZEn. */
void mhdl_can_mh_set_tx_fifo_config(uint32_t base, uint32_t fifo,
                                     uint32_t start_addr, uint32_t max_desc);

/* Set the enable bit for a TX FIFO in TX_FQ_CTRL2. */
void mhdl_can_mh_tx_fifo_enable(uint32_t base, uint32_t fifo);

/* Trigger TX FIFO start by writing TX_FQ_CTRL0. */
void mhdl_can_mh_tx_fifo_start(uint32_t base, uint32_t fifo);

/* Abort a TX FIFO: unlock -> set abort -> wait -> clear abort -> disable.
 * This is a tightly-coupled multi-step sequence. */
void mhdl_can_mh_tx_fifo_abort(uint32_t base, uint32_t fifo);

/* Read TX_FQ_STS0 (busy + stop bits per FIFO). */
uint32_t mhdl_can_mh_tx_fifo_get_sts0(uint32_t base);

/* Read TX_FQ_STS1 (unvalid + error bits per FIFO). */
uint32_t mhdl_can_mh_tx_fifo_get_sts1(uint32_t base);

/* Read TX_FQ_INT_STS register. */
uint32_t mhdl_can_mh_tx_fifo_get_int_status(uint32_t base);

/* Clear TX FIFO interrupt status bits by writing mask to TX_FQ_INT_STS. */
void mhdl_can_mh_tx_fifo_clear_int(uint32_t base, uint32_t mask);

/* Configure TX Priority Queue: start address + enable mask.
 * Writes TX_PQ_START_ADD and TX_PQ_CTRL2. */
void mhdl_can_mh_set_tx_pq_config(uint32_t base, uint32_t start_addr,
                                    uint32_t enable_mask);

/* Trigger TX PQ slot start by writing TX_PQ_CTRL0. */
void mhdl_can_mh_tx_pq_start(uint32_t base, uint32_t slot);

/* Abort a TX PQ slot: unlock -> abort -> wait -> clear -> disable.
 * Tightly-coupled multi-step sequence. */
void mhdl_can_mh_tx_pq_abort(uint32_t base, uint32_t slot);

/* Read TX_PQ_STS0 (busy bits per slot). */
uint32_t mhdl_can_mh_tx_pq_get_sts0(uint32_t base);

/* Configure an RX FIFO queue.
 * Writes RX_FQ_START_ADDn, RX_FQ_SIZEn, and optionally RX_FQ_DC_START_ADDn
 * and RX_FQ_RD_ADD_PTn (for continuous mode). */
void mhdl_can_mh_set_rx_fifo_config(uint32_t base, uint32_t fifo,
                                     uint32_t start_addr, uint32_t max_desc,
                                     uint32_t dc_size_word, uint32_t dc_start,
                                     bool continuous_mode);

/* Set the enable bit for an RX FIFO in RX_FQ_CTRL2. */
void mhdl_can_mh_rx_fifo_enable(uint32_t base, uint32_t fifo);

/* Trigger RX FIFO start by writing RX_FQ_CTRL0. */
void mhdl_can_mh_rx_fifo_start(uint32_t base, uint32_t fifo);

/* Abort an RX FIFO: unlock -> abort -> wait -> clear -> disable.
 * Tightly-coupled multi-step sequence. */
void mhdl_can_mh_rx_fifo_abort(uint32_t base, uint32_t fifo);

/* Read RX_FQ_STS0 (busy + stop bits per RX FIFO). */
uint32_t mhdl_can_mh_rx_fifo_get_sts0(uint32_t base);

/* Read RX_FQ_INT_STS register. */
uint32_t mhdl_can_mh_rx_fifo_get_int_status(uint32_t base);

/* Write RX_FILTER_CTRL register. */
void mhdl_can_mh_set_rx_filter_ctrl(uint32_t base, uint32_t num_elements,
                                     uint32_t anmf, uint32_t anff,
                                     uint32_t anmf_fifo, uint32_t threshold);

/* Write a 32-bit word to Local Memory (LMEM) at the given offset. */
void mhdl_can_mh_write_lmem(uint32_t lmem_base, uint32_t offset, uint32_t value);

/* Read a 32-bit word from Local Memory (LMEM) at the given offset. */
uint32_t mhdl_can_mh_read_lmem(uint32_t lmem_base, uint32_t offset);

/* Read MH VERSION register. */
uint32_t mhdl_can_mh_get_version(uint32_t base);

/* Read MH_STS register (raw). */
uint32_t mhdl_can_mh_get_status(uint32_t base);

/* *************************************************************************
 * IRC — Interrupt Controller register operations
 * (Reference: xcand_irc.c)
 * ************************************************************************* */

/* Write CONTROL_FUNC_ENA register. */
void mhdl_can_irc_set_func_ena(uint32_t base, uint32_t mask);

/* Write CONTROL_ERR_ENA register. */
void mhdl_can_irc_set_err_ena(uint32_t base, uint32_t mask);

/* Write CONTROL_SAFETY_ENA register. */
void mhdl_can_irc_set_safety_ena(uint32_t base, uint32_t mask);

/* Read EVENT_FUNC_RAW register. */
uint32_t mhdl_can_irc_get_func_raw(uint32_t base);

/* Read EVENT_ERR_RAW register. */
uint32_t mhdl_can_irc_get_err_raw(uint32_t base);

/* Read EVENT_SAFETY_RAW register. */
uint32_t mhdl_can_irc_get_safety_raw(uint32_t base);

/* Clear functional interrupt flags by writing mask to CONTROL_FUNC_CLR. */
void mhdl_can_irc_clear_func(uint32_t base, uint32_t mask);

/* Clear error interrupt flags by writing mask to CONTROL_ERR_CLR. */
void mhdl_can_irc_clear_err(uint32_t base, uint32_t mask);

/* Clear safety interrupt flags by writing mask to CONTROL_SAFETY_CLR. */
void mhdl_can_irc_clear_safety(uint32_t base, uint32_t mask);

#endif /* MHDL_CAN_H */
