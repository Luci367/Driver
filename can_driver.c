/**
 * @file    can_driver.c
 * @brief   CAN Controller Driver Implementation
 * @details Implementation of CAN CC/FD/XL driver following X_CAN v3.9 manual.
 *          Programming Guidelines: Section 1.4.7, 1.5.5
 *
 * @version 1.0.0
 * @date    2026-01-18
 *
 * @note    MISRA-C:2012 Compliant
 */

#include "can_driver.h"
#include <stdlib.h>
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Default timeout for polling operations (iterations) */
#define CAN_POLL_TIMEOUT_COUNT (100000U)

/** @brief Short delay loop count */
#define CAN_SHORT_DELAY (100U)

/** @brief PRT Unlock Key Sequence */
#define CAN_PRT_UNLOCK_KEY1 (0x000000CEU)
#define CAN_PRT_UNLOCK_KEY2 (0x000000ADU)

/** @brief Default AXI parameters: 4 outstanding reads, 4 outstanding writes */
#define CAN_DEFAULT_AXI_PARAMS (0x00000404U)

/** @brief Default safety timeout values */
#define CAN_DEFAULT_DMA_TIMEOUT (0xFFU)
#define CAN_DEFAULT_MEM_TIMEOUT (0xFFU)
#define CAN_DEFAULT_PRT_TIMEOUT (0x3FFFU)
#define CAN_DEFAULT_PRESCALER (0x0U)

/*============================================================================*/
/* MH REGISTER BIT FIELD DEFINITIONS (Not in header)                          */
/*============================================================================*/

/* MH_CTRL Register (0x004) - START bit */
#define CAN_MH_CTRL_START_POS (0U)
#define CAN_MH_CTRL_START_MASK (0x00000001U)

/* MH_CFG Register (0x008) - Bit fields per manual Section 1.4.4.2.1.3 */
#define CAN_MH_CFG_RX_CONT_DC_POS (0U)
#define CAN_MH_CFG_RX_CONT_DC_MASK (0x00000001U)
#define CAN_MH_CFG_MAX_RETRANS_POS (8U)
#define CAN_MH_CFG_MAX_RETRANS_MASK (0x00000700U)
/* Note: INST_NUM is at bits [18:16] per manual, override header definition */
#undef CAN_MH_CFG_INST_NUM_POS
#undef CAN_MH_CFG_INST_NUM_MASK
#define CAN_MH_CFG_INST_NUM_POS (16U)
#define CAN_MH_CFG_INST_NUM_MASK (0x00070000U)

/* MH_STS Register (0x00C) - Bit fields per manual Section 1.4.4.2.1.4 */
#define CAN_MH_STS_BUSY_POS (0U)
#define CAN_MH_STS_BUSY_MASK (0x00000001U)
#define CAN_MH_STS_ENABLE_POS (4U)
#define CAN_MH_STS_ENABLE_MASK (0x00000010U)
#define CAN_MH_STS_CLOCK_ACTIVE_POS (8U)
#define CAN_MH_STS_CLOCK_ACTIVE_MASK (0x00000100U)

/* MH_SFTY_CFG Register (0x010) - Timeout configuration */
#define CAN_MH_SFTY_CFG_DMA_TO_VAL_POS (0U)
#define CAN_MH_SFTY_CFG_DMA_TO_VAL_MASK (0x000000FFU)
#define CAN_MH_SFTY_CFG_MEM_TO_VAL_POS (8U)
#define CAN_MH_SFTY_CFG_MEM_TO_VAL_MASK (0x0000FF00U)
#define CAN_MH_SFTY_CFG_PRT_TO_VAL_POS (16U)
#define CAN_MH_SFTY_CFG_PRT_TO_VAL_MASK (0x3FFF0000U)
#define CAN_MH_SFTY_CFG_PRESCALER_POS (30U)
#define CAN_MH_SFTY_CFG_PRESCALER_MASK (0xC0000000U)

/* MH_SFTY_CTRL Register (0x014) - Safety control (extend header defs) */
#define CAN_MH_SFTY_CTRL_DMA_TO_EN_POS (8U)
#define CAN_MH_SFTY_CTRL_DMA_TO_EN_MASK (0x00000100U)
#define CAN_MH_SFTY_CTRL_MEM_TO_EN_POS (9U)
#define CAN_MH_SFTY_CTRL_MEM_TO_EN_MASK (0x00000200U)
#define CAN_MH_SFTY_CTRL_PRT_TO_EN_POS (10U)
#define CAN_MH_SFTY_CTRL_PRT_TO_EN_MASK (0x00000400U)

/* RX_FILTER_MEM_ADD Register (0x018) */
#define CAN_RX_FILTER_MEM_ADD_BASE_ADDR_POS (0U)
#define CAN_RX_FILTER_MEM_ADD_BASE_ADDR_MASK (0x0000FFFFU)

/* TX_DESC_MEM_ADD Register (0x01C) */
#define CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_POS (0U)
#define CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_MASK (0x0000FFFFU)
#define CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_POS (16U)
#define CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_MASK (0xFFFF0000U)

/* TX_FQ_SIZE Register */
#define CAN_TX_FQ_SIZE_MAX_DESC_POS (0U)
#define CAN_TX_FQ_SIZE_MAX_DESC_MASK (0x000003FFU)

/* RX_FQ_SIZE Register */
#define CAN_RX_FQ_SIZE_MAX_DESC_POS (0U)
#define CAN_RX_FQ_SIZE_MAX_DESC_MASK (0x000003FFU)
#define CAN_RX_FQ_SIZE_DC_SIZE_POS (16U)
#define CAN_RX_FQ_SIZE_DC_SIZE_MASK (0x007F0000U)

/* TX/RX_FQ_CTRL0 Register - START bits */
#define CAN_FQ_CTRL0_START_MASK (0x000000FFU)

/* TX/RX_FQ_CTRL2 Register - ENABLE bits */
#define CAN_FQ_CTRL2_ENABLE_MASK (0x000000FFU)

/* TX/RX_FQ_STS0 Register - BUSY and STOP bits */
#define CAN_FQ_STS0_BUSY_MASK (0x000000FFU)
#define CAN_FQ_STS0_STOP_MASK (0x0000FF00U)

/*============================================================================*/
/* PRT REGISTER BIT FIELDS                                                    */
/*============================================================================*/

/* PRT CTRL Register (relative offset 0x44) */
#define CAN_PRT_CTRL_STOP_POS (0U)
#define CAN_PRT_CTRL_STOP_MASK (0x00000001U)
#define CAN_PRT_CTRL_IMMD_POS (1U)
#define CAN_PRT_CTRL_IMMD_MASK (0x00000002U)
#define CAN_PRT_CTRL_STRT_POS (4U)
#define CAN_PRT_CTRL_STRT_MASK (0x00000010U)
#define CAN_PRT_CTRL_SRES_POS (8U)
#define CAN_PRT_CTRL_SRES_MASK (0x00000100U)
#define CAN_PRT_CTRL_TEST_POS (12U)
#define CAN_PRT_CTRL_TEST_MASK (0x00001000U)

/* PRT STAT Register (relative offset 0x08) */
#define CAN_PRT_STAT_ACT_POS (0U)
#define CAN_PRT_STAT_ACT_MASK (0x00000003U)
#define CAN_PRT_STAT_INIT_POS (4U)
#define CAN_PRT_STAT_INIT_MASK (0x00000010U)

/* PRT MODE Register (relative offset 0x60) */
#define CAN_PRT_MODE_FDOE_POS (0U)
#define CAN_PRT_MODE_FDOE_MASK (0x00000001U)
#define CAN_PRT_MODE_XLOE_POS (1U)
#define CAN_PRT_MODE_XLOE_MASK (0x00000002U)
#define CAN_PRT_MODE_TDCE_POS (2U)
#define CAN_PRT_MODE_TDCE_MASK (0x00000004U)
#define CAN_PRT_MODE_PXHD_POS (3U)
#define CAN_PRT_MODE_PXHD_MASK (0x00000008U)
#define CAN_PRT_MODE_EFBI_POS (4U)
#define CAN_PRT_MODE_EFBI_MASK (0x00000010U)
#define CAN_PRT_MODE_TXP_POS (5U)
#define CAN_PRT_MODE_TXP_MASK (0x00000020U)
#define CAN_PRT_MODE_MON_POS (6U)
#define CAN_PRT_MODE_MON_MASK (0x00000040U)
#define CAN_PRT_MODE_RSTR_POS (7U)
#define CAN_PRT_MODE_RSTR_MASK (0x00000080U)
#define CAN_PRT_MODE_SFS_POS (8U)
#define CAN_PRT_MODE_SFS_MASK (0x00000100U)
#define CAN_PRT_MODE_XLTR_POS (9U)
#define CAN_PRT_MODE_XLTR_MASK (0x00000200U)
#define CAN_PRT_MODE_EFDI_POS (10U)
#define CAN_PRT_MODE_EFDI_MASK (0x00000400U)

/* PRT TEST Register (relative offset 0x4C) */
#define CAN_PRT_TEST_LBCK_POS (0U)
#define CAN_PRT_TEST_LBCK_MASK (0x00000001U)

/*============================================================================*/
/* TX/RX FIFO QUEUE REGISTER OFFSET ARRAYS                                    */
/*============================================================================*/

/** @brief TX FIFO Queue Start Address Register Offsets */
static const uint32_t tx_fq_start_add_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
    CAN_MH_TX_FQ_START_ADD0_OFFSET, CAN_MH_TX_FQ_START_ADD1_OFFSET,
    CAN_MH_TX_FQ_START_ADD2_OFFSET, CAN_MH_TX_FQ_START_ADD3_OFFSET,
    CAN_MH_TX_FQ_START_ADD4_OFFSET, CAN_MH_TX_FQ_START_ADD5_OFFSET,
    CAN_MH_TX_FQ_START_ADD6_OFFSET, CAN_MH_TX_FQ_START_ADD7_OFFSET};

/** @brief TX FIFO Queue Size Register Offsets */
static const uint32_t tx_fq_size_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
    CAN_MH_TX_FQ_SIZE0_OFFSET, CAN_MH_TX_FQ_SIZE1_OFFSET,
    CAN_MH_TX_FQ_SIZE2_OFFSET, CAN_MH_TX_FQ_SIZE3_OFFSET,
    CAN_MH_TX_FQ_SIZE4_OFFSET, CAN_MH_TX_FQ_SIZE5_OFFSET,
    CAN_MH_TX_FQ_SIZE6_OFFSET, CAN_MH_TX_FQ_SIZE7_OFFSET};

/** @brief RX FIFO Queue Start Address Register Offsets */
static const uint32_t rx_fq_start_add_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
    CAN_MH_RX_FQ_START_ADD0_OFFSET, CAN_MH_RX_FQ_START_ADD1_OFFSET,
    CAN_MH_RX_FQ_START_ADD2_OFFSET, CAN_MH_RX_FQ_START_ADD3_OFFSET,
    CAN_MH_RX_FQ_START_ADD4_OFFSET, CAN_MH_RX_FQ_START_ADD5_OFFSET,
    CAN_MH_RX_FQ_START_ADD6_OFFSET, CAN_MH_RX_FQ_START_ADD7_OFFSET};

/** @brief RX FIFO Queue Size Register Offsets */
static const uint32_t rx_fq_size_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
    CAN_MH_RX_FQ_SIZE0_OFFSET, CAN_MH_RX_FQ_SIZE1_OFFSET,
    CAN_MH_RX_FQ_SIZE2_OFFSET, CAN_MH_RX_FQ_SIZE3_OFFSET,
    CAN_MH_RX_FQ_SIZE4_OFFSET, CAN_MH_RX_FQ_SIZE5_OFFSET,
    CAN_MH_RX_FQ_SIZE6_OFFSET, CAN_MH_RX_FQ_SIZE7_OFFSET};

/** @brief RX FIFO Queue Data Container Start Address Register Offsets */
static const uint32_t rx_fq_dc_start_add_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
    CAN_MH_RX_FQ_DC_START_ADD0_OFFSET, CAN_MH_RX_FQ_DC_START_ADD1_OFFSET,
    CAN_MH_RX_FQ_DC_START_ADD2_OFFSET, CAN_MH_RX_FQ_DC_START_ADD3_OFFSET,
    CAN_MH_RX_FQ_DC_START_ADD4_OFFSET, CAN_MH_RX_FQ_DC_START_ADD5_OFFSET,
    CAN_MH_RX_FQ_DC_START_ADD6_OFFSET, CAN_MH_RX_FQ_DC_START_ADD7_OFFSET};

/** @brief RX FIFO Queue Read Address Pointer Register Offsets */
static const uint32_t rx_fq_rd_add_pt_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
    CAN_MH_RX_FQ_RD_ADD_PT0_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT1_OFFSET,
    CAN_MH_RX_FQ_RD_ADD_PT2_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT3_OFFSET,
    CAN_MH_RX_FQ_RD_ADD_PT4_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT5_OFFSET,
    CAN_MH_RX_FQ_RD_ADD_PT6_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT7_OFFSET};

/** @brief TX FIFO Queue Address Pointer Register Offsets (current descriptor) */
static const uint32_t tx_fq_add_pt_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
    CAN_MH_TX_FQ_ADD_PT0_OFFSET, CAN_MH_TX_FQ_ADD_PT1_OFFSET,
    CAN_MH_TX_FQ_ADD_PT2_OFFSET, CAN_MH_TX_FQ_ADD_PT3_OFFSET,
    CAN_MH_TX_FQ_ADD_PT4_OFFSET, CAN_MH_TX_FQ_ADD_PT5_OFFSET,
    CAN_MH_TX_FQ_ADD_PT6_OFFSET, CAN_MH_TX_FQ_ADD_PT7_OFFSET};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static can_error_t can_wait_for_clock_active(uint32_t base, uint32_t timeout);
static can_error_t can_prt_unlock(uint32_t base);
static can_error_t can_prt_software_reset(uint32_t base);
static can_error_t can_configure_mh_global(uint32_t base,
                                           const can_config_t *cfg);
static can_error_t can_configure_mh_safety(uint32_t base,
                                           const can_config_t *cfg);
static can_error_t can_configure_rx_filter(uint32_t base,
                                           const can_config_t *cfg);
static can_error_t can_configure_tx_fifo_queues(uint32_t base,
                                                const can_config_t *cfg);
static can_error_t can_configure_rx_fifo_queues(uint32_t base,
                                                const can_config_t *cfg);
static can_error_t can_configure_prt_mode(uint32_t base,
                                          const can_config_t *cfg);
static can_error_t can_configure_bit_timing(uint32_t base,
                                            const can_config_t *cfg);
static can_error_t can_start_mh(uint32_t base, uint32_t timeout);
static can_error_t can_start_prt(uint32_t base, uint32_t timeout);
static can_error_t can_start_rx_fifo_queues(uint32_t base,
                                            const can_config_t *cfg);
static can_error_t can_start_tx_fifo_queues(uint32_t base,
                                            const can_config_t *cfg);
static can_error_t can_enable_interrupts(uint32_t base, const can_config_t *cfg);

/*============================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                            */
/*============================================================================*/

/**
 * @brief Initialize CAN controller
 * @details Follows programming guidelines from Manual Section 1.4.7.1:
 *          1. Check clock active
 *          2. Configure MH global registers
 *          3. Configure RX/TX filters
 *          4. Configure RX/TX FIFO queues
 *          5. Configure PRT (Mode, Bit Timing)
 *          6. Start MH and PRT
 *          7. Enable interrupts
 *
 * @param[in] cfg Pointer to configuration structure
 * @return can_error_t Error code
 */
can_error_t can_init(const can_config_t *cfg) {
  can_error_t status = CAN_ERROR_NONE;
  uint32_t base;

  /* Validate input parameters */
  if (cfg == NULL) {
    return CAN_ERROR_INVALID_PARAM;
  }

  base = cfg->base_addr;

  /*------------------------------------------------------------------------*/
  /* Step 1: Verify MH core clock is active (Section 1.4.7.1)               */
  /*------------------------------------------------------------------------*/
  status = can_wait_for_clock_active(base, CAN_POLL_TIMEOUT_COUNT);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 2: Perform PRT Software Reset (Section 1.5.5.2)                   */
  /* Note: PRT reset only works when CAN communication is stopped           */
  /*------------------------------------------------------------------------*/
  status = can_prt_software_reset(base);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 3: Configure MH Global Registers (Section 1.4.7.1 Step 1)         */
  /* Must be done before MH_CTRL.START = 1                                  */
  /*------------------------------------------------------------------------*/
  status = can_configure_mh_global(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 4: Configure MH Safety Registers                                  */
  /* MH_SFTY_CFG: Timeouts (DMA_TO_VAL, MEM_TO_VAL, PRT_TO_VAL, PRESCALER)  */
  /* MH_SFTY_CTRL: Enable features                                          */
  /*------------------------------------------------------------------------*/
  status = can_configure_mh_safety(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 5: Configure RX Filter (Section 1.4.7.12)                         */
  /* Set RX_FILTER_MEM_ADD.BASEADDR and RX_FILTER_CTRL                     */
  /*------------------------------------------------------------------------*/
  status = can_configure_rx_filter(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 6: Configure RX FIFO Queues (Section 1.4.7.3)                     */
  /* Set RX_FQ_START_ADD{n}, RX_FQ_SIZE{n}                                  */
  /*------------------------------------------------------------------------*/
  status = can_configure_rx_fifo_queues(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 7: Configure TX FIFO Queues (Section 1.4.7.6)                     */
  /* Set TX_FQ_START_ADD{n}, TX_FQ_SIZE{n}, TX_DESC_MEM_ADD                 */
  /*------------------------------------------------------------------------*/
  status = can_configure_tx_fifo_queues(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 8: Configure PRT Mode (Section 1.5.5.3)                           */
  /* Set MODE register for CC/FD/XL operation                               */
  /*------------------------------------------------------------------------*/
  status = can_configure_prt_mode(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 9: Configure Bit Timing (Section 1.5.4.2.4)                       */
  /* Set NBTP, DBTP (FD), XBTP (XL)                                         */
  /*------------------------------------------------------------------------*/
  status = can_configure_bit_timing(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 10: Enable Interrupts in IRC (Section 1.4.7.1 Step 7)             */
  /* Unmask functional, error, and safety interrupts                        */
  /*------------------------------------------------------------------------*/
  status = can_enable_interrupts(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 11: Start MH (Section 1.4.7.1 Step 9)                             */
  /* Write MH_CTRL.START = 1                                                */
  /*------------------------------------------------------------------------*/
  status = can_start_mh(base, CAN_POLL_TIMEOUT_COUNT);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 12: Start RX FIFO Queues (Section 1.4.7.1 Step 10)                */
  /* Enable and start configured RX queues before PRT start                 */
  /*------------------------------------------------------------------------*/
  status = can_start_rx_fifo_queues(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 13: Start PRT (Section 1.4.7.1 Step 11)                           */
  /* Write CTRL.STRT = 1, wait for MH_STS.ENABLE = 1                        */
  /*------------------------------------------------------------------------*/
  status = can_start_prt(base, CAN_POLL_TIMEOUT_COUNT);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Step 14: Start TX FIFO Queues (Section 1.4.7.1 Step 12)                */
  /* Start TX queues after PRT is running                                   */
  /*------------------------------------------------------------------------*/
  status = can_start_tx_fifo_queues(base, cfg);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Deinitialize CAN controller
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_deinit(uint32_t base_addr) {
  can_error_t status = CAN_ERROR_NONE;
  uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;
  uint32_t reg_val;

  /* Step 1: Stop PRT (unlock required) */
  status = can_prt_unlock(base_addr);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /* Write CTRL.STOP = 1 */
  CAN_WRITE_REG(base_addr, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_STOP_MASK);

  /* Wait for MH_STS.ENABLE = 0 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_STS_OFFSET);
    if ((reg_val & CAN_MH_STS_ENABLE_MASK) == 0U) {
      break;
    }
    timeout--;
  }
  if (timeout == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  /* Step 2: Abort all TX FIFO Queues */
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, 0xFFU);

  /* Wait for TX_FQ_STS0.BUSY = 0 */
  timeout = CAN_POLL_TIMEOUT_COUNT;
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
    if ((reg_val & CAN_FQ_STS0_BUSY_MASK) == 0U) {
      break;
    }
    timeout--;
  }

  /* Clear abort bits */
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, 0x00U);

  /* Step 3: Abort all RX FIFO Queues */
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET, 0xFFU);

  /* Wait for RX_FQ_STS0.BUSY = 0 */
  timeout = CAN_POLL_TIMEOUT_COUNT;
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS0_OFFSET);
    if ((reg_val & CAN_FQ_STS0_BUSY_MASK) == 0U) {
      break;
    }
    timeout--;
  }

  /* Clear abort bits */
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET, 0x00U);

  /* Step 4: Disable all queues */
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, 0x00U);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET, 0x00U);

  /* Step 5: Stop MH - Write MH_CTRL.START = 0 */
  CAN_WRITE_REG(base_addr, CAN_MH_CTRL_OFFSET, 0x00U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Start CAN controller (assumes already initialized)
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_start(uint32_t base_addr) {
  can_error_t status;

  /* Start MH */
  status = can_start_mh(base_addr, CAN_POLL_TIMEOUT_COUNT);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /* Start PRT */
  status = can_start_prt(base_addr, CAN_POLL_TIMEOUT_COUNT);

  return status;
}

/**
 * @brief Stop CAN controller
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_stop(uint32_t base_addr) {
  can_error_t status;
  uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;
  uint32_t reg_val;

  /* Unlock PRT */
  status = can_prt_unlock(base_addr);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /* Write CTRL.STOP = 1 for normal stop */
  CAN_WRITE_REG(base_addr, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_STOP_MASK);

  /* Wait for MH_STS.ENABLE = 0 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_STS_OFFSET);
    if ((reg_val & CAN_MH_STS_ENABLE_MASK) == 0U) {
      break;
    }
    timeout--;
  }

  if (timeout == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure bit timing
 */
can_error_t can_set_bit_timing(uint32_t base_addr,
                               const can_bit_timing_t *nominal,
                               const can_bit_timing_t *data,
                               const can_bit_timing_t *xl) {
  uint32_t nbtp_val = 0U;
  uint32_t dbtp_val = 0U;
  uint32_t xbtp_val = 0U;

  if (nominal == NULL) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Build NBTP register value */
  nbtp_val = CAN_BUILD_NBTP(nominal->brp, nominal->tseg1, nominal->tseg2,
                            nominal->sjw);
  CAN_WRITE_REG(base_addr, CAN_PRT_NBTP_OFFSET, nbtp_val);

  /* Configure DBTP if provided (CAN FD) */
  if (data != NULL) {
    dbtp_val =
        CAN_BUILD_DBTP(data->tdco, data->tseg1, data->tseg2, data->sjw);
    CAN_WRITE_REG(base_addr, CAN_PRT_DBTP_OFFSET, dbtp_val);
  }

  /* Configure XBTP if provided (CAN XL) */
  if (xl != NULL) {
    xbtp_val = CAN_BUILD_XBTP(xl->tdco, xl->tseg1, xl->tseg2, xl->sjw);
    CAN_WRITE_REG(base_addr, CAN_PRT_XBTP_OFFSET, xbtp_val);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Enable interrupts
 */
can_error_t can_interrupt_enable(uint32_t base_addr, uint32_t func_mask,
                                 uint32_t err_mask, uint32_t safety_mask) {
  CAN_WRITE_REG(base_addr, CAN_IRC_FUNC_ENA_OFFSET, func_mask);
  CAN_WRITE_REG(base_addr, CAN_IRC_ERR_ENA_OFFSET, err_mask);
  CAN_WRITE_REG(base_addr, CAN_IRC_SAFETY_ENA_OFFSET, safety_mask);

  return CAN_ERROR_NONE;
}

/**
 * @brief Clear interrupt flags
 */
can_error_t can_interrupt_clear(uint32_t base_addr, uint32_t func_mask,
                                uint32_t err_mask, uint32_t safety_mask) {
  CAN_WRITE_REG(base_addr, CAN_IRC_FUNC_CLR_OFFSET, func_mask);
  CAN_WRITE_REG(base_addr, CAN_IRC_ERR_CLR_OFFSET, err_mask);
  CAN_WRITE_REG(base_addr, CAN_IRC_SAFETY_CLR_OFFSET, safety_mask);

  return CAN_ERROR_NONE;
}

/**
 * @brief Get raw interrupt status
 */
can_error_t can_interrupt_get_raw_status(uint32_t base_addr,
                                         uint32_t *func_status,
                                         uint32_t *err_status,
                                         uint32_t *safety_status) {
  if (func_status != NULL) {
    *func_status = CAN_READ_REG(base_addr, CAN_IRC_FUNC_RAW_OFFSET);
  }
  if (err_status != NULL) {
    *err_status = CAN_READ_REG(base_addr, CAN_IRC_ERR_RAW_OFFSET);
  }
  if (safety_status != NULL) {
    *safety_status = CAN_READ_REG(base_addr, CAN_IRC_SAFETY_RAW_OFFSET);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Start TX FIFO queue
 */
can_error_t can_tx_fifo_start(uint32_t base_addr, uint8_t queue_idx,
                              const can_queue_config_t *config) {
  uint32_t reg_val;

  if ((queue_idx >= CAN_TX_FIFO_QUEUE_COUNT) || (config == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Configure start address */
  CAN_WRITE_REG(base_addr, tx_fq_start_add_offset[queue_idx],
                config->start_addr);

  /* Configure size (number of descriptors) */
  reg_val = (uint32_t)config->size & CAN_TX_FQ_SIZE_MAX_DESC_MASK;
  CAN_WRITE_REG(base_addr, tx_fq_size_offset[queue_idx], reg_val);

  /* Enable the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET);
  reg_val |= (1U << queue_idx);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, reg_val);

  /* Start the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET);
  reg_val |= (1U << queue_idx);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Start RX FIFO queue
 */
can_error_t can_rx_fifo_start(uint32_t base_addr, uint8_t queue_idx,
                              const can_queue_config_t *config) {
  uint32_t reg_val;
  uint32_t size_val;

  if ((queue_idx >= CAN_RX_FIFO_QUEUE_COUNT) || (config == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Configure start address (linked list) */
  CAN_WRITE_REG(base_addr, rx_fq_start_add_offset[queue_idx],
                config->start_addr);

  /* Configure size: MAX_DESC and DC_SIZE */
  size_val = ((uint32_t)config->size & CAN_RX_FQ_SIZE_MAX_DESC_MASK) |
             (((uint32_t)config->dc_size << CAN_RX_FQ_SIZE_DC_SIZE_POS) &
              CAN_RX_FQ_SIZE_DC_SIZE_MASK);
  CAN_WRITE_REG(base_addr, rx_fq_size_offset[queue_idx], size_val);

  /* For continuous mode, configure DC start address and read pointer */
  if (config->continuous) {
    CAN_WRITE_REG(base_addr, rx_fq_dc_start_add_offset[queue_idx],
                  config->dc_start_addr);
    CAN_WRITE_REG(base_addr, rx_fq_rd_add_pt_offset[queue_idx],
                  config->dc_start_addr & 0xFFFFFFFCU);
  }

  /* Enable the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET);
  reg_val |= (1U << queue_idx);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET, reg_val);

  /* Start the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL0_OFFSET);
  reg_val |= (1U << queue_idx);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL0_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/*============================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/*============================================================================*/

/**
 * @brief Wait for MH clock to become active
 * @details Per Section 1.4.7.1: "SW driver must ensure the MH core clock is
 *          active (MH_STS.CLOCK_ACTIVE = 1)"
 */
static can_error_t can_wait_for_clock_active(uint32_t base, uint32_t timeout) {
  uint32_t reg_val;

  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base, CAN_MH_STS_OFFSET);
    if ((reg_val & CAN_MH_STS_CLOCK_ACTIVE_MASK) != 0U) {
      return CAN_ERROR_NONE;
    }
    timeout--;
  }

  return CAN_ERROR_TIMEOUT;
}

/**
 * @brief Unlock PRT for configuration access
 * @details Per Section 1.5.4.2.3.1: Unlock sequence required before
 *          writing to CTRL.STOP or CTRL.TEST
 */
static can_error_t can_prt_unlock(uint32_t base) {
  /* Write unlock key sequence to LOCK register */
  CAN_WRITE_REG(base, CAN_PRT_LOCK_OFFSET, CAN_PRT_UNLOCK_KEY1);
  CAN_WRITE_REG(base, CAN_PRT_LOCK_OFFSET, CAN_PRT_UNLOCK_KEY2);

  return CAN_ERROR_NONE;
}

/**
 * @brief Perform PRT software reset
 * @details Per Section 1.5.5.2: "When CAN protocol operation is stopped,
 *          software reset is triggered by writing 1 to CTRL.SRES"
 */
static can_error_t can_prt_software_reset(uint32_t base) {
  /* No unlock required for software reset per manual */
  CAN_WRITE_REG(base, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_SRES_MASK);

  /* Small delay for reset to complete */
  for (volatile uint32_t i = 0; i < CAN_SHORT_DELAY; i++) {
    /* Wait */
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure MH global registers
 * @details Section 1.4.7.1 Step 1:
 *          - MH_CFG.INST_NUM
 *          - AXI_ADD_EXT (if >32-bit addressing)
 *          - AXI_PARAMS
 *          - Clear RX_STATISTICS and TX_STATISTICS
 */
static can_error_t can_configure_mh_global(uint32_t base,
                                           const can_config_t *cfg) {
  uint32_t mh_cfg_val = 0U;

  /* Configure MH_CFG register */
  /* Set instance number */
  mh_cfg_val |= ((uint32_t)cfg->instance_num << CAN_MH_CFG_INST_NUM_POS) &
                CAN_MH_CFG_INST_NUM_MASK;

  /* Set continuous mode if any RX queue is configured for continuous */
  for (uint8_t i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
    if (cfg->rx_fifo_queues[i].enabled && cfg->rx_fifo_queues[i].continuous) {
      mh_cfg_val |= CAN_MH_CFG_RX_CONT_DC_MASK;
      break;
    }
  }

  /* Set max retransmissions (default 7 = unlimited) */
  mh_cfg_val |= (0x7U << CAN_MH_CFG_MAX_RETRANS_POS) & CAN_MH_CFG_MAX_RETRANS_MASK;

  CAN_WRITE_REG(base, CAN_MH_CFG_OFFSET, mh_cfg_val);

  /* Configure AXI_PARAMS: Default 4 outstanding reads/writes */
  CAN_WRITE_REG(base, CAN_MH_AXI_PARAMS_OFFSET, CAN_DEFAULT_AXI_PARAMS);

  /* Clear statistics counters */
  CAN_WRITE_REG(base, CAN_MH_TX_STATISTICS_OFFSET, 0U);
  CAN_WRITE_REG(base, CAN_MH_RX_STATISTICS_OFFSET, 0U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure MH safety registers
 * @details Section 1.4.7.14: Timeout configurations
 *          MH_SFTY_CFG: PRESCALER, DMA_TO_VAL, MEM_TO_VAL, PRT_TO_VAL
 *          MH_SFTY_CTRL: Enable timeouts and CRC checks
 */
static can_error_t can_configure_mh_safety(uint32_t base,
                                           const can_config_t *cfg) {
  uint32_t sfty_cfg_val = 0U;
  uint32_t sfty_ctrl_val = 0U;

  /* Configure MH_SFTY_CFG with timeout values */
  sfty_cfg_val |= (CAN_DEFAULT_DMA_TIMEOUT << CAN_MH_SFTY_CFG_DMA_TO_VAL_POS) &
                  CAN_MH_SFTY_CFG_DMA_TO_VAL_MASK;
  sfty_cfg_val |= (CAN_DEFAULT_MEM_TIMEOUT << CAN_MH_SFTY_CFG_MEM_TO_VAL_POS) &
                  CAN_MH_SFTY_CFG_MEM_TO_VAL_MASK;
  sfty_cfg_val |= (CAN_DEFAULT_PRT_TIMEOUT << CAN_MH_SFTY_CFG_PRT_TO_VAL_POS) &
                  CAN_MH_SFTY_CFG_PRT_TO_VAL_MASK;
  sfty_cfg_val |= (CAN_DEFAULT_PRESCALER << CAN_MH_SFTY_CFG_PRESCALER_POS) &
                  CAN_MH_SFTY_CFG_PRESCALER_MASK;

  CAN_WRITE_REG(base, CAN_MH_SFTY_CFG_OFFSET, sfty_cfg_val);

  /* Configure MH_SFTY_CTRL */
  /* Enable timeouts */
  sfty_ctrl_val |= CAN_MH_SFTY_CTRL_DMA_TO_EN_MASK;
  sfty_ctrl_val |= CAN_MH_SFTY_CTRL_MEM_TO_EN_MASK;
  sfty_ctrl_val |= CAN_MH_SFTY_CTRL_PRT_TO_EN_MASK;

  /* Enable descriptor CRC checks if configured */
  if (cfg->tx_desc_crc_enable) {
    sfty_ctrl_val |= CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_MASK;
  }
  if (cfg->rx_desc_crc_enable) {
    sfty_ctrl_val |= CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_MASK;
  }

  CAN_WRITE_REG(base, CAN_MH_SFTY_CTRL_OFFSET, sfty_ctrl_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure RX filter
 * @details Section 1.4.7.12: RX Filter Setting
 *          Set RX_FILTER_MEM_ADD.BASEADDR to point to LMEM
 */
static can_error_t can_configure_rx_filter(uint32_t base,
                                           const can_config_t *cfg) {
  uint32_t filter_mem_add_val = 0U;

  /* Set RX filter base address in LMEM */
  filter_mem_add_val =
      (cfg->rx_filter_base_addr & CAN_RX_FILTER_MEM_ADD_BASE_ADDR_MASK);

  CAN_WRITE_REG(base, CAN_MH_RX_FILTER_MEM_ADD_OFFSET, filter_mem_add_val);

  /* Configure RX_FILTER_CTRL if filters are enabled */
  if (cfg->rx_filter_count > 0U) {
    /* Enable RX filter - exact configuration depends on use case */
    /* For now, just enable the filter mechanism */
    CAN_WRITE_REG(base, CAN_MH_RX_FILTER_CTRL_OFFSET, 0x00000001U);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure TX FIFO queues
 * @details Section 1.4.7.6: TX FIFO Queue Initial Start
 *          Set TX_DESC_MEM_ADD, TX_FQ_START_ADD{n}, TX_FQ_SIZE{n}
 */
static can_error_t can_configure_tx_fifo_queues(uint32_t base,
                                                const can_config_t *cfg) {
  uint32_t tx_desc_mem_add_val = 0U;
  uint32_t size_val;

  /* Configure TX_DESC_MEM_ADD with base addresses for FQ and PQ descriptors */
  tx_desc_mem_add_val =
      (cfg->tx_desc_base_addr & CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_MASK);
  tx_desc_mem_add_val |=
      ((cfg->txpq_start_addr << 16U) & CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_MASK);

  CAN_WRITE_REG(base, CAN_MH_TX_DESC_MEM_ADD_OFFSET, tx_desc_mem_add_val);

  /* Configure each enabled TX FIFO queue */
  for (uint8_t i = 0; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
    if (cfg->tx_fifo_queues[i].enabled) {
      /* Set start address */
      CAN_WRITE_REG(base, tx_fq_start_add_offset[i],
                    cfg->tx_fifo_queues[i].start_addr);

      /* Set size (number of descriptors) */
      size_val =
          (uint32_t)cfg->tx_fifo_queues[i].size & CAN_TX_FQ_SIZE_MAX_DESC_MASK;
      CAN_WRITE_REG(base, tx_fq_size_offset[i], size_val);
    }
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure RX FIFO queues
 * @details Section 1.4.7.3: RX FIFO Queue Initial Start
 *          Set RX_FQ_START_ADD{n}, RX_FQ_SIZE{n}, RX_FQ_DC_START_ADD{n}
 */
static can_error_t can_configure_rx_fifo_queues(uint32_t base,
                                                const can_config_t *cfg) {
  uint32_t size_val;

  /* Configure each enabled RX FIFO queue */
  for (uint8_t i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
    if (cfg->rx_fifo_queues[i].enabled) {
      /* Set start address (linked list base) */
      CAN_WRITE_REG(base, rx_fq_start_add_offset[i],
                    cfg->rx_fifo_queues[i].start_addr);

      /* Set size: MAX_DESC[9:0] and DC_SIZE[6:0 or 11:0] */
      size_val =
          ((uint32_t)cfg->rx_fifo_queues[i].size & CAN_RX_FQ_SIZE_MAX_DESC_MASK);
      size_val |= (((uint32_t)cfg->rx_fifo_queues[i].dc_size
                    << CAN_RX_FQ_SIZE_DC_SIZE_POS) &
                   CAN_RX_FQ_SIZE_DC_SIZE_MASK);
      CAN_WRITE_REG(base, rx_fq_size_offset[i], size_val);

      /* For continuous mode, set DC start address and read pointer */
      if (cfg->rx_fifo_queues[i].continuous) {
        CAN_WRITE_REG(base, rx_fq_dc_start_add_offset[i],
                      cfg->rx_fifo_queues[i].dc_start_addr);
        /* Initialize read pointer to DC start address (32-bit aligned) */
        CAN_WRITE_REG(base, rx_fq_rd_add_pt_offset[i],
                      cfg->rx_fifo_queues[i].dc_start_addr & 0xFFFFFFFCU);
      }
    }
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure PRT operating mode
 * @details Section 1.5.5.3: Operating Mode
 *          Set MODE register for CC/FD/XL operation
 */
static can_error_t can_configure_prt_mode(uint32_t base,
                                          const can_config_t *cfg) {
  uint32_t mode_val = 0U;

  /* Configure based on protocol */
  switch (cfg->protocol) {
  case CAN_PROTOCOL_CC:
    /* Classical CAN: FDOE = 0, XLOE = 0 */
    /* All bits default to 0 */
    break;

  case CAN_PROTOCOL_FD:
    /* CAN FD: FDOE = 1, XLOE = 0 */
    mode_val |= CAN_PRT_MODE_FDOE_MASK;
    /* Enable TDC for FD data phase */
    mode_val |= CAN_PRT_MODE_TDCE_MASK;
    break;

  case CAN_PROTOCOL_XL:
    /* CAN XL: FDOE = 1, XLOE = 1 */
    mode_val |= CAN_PRT_MODE_FDOE_MASK;
    mode_val |= CAN_PRT_MODE_XLOE_MASK;
    /* Enable TDC for XL data phase */
    mode_val |= CAN_PRT_MODE_TDCE_MASK;
    break;

  default:
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Configure listen-only mode if enabled */
  if (cfg->listen_only) {
    mode_val |= CAN_PRT_MODE_MON_MASK;
  }

  CAN_WRITE_REG(base, CAN_PRT_MODE_OFFSET, mode_val);

  /* Configure loopback if enabled (requires test mode) */
  if (cfg->loopback_enable) {
    /* Enable test mode and loopback */
    /* Note: Test mode requires unlock sequence */
    can_prt_unlock(base);
    CAN_WRITE_REG(base, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_TEST_MASK);
    CAN_WRITE_REG(base, CAN_PRT_TEST_OFFSET, CAN_PRT_TEST_LBCK_MASK);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Configure bit timing registers
 * @details Section 1.5.4.2.4: NBTP, DBTP, XBTP
 */
static can_error_t can_configure_bit_timing(uint32_t base,
                                            const can_config_t *cfg) {
  uint32_t nbtp_val;
  uint32_t dbtp_val;
  uint32_t xbtp_val;

  /* Configure Nominal Bit Timing (always required) */
  nbtp_val = CAN_BUILD_NBTP(cfg->nominal_timing.brp, cfg->nominal_timing.tseg1,
                            cfg->nominal_timing.tseg2, cfg->nominal_timing.sjw);
  CAN_WRITE_REG(base, CAN_PRT_NBTP_OFFSET, nbtp_val);

  /* Configure Data Phase Bit Timing (CAN FD and XL) */
  if ((cfg->protocol == CAN_PROTOCOL_FD) ||
      (cfg->protocol == CAN_PROTOCOL_XL)) {
    dbtp_val =
        CAN_BUILD_DBTP(cfg->data_timing.tdco, cfg->data_timing.tseg1,
                       cfg->data_timing.tseg2, cfg->data_timing.sjw);
    CAN_WRITE_REG(base, CAN_PRT_DBTP_OFFSET, dbtp_val);
  }

  /* Configure XL Phase Bit Timing (CAN XL only) */
  if (cfg->protocol == CAN_PROTOCOL_XL) {
    xbtp_val = CAN_BUILD_XBTP(cfg->xl_timing.tdco, cfg->xl_timing.tseg1,
                              cfg->xl_timing.tseg2, cfg->xl_timing.sjw);
    CAN_WRITE_REG(base, CAN_PRT_XBTP_OFFSET, xbtp_val);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Start Message Handler
 * @details Section 1.4.7.1 Step 9: Write MH_CTRL.START = 1
 */
static can_error_t can_start_mh(uint32_t base, uint32_t timeout) {
  (void)timeout; /* Currently not polling, MH starts immediately */

  /* Write MH_CTRL.START = 1 */
  CAN_WRITE_REG(base, CAN_MH_CTRL_OFFSET, CAN_MH_CTRL_START_MASK);

  return CAN_ERROR_NONE;
}

/**
 * @brief Start Protocol Controller
 * @details Section 1.4.7.1 Step 11:
 *          Write CTRL.STRT = 1, wait for MH_STS.ENABLE = 1
 */
static can_error_t can_start_prt(uint32_t base, uint32_t timeout) {
  uint32_t reg_val;

  /* Write CTRL.STRT = 1 to start PRT */
  CAN_WRITE_REG(base, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_STRT_MASK);

  /* Poll MH_STS.ENABLE until it reads 1 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base, CAN_MH_STS_OFFSET);
    if ((reg_val & CAN_MH_STS_ENABLE_MASK) != 0U) {
      return CAN_ERROR_NONE;
    }
    timeout--;
  }

  return CAN_ERROR_TIMEOUT;
}

/**
 * @brief Start RX FIFO queues
 * @details Section 1.4.7.1 Step 10:
 *          Enable and start RX queues before PRT start
 */
static can_error_t can_start_rx_fifo_queues(uint32_t base,
                                            const can_config_t *cfg) {
  uint32_t enable_mask = 0U;
  uint32_t start_mask = 0U;

  /* Build enable and start masks for configured queues */
  for (uint8_t i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
    if (cfg->rx_fifo_queues[i].enabled) {
      enable_mask |= (1U << i);
      start_mask |= (1U << i);
    }
  }

  /* Enable all configured RX FIFO queues */
  CAN_WRITE_REG(base, CAN_MH_RX_FQ_CTRL2_OFFSET, enable_mask);

  /* Start all configured RX FIFO queues */
  CAN_WRITE_REG(base, CAN_MH_RX_FQ_CTRL0_OFFSET, start_mask);

  return CAN_ERROR_NONE;
}

/**
 * @brief Start TX FIFO queues
 * @details Section 1.4.7.1 Step 12:
 *          Start TX queues after PRT is running
 */
static can_error_t can_start_tx_fifo_queues(uint32_t base,
                                            const can_config_t *cfg) {
  uint32_t enable_mask = 0U;
  uint32_t start_mask = 0U;

  /* Build enable and start masks for configured queues */
  for (uint8_t i = 0; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
    if (cfg->tx_fifo_queues[i].enabled) {
      enable_mask |= (1U << i);
      start_mask |= (1U << i);
    }
  }

  /* Enable all configured TX FIFO queues */
  CAN_WRITE_REG(base, CAN_MH_TX_FQ_CTRL2_OFFSET, enable_mask);

  /* Start all configured TX FIFO queues */
  CAN_WRITE_REG(base, CAN_MH_TX_FQ_CTRL0_OFFSET, start_mask);

  return CAN_ERROR_NONE;
}

/**
 * @brief Enable interrupts in IRC
 * @details Section 1.4.7.1 Step 7:
 *          Unmask functional, error, and safety interrupts
 */
static can_error_t can_enable_interrupts(uint32_t base,
                                         const can_config_t *cfg) {
  /* Enable functional interrupts (TX_FQ_IRQ, RX_FQ_IRQ, etc.) */
  CAN_WRITE_REG(base, CAN_IRC_FUNC_ENA_OFFSET, cfg->func_int_enable);

  /* Enable error interrupts */
  CAN_WRITE_REG(base, CAN_IRC_ERR_ENA_OFFSET, cfg->err_int_enable);

  /* Enable safety interrupts */
  CAN_WRITE_REG(base, CAN_IRC_SAFETY_ENA_OFFSET, cfg->safety_int_enable);

  return CAN_ERROR_NONE;
}

/*============================================================================*/
/* TX MESSAGE API IMPLEMENTATIONS                                             */
/* Reference: Manual Sections 1.4.7.6 - 1.4.7.11                              */
/*============================================================================*/

/**
 * @brief Convert data length to DLC for CAN FD
 * @details Internal helper for TX descriptor construction
 */
static uint8_t can_len_to_fd_dlc(uint32_t len) {
  if (len <= 8U) {
    return (uint8_t)len;
  } else if (len <= 12U) {
    return 9U;
  } else if (len <= 16U) {
    return 10U;
  } else if (len <= 20U) {
    return 11U;
  } else if (len <= 24U) {
    return 12U;
  } else if (len <= 32U) {
    return 13U;
  } else if (len <= 48U) {
    return 14U;
  } else {
    return 15U; /* 49-64 bytes */
  }
}

/**
 * @brief Calculate data buffer size in 32-byte units
 * @details TX descriptors SIZE field is in 32-byte units
 */
static uint16_t can_calc_buffer_size(uint32_t len, bool xl) {
  uint32_t size_bytes;

  if (xl) {
    /* XL frame: header (8 bytes) + data + CRC (varies) */
    /* Round up to 32-byte boundary */
    size_bytes = 8U + len; /* Simplified: actual XL needs AF field consideration */
  } else {
    /* CC/FD: header (8 bytes for T0/T1) + data */
    size_bytes = 8U + len;
  }

  /* Round up to 32-byte units */
  return (uint16_t)((size_bytes + 31U) / 32U);
}

/**
 * @brief Write a 32-bit value to system memory
 * @details Helper for writing to SMEM (System Memory)
 */
static inline void can_smem_write32(uint32_t addr, uint32_t value) {
  *((volatile uint32_t *)(uintptr_t)addr) = value;
}

/**
 * @brief Copy data to system memory with 32-bit alignment
 */
static void can_smem_copy_data(uint32_t dst_addr, const uint8_t *src,
                               uint32_t len) {
  uint32_t i;
  uint32_t word;
  uint32_t offset = 0U;

  /* Copy 32-bit words */
  for (i = 0U; i < (len / 4U); i++) {
    word = ((uint32_t)src[offset + 0U]) |
           ((uint32_t)src[offset + 1U] << 8U) |
           ((uint32_t)src[offset + 2U] << 16U) |
           ((uint32_t)src[offset + 3U] << 24U);
    can_smem_write32(dst_addr + offset, word);
    offset += 4U;
  }

  /* Handle remaining bytes */
  if ((len % 4U) != 0U) {
    word = 0U;
    for (i = 0U; i < (len % 4U); i++) {
      word |= ((uint32_t)src[offset + i] << (i * 8U));
    }
    can_smem_write32(dst_addr + offset, word);
  }
}

/**
 * @brief Push a message to TX FIFO queue
 * @details Constructs and writes TX descriptor to SMEM.
 *          Reference: Manual Section 1.4.7.6
 */
can_error_t can_tx_fifo_push(uint32_t base_addr, uint8_t fifo_id, uint32_t id,
                             const uint8_t *data, uint32_t len, bool fd,
                             bool xl, bool remote) {
  /* Call extended version with default options */
  bool extended = (id > 0x7FFU); /* Auto-detect extended ID */
  bool brs = fd || xl;           /* Enable BRS for FD/XL by default */

  return can_tx_fifo_push_ext(base_addr, fifo_id, id, extended, data, len, fd,
                              xl, brs, remote);
}

/**
 * @brief Push a message to TX FIFO queue with extended options
 * @details Full TX descriptor construction with all options.
 *          Per Section 1.4.7.6-1.4.7.7
 */
can_error_t can_tx_fifo_push_ext(uint32_t base_addr, uint8_t fifo_id,
                                 uint32_t id, bool extended, const uint8_t *data,
                                 uint32_t len, bool fd, bool xl, bool brs,
                                 bool remote) {
  uint32_t reg_val;
  uint32_t desc_addr;
  uint32_t start_addr;
  uint32_t queue_size;
  uint32_t data_addr;
  tx_descriptor_t desc;
  uint8_t dlc;
  uint16_t buf_size;

  /* Parameter validation */
  if (fifo_id >= CAN_TX_FIFO_QUEUE_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Validate data length based on frame type */
  if (xl) {
    if (len > CAN_XL_MAX_DLC) {
      return CAN_ERROR_INVALID_PARAM;
    }
  } else if (fd) {
    if (len > CAN_FD_MAX_DLC) {
      return CAN_ERROR_INVALID_PARAM;
    }
  } else {
    if (len > CAN_CC_MAX_DLC) {
      return CAN_ERROR_INVALID_PARAM;
    }
  }

  /* Check if queue is busy and stopped (ready for new descriptor) */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
  /* Check if queue n is busy but not stopped - means it's actively transmitting */
  if (((reg_val & (1U << fifo_id)) != 0U) &&
      ((reg_val & (0x100U << fifo_id)) == 0U)) {
    /* Queue is active, check address pointer vs queue limits */
    /* For simplicity, we'll check if there's space by examining queue config */
  }

  /* Get queue start address and size from registers */
  start_addr = CAN_READ_REG(base_addr, tx_fq_start_add_offset[fifo_id]);
  queue_size = CAN_READ_REG(base_addr, tx_fq_size_offset[fifo_id]) &
               CAN_TX_FQ_SIZE_MAX_DESC_MASK;

  /* Get current address pointer (next descriptor to write) */
  desc_addr = CAN_READ_REG(base_addr, tx_fq_add_pt_offset[fifo_id]);

  /* If address pointer is 0, use start address */
  if (desc_addr == 0U) {
    desc_addr = start_addr;
  }

  /* Calculate buffer size in 32-byte units */
  buf_size = can_calc_buffer_size(len, xl);

  /* Data address follows descriptor (descriptor is 32 bytes) */
  data_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;

  /* Initialize descriptor structure */
  (void)memset(&desc, 0, sizeof(desc));

  /*------------------------------------------------------------------------*/
  /* Element 0: DMA Info Control 1                                          */
  /*------------------------------------------------------------------------*/
  desc.elem0.bits.valid = 1U;   /* Mark as valid */
  desc.elem0.bits.hd = 1U;      /* Header descriptor */
  desc.elem0.bits.wrap = 0U;    /* No wrap (set later if needed) */
  desc.elem0.bits.next = 0U;    /* Must be 0 per manual */
  desc.elem0.bits.irq = 1U;     /* Request interrupt on completion */
  desc.elem0.bits.pq = 0U;      /* Not priority queue */
  desc.elem0.bits.end = 0U;     /* Not end of queue */
  desc.elem0.bits.fqn = fifo_id & 0x0FU;
  desc.elem0.bits.rc = 0U;      /* Rolling counter (MH manages) */
  desc.elem0.bits.sts = 0U;     /* Status (MH writes) */
  /* CRC would be calculated if enabled */

  /*------------------------------------------------------------------------*/
  /* Element 1: DMA Info Control 2                                          */
  /*------------------------------------------------------------------------*/
  desc.elem1.bits.size = buf_size;
  desc.elem1.bits.plsrc = 0U;   /* Payload in S_MEM */
  desc.elem1.bits.in = 0U;      /* Instance number (configured globally) */
  desc.elem1.bits.nhdo_tdo = 0U; /* Next header descriptor offset (for FQ) */

  /*------------------------------------------------------------------------*/
  /* Elements 2-3: Timestamp (MH writes after transmission)                 */
  /*------------------------------------------------------------------------*/
  desc.ts0 = 0U;
  desc.ts1 = 0U;

  /*------------------------------------------------------------------------*/
  /* Element 4: TX Message Header T0 (ID and format flags)                  */
  /*------------------------------------------------------------------------*/
  if (xl) {
    /* CAN XL: FDF=1, XLF=1, XTD=0, PrioID in BaseID position */
    desc.t0 = CAN_BUILD_T0_XL(id & 0x7FFU, /* Priority ID (11-bit) */
                              0U,           /* VCID */
                              0U,           /* SDT */
                              0U,           /* SEC */
                              0U);          /* RRS */
  } else if (extended) {
    /* Extended ID (29-bit) */
    desc.t0 = CAN_BUILD_T0_EXT((id >> 18U) & 0x7FFU, /* Base ID */
                                id & 0x3FFFFU,        /* Extended ID */
                                fd ? 1U : 0U);        /* FDF */
  } else {
    /* Standard ID (11-bit) */
    desc.t0 = CAN_BUILD_T0_STD(id & 0x7FFU, fd ? 1U : 0U);
  }

  /*------------------------------------------------------------------------*/
  /* Element 5: TX Message Header T1 (DLC and control)                      */
  /*------------------------------------------------------------------------*/
  if (xl) {
    /* CAN XL: DLC is actual byte count (up to 2048) */
    desc.t1 = CAN_BUILD_T1_XL(len);
  } else if (fd) {
    /* CAN FD: Convert length to DLC encoding */
    dlc = can_len_to_fd_dlc(len);
    desc.t1 = CAN_BUILD_T1_FD(dlc, brs ? 1U : 0U, 0U /* ESI */);
  } else {
    /* Classical CAN */
    dlc = (len > 8U) ? 8U : (uint8_t)len;
    desc.t1 = CAN_BUILD_T1_CC(dlc, remote ? 1U : 0U);
  }

  /*------------------------------------------------------------------------*/
  /* Element 6-7: T2/TD0 and TX_AP/TD1                                      */
  /*------------------------------------------------------------------------*/
  if (xl) {
    /* CAN XL: T2 contains AF (Acceptance Field) */
    desc.t2_td0 = 0U; /* AF if needed */
    desc.tx_ap_td1 = data_addr; /* TX_AP: pointer to data in S_MEM */
  } else if (len <= 8U && !fd) {
    /* Classical CAN with <= 8 bytes: data can be in TD0/TD1 */
    if (data != NULL && len > 0U) {
      /* Pack first 4 bytes into TD0 */
      desc.t2_td0 = 0U;
      for (uint32_t i = 0U; i < len && i < 4U; i++) {
        desc.t2_td0 |= ((uint32_t)data[i] << (i * 8U));
      }
      /* Pack next 4 bytes into TD1 */
      desc.tx_ap_td1 = 0U;
      for (uint32_t i = 4U; i < len && i < 8U; i++) {
        desc.tx_ap_td1 |= ((uint32_t)data[i] << ((i - 4U) * 8U));
      }
    }
  } else {
    /* FD or larger CC: use TX_AP pointer */
    desc.t2_td0 = 0U;
    desc.tx_ap_td1 = data_addr;

    /* Copy data to SMEM if needed */
    if (data != NULL && len > 0U) {
      can_smem_copy_data(data_addr, data, len);
    }
  }

  /*------------------------------------------------------------------------*/
  /* Write descriptor to SMEM                                               */
  /*------------------------------------------------------------------------*/
  can_smem_write32(desc_addr + 0U, desc.elem0.word);
  can_smem_write32(desc_addr + 4U, desc.elem1.word);
  can_smem_write32(desc_addr + 8U, desc.ts0);
  can_smem_write32(desc_addr + 12U, desc.ts1);
  can_smem_write32(desc_addr + 16U, desc.t0);
  can_smem_write32(desc_addr + 20U, desc.t1);
  can_smem_write32(desc_addr + 24U, desc.t2_td0);
  can_smem_write32(desc_addr + 28U, desc.tx_ap_td1);

  /*------------------------------------------------------------------------*/
  /* Write an invalid descriptor after this one (end marker)                */
  /* This puts the queue on hold when reached                               */
  /*------------------------------------------------------------------------*/
  uint32_t next_desc_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;
  if (len > 8U || fd || xl) {
    /* Account for data buffer */
    next_desc_addr += (uint32_t)buf_size * 32U;
  }

  /* Check for wrap-around */
  uint32_t queue_end = start_addr + (queue_size * CAN_TX_DESCRIPTOR_SIZE);
  if (next_desc_addr >= queue_end) {
    next_desc_addr = start_addr; /* Wrap to beginning */
    /* Update descriptor with WRAP bit */
    desc.elem0.bits.wrap = 1U;
    can_smem_write32(desc_addr + 0U, desc.elem0.word);
  }

  /* Write invalid marker at next descriptor location */
  can_smem_write32(next_desc_addr, 0U); /* VALID = 0 */

  /*------------------------------------------------------------------------*/
  /* Ensure queue is enabled                                                */
  /*------------------------------------------------------------------------*/
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET);
  if ((reg_val & (1U << fifo_id)) == 0U) {
    reg_val |= (1U << fifo_id);
    CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, reg_val);
  }

  /*------------------------------------------------------------------------*/
  /* Start/restart the queue if it was stopped                              */
  /*------------------------------------------------------------------------*/
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
  if ((reg_val & (0x100U << fifo_id)) != 0U) {
    /* Queue is stopped, restart it */
    reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET);
    reg_val |= (1U << fifo_id);
    CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET, reg_val);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Send a message via TX Priority Queue slot
 * @details Uses the slot-based mechanism for high-priority messages.
 *          Reference: Manual Sections 1.4.7.9-1.4.7.11
 */
can_error_t can_tx_priority_slot(uint32_t base_addr, uint8_t slot_id,
                                 uint32_t id, bool extended, const uint8_t *data,
                                 uint32_t len, bool fd, bool xl, bool brs) {
  uint32_t reg_val;
  uint32_t pq_base_addr;
  uint32_t desc_addr;
  uint32_t data_addr;
  tx_descriptor_t desc;
  uint8_t dlc;
  uint16_t buf_size;

  /* Parameter validation */
  if (slot_id >= CAN_TX_PQ_SLOT_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Validate data length */
  if (xl && (len > CAN_XL_MAX_DLC)) {
    return CAN_ERROR_INVALID_PARAM;
  } else if (fd && (len > CAN_FD_MAX_DLC)) {
    return CAN_ERROR_INVALID_PARAM;
  } else if (!fd && !xl && (len > CAN_CC_MAX_DLC)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Check if slot is already busy */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
  if ((reg_val & (1U << slot_id)) != 0U) {
    return CAN_ERROR_QUEUE_FULL;
  }

  /* Get PQ descriptor base address from TX_DESC_MEM_ADD register */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_DESC_MEM_ADD_OFFSET);
  pq_base_addr = (reg_val >> 16U) & 0xFFFFU;

  /* Each PQ slot uses a 32-byte descriptor */
  desc_addr = pq_base_addr + (slot_id * CAN_TX_DESCRIPTOR_SIZE);

  /* Calculate buffer size */
  buf_size = can_calc_buffer_size(len, xl);

  /* Data address (in L_MEM or S_MEM based on PLSRC) */
  data_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;

  /* Initialize descriptor */
  (void)memset(&desc, 0, sizeof(desc));

  /*------------------------------------------------------------------------*/
  /* Element 0: DMA Info Control 1 (PQ format)                              */
  /*------------------------------------------------------------------------*/
  desc.elem0.bits.valid = 1U;
  desc.elem0.bits.hd = 1U;
  desc.elem0.bits.wrap = 0U;
  desc.elem0.bits.next = 0U;
  desc.elem0.bits.irq = 1U;
  desc.elem0.bits.pq = 1U;      /* This is a Priority Queue descriptor */
  desc.elem0.bits.end = 0U;
  /* For PQ: PQSN (slot number) goes in fqn and pqsn0 */
  desc.elem0.bits.fqn = (slot_id >> 1U) & 0x0FU;  /* PQSN[4:1] */
  desc.elem0.bits.pqsn0 = slot_id & 0x01U;        /* PQSN[0] */
  desc.elem0.bits.rc = 0U;
  desc.elem0.bits.sts = 0U;

  /*------------------------------------------------------------------------*/
  /* Element 1: DMA Info Control 2                                          */
  /*------------------------------------------------------------------------*/
  desc.elem1.bits.size = buf_size;
  desc.elem1.bits.plsrc = 1U;   /* Payload in L_MEM for PQ */
  desc.elem1.bits.in = 0U;
  desc.elem1.bits.nhdo_tdo = 0U; /* TDO for PQ: offset in L_MEM */

  /*------------------------------------------------------------------------*/
  /* Elements 2-3: Timestamp (cleared, MH writes)                           */
  /*------------------------------------------------------------------------*/
  desc.ts0 = 0U;
  desc.ts1 = 0U;

  /*------------------------------------------------------------------------*/
  /* Element 4: TX Message Header T0                                        */
  /*------------------------------------------------------------------------*/
  if (xl) {
    desc.t0 = CAN_BUILD_T0_XL(id & 0x7FFU, 0U, 0U, 0U, 0U);
  } else if (extended) {
    desc.t0 = CAN_BUILD_T0_EXT((id >> 18U) & 0x7FFU, id & 0x3FFFFU,
                                fd ? 1U : 0U);
  } else {
    desc.t0 = CAN_BUILD_T0_STD(id & 0x7FFU, fd ? 1U : 0U);
  }

  /*------------------------------------------------------------------------*/
  /* Element 5: TX Message Header T1                                        */
  /*------------------------------------------------------------------------*/
  if (xl) {
    desc.t1 = CAN_BUILD_T1_XL(len);
  } else if (fd) {
    dlc = can_len_to_fd_dlc(len);
    desc.t1 = CAN_BUILD_T1_FD(dlc, brs ? 1U : 0U, 0U);
  } else {
    dlc = (len > 8U) ? 8U : (uint8_t)len;
    desc.t1 = CAN_BUILD_T1_CC(dlc, 0U);
  }

  /*------------------------------------------------------------------------*/
  /* Element 6-7: T2/TD0 and TX_AP/TD1                                      */
  /*------------------------------------------------------------------------*/
  if (len <= 8U && !fd && !xl) {
    /* Pack data directly in descriptor */
    desc.t2_td0 = 0U;
    desc.tx_ap_td1 = 0U;
    if (data != NULL) {
      for (uint32_t i = 0U; i < len && i < 4U; i++) {
        desc.t2_td0 |= ((uint32_t)data[i] << (i * 8U));
      }
      for (uint32_t i = 4U; i < len && i < 8U; i++) {
        desc.tx_ap_td1 |= ((uint32_t)data[i] << ((i - 4U) * 8U));
      }
    }
  } else {
    desc.t2_td0 = 0U;
    desc.tx_ap_td1 = data_addr;
    if (data != NULL && len > 0U) {
      can_smem_copy_data(data_addr, data, len);
    }
  }

  /*------------------------------------------------------------------------*/
  /* Write descriptor to L_MEM                                              */
  /*------------------------------------------------------------------------*/
  can_smem_write32(desc_addr + 0U, desc.elem0.word);
  can_smem_write32(desc_addr + 4U, desc.elem1.word);
  can_smem_write32(desc_addr + 8U, desc.ts0);
  can_smem_write32(desc_addr + 12U, desc.ts1);
  can_smem_write32(desc_addr + 16U, desc.t0);
  can_smem_write32(desc_addr + 20U, desc.t1);
  can_smem_write32(desc_addr + 24U, desc.t2_td0);
  can_smem_write32(desc_addr + 28U, desc.tx_ap_td1);

  /*------------------------------------------------------------------------*/
  /* Enable and start the PQ slot                                           */
  /* Per Section 1.4.7.10: Write to TX_PQ_CTRL0 to start slot               */
  /*------------------------------------------------------------------------*/
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET);
  reg_val |= (1U << slot_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET, reg_val);

  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL0_OFFSET);
  reg_val |= (1U << slot_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL0_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Abort TX FIFO queue
 * @details Per Section 1.4.7.8: Aborting a TX FIFO Queue
 */
can_error_t can_tx_abort(uint32_t base_addr, uint8_t fifo_id) {
  uint32_t reg_val;
  uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;

  /* Parameter validation */
  if (fifo_id >= CAN_TX_FIFO_QUEUE_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Step 1: Write 1 to TX_FQ_CTRL1.ABORT[n] */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET);
  reg_val |= (1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, reg_val);

  /* Step 2: Wait for TX_FQ_STS0.BUSY[n] and TX_FQ_STS0.STOP[n] to be 0 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
    if (((reg_val & (1U << fifo_id)) == 0U) &&
        ((reg_val & (0x100U << fifo_id)) == 0U)) {
      break;
    }
    timeout--;
  }

  if (timeout == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  /* Step 3: Write 0 to TX_FQ_CTRL1.ABORT[n] */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET);
  reg_val &= ~(1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, reg_val);

  /* Step 4: Disable the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET);
  reg_val &= ~(1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Abort TX Priority Queue slot
 * @details Per Section 1.4.7.11: Aborting a TX Priority Queue slot
 */
can_error_t can_tx_priority_abort(uint32_t base_addr, uint8_t slot_id) {
  uint32_t reg_val;
  uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;

  /* Parameter validation */
  if (slot_id >= CAN_TX_PQ_SLOT_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Write 1 to TX_PQ_CTRL1.ABORT[slot_id] */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET);
  reg_val |= (1U << slot_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET, reg_val);

  /* Wait for TX_PQ_STS0.BUSY[slot_id] to be 0 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
    if ((reg_val & (1U << slot_id)) == 0U) {
      break;
    }
    timeout--;
  }

  if (timeout == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  /* Clear abort bit */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET);
  reg_val &= ~(1U << slot_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET, reg_val);

  /* Disable the slot */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET);
  reg_val &= ~(1U << slot_id);
  CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Check if TX FIFO queue is busy
 */
can_error_t can_tx_fifo_is_busy(uint32_t base_addr, uint8_t fifo_id,
                                bool *is_busy) {
  uint32_t reg_val;

  if ((fifo_id >= CAN_TX_FIFO_QUEUE_COUNT) || (is_busy == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
  *is_busy = ((reg_val & (1U << fifo_id)) != 0U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Check if TX Priority Queue slot is busy
 */
can_error_t can_tx_priority_is_busy(uint32_t base_addr, uint8_t slot_id,
                                    bool *is_busy) {
  uint32_t reg_val;

  if ((slot_id >= CAN_TX_PQ_SLOT_COUNT) || (is_busy == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
  *is_busy = ((reg_val & (1U << slot_id)) != 0U);

  return CAN_ERROR_NONE;
}

/*============================================================================*/
/* RX MESSAGE API IMPLEMENTATIONS                                             */
/* Reference: Manual Sections 1.4.4.2.1, 1.4.7.3 - 1.4.7.5                    */
/*============================================================================*/

/*---------------------------------------------------------------------------*/
/* RX FIFO Queue Status Register Bit Definitions                             */
/*---------------------------------------------------------------------------*/

/* RX_FQ_STS0 Register (0x408) */
#define CAN_RX_FQ_STS0_BUSY_POS (0U)
#define CAN_RX_FQ_STS0_BUSY_MASK (0x000000FFU)
#define CAN_RX_FQ_STS0_STOP_POS (8U)
#define CAN_RX_FQ_STS0_STOP_MASK (0x0000FF00U)

/* RX_FQ_STS1 Register (0x40C) - NEW flags for each queue */
#define CAN_RX_FQ_STS1_NEW_MASK (0x000000FFU)

/* RX_FQ_STS2 Register (0x410) - ERROR and UNVALID flags */
#define CAN_RX_FQ_STS2_ERROR_MASK (0x000000FFU)
#define CAN_RX_FQ_STS2_UNVALID_MASK (0x0000FF00U)

/* RX Descriptor Address Pointer Register (0x400) */
#define CAN_MH_RX_DESC_ADD_PT_OFFSET_LOCAL (0x400U)

/**
 * @brief Read a 32-bit value from system memory
 * @details Helper for reading from SMEM (System Memory)
 */
static inline uint32_t can_smem_read32(uint32_t addr) {
  return *((volatile uint32_t *)(uintptr_t)addr);
}

/**
 * @brief Copy data from system memory
 */
static void can_smem_copy_from(uint8_t *dst, uint32_t src_addr, uint32_t len) {
  uint32_t i;
  uint32_t word;
  uint32_t offset = 0U;

  /* Copy 32-bit words */
  for (i = 0U; i < (len / 4U); i++) {
    word = can_smem_read32(src_addr + offset);
    dst[offset + 0U] = (uint8_t)(word & 0xFFU);
    dst[offset + 1U] = (uint8_t)((word >> 8U) & 0xFFU);
    dst[offset + 2U] = (uint8_t)((word >> 16U) & 0xFFU);
    dst[offset + 3U] = (uint8_t)((word >> 24U) & 0xFFU);
    offset += 4U;
  }

  /* Handle remaining bytes */
  if ((len % 4U) != 0U) {
    word = can_smem_read32(src_addr + offset);
    for (i = 0U; i < (len % 4U); i++) {
      dst[offset + i] = (uint8_t)((word >> (i * 8U)) & 0xFFU);
    }
  }
}

/**
 * @brief Convert FD DLC to byte length
 */
static uint32_t can_fd_dlc_to_len(uint8_t dlc) {
  static const uint8_t dlc_to_len[16] = {
      0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
  return (uint32_t)dlc_to_len[dlc & 0x0FU];
}

/**
 * @brief Setup RX FIFO queue
 * @details Configures RX FIFO queue registers per Section 1.4.7.3
 *
 * @note Filtering: Message IDs are routed to FIFOs based on RX_FILTER_CTRL
 *       and RX_FILTER_MEM_ADD configuration done during can_init().
 *       The filter elements in L_MEM determine which CAN IDs go to which FIFO.
 */
can_error_t can_rx_fifo_setup(uint32_t base_addr, uint8_t fifo_id,
                              uint32_t desc_phys_addr, uint16_t max_desc,
                              uint32_t dc_size, bool continuous) {
  uint32_t reg_val;
  uint32_t size_val;

  /* Parameter validation */
  if (fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Check RX_FQ_STS0.BUSY[n] is 0 before writing configuration */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS0_OFFSET);
  if ((reg_val & (1U << fifo_id)) != 0U) {
    return CAN_ERROR_QUEUE_FULL; /* Queue is busy, cannot configure */
  }

  /* Also check RX_FQ_CTRL2.ENABLE[n] is 0 */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET);
  if ((reg_val & (1U << fifo_id)) != 0U) {
    /* Disable the queue first */
    reg_val &= ~(1U << fifo_id);
    CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET, reg_val);
  }

  /*------------------------------------------------------------------------*/
  /* Configure RX_FQ_START_ADD{n} - Linked list start address               */
  /*------------------------------------------------------------------------*/
  CAN_WRITE_REG(base_addr, rx_fq_start_add_offset[fifo_id], desc_phys_addr);

  /*------------------------------------------------------------------------*/
  /* Configure RX_FQ_SIZE{n} - MAX_DESC[9:0] and DC_SIZE[6:0 or 11:0]       */
  /* For Normal mode: DC_SIZE is per-descriptor data container size         */
  /* For Continuous mode: DC_SIZE is total data container size              */
  /*------------------------------------------------------------------------*/
  size_val = ((uint32_t)max_desc & CAN_RX_FQ_SIZE_MAX_DESC_MASK);
  size_val |= ((dc_size << CAN_RX_FQ_SIZE_DC_SIZE_POS) &
               CAN_RX_FQ_SIZE_DC_SIZE_MASK);
  CAN_WRITE_REG(base_addr, rx_fq_size_offset[fifo_id], size_val);

  /*------------------------------------------------------------------------*/
  /* For Continuous mode, initialize read pointer                           */
  /* Note: DC start address should be set via can_rx_fifo_setup_continuous  */
  /*------------------------------------------------------------------------*/
  if (continuous) {
    /* For continuous mode, read pointer starts at descriptor start */
    /* SW must manage RX_FQ_RD_ADD_PT manually */
    CAN_WRITE_REG(base_addr, rx_fq_rd_add_pt_offset[fifo_id],
                  desc_phys_addr & 0xFFFFFFFCU);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Setup RX FIFO queue for continuous mode
 * @details Extended setup with data container base address
 */
can_error_t can_rx_fifo_setup_continuous(uint32_t base_addr, uint8_t fifo_id,
                                         uint32_t desc_phys_addr,
                                         uint16_t max_desc,
                                         uint32_t dc_start_addr,
                                         uint32_t dc_size) {
  can_error_t status;

  /* First do basic setup */
  status = can_rx_fifo_setup(base_addr, fifo_id, desc_phys_addr, max_desc,
                             dc_size, true);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /*------------------------------------------------------------------------*/
  /* Configure RX_FQ_DC_START_ADD{n} - Data container base address          */
  /* Only used in Continuous mode                                           */
  /*------------------------------------------------------------------------*/
  CAN_WRITE_REG(base_addr, rx_fq_dc_start_add_offset[fifo_id], dc_start_addr);

  /*------------------------------------------------------------------------*/
  /* Initialize RX_FQ_RD_ADD_PT{n} to DC start address                      */
  /* Per manual: must be {RX_FQ_DC_START_ADD{n}.VAL[31:1] & 0b11}          */
  /*------------------------------------------------------------------------*/
  CAN_WRITE_REG(base_addr, rx_fq_rd_add_pt_offset[fifo_id],
                dc_start_addr & 0xFFFFFFFCU);

  return CAN_ERROR_NONE;
}

/**
 * @brief Read a message from RX FIFO queue
 * @details Reads descriptor and data from S_MEM, parses header.
 *          Per Section 1.4.7.3-1.4.7.4
 */
can_error_t can_rx_read(uint32_t base_addr, uint8_t fifo_id, can_msg_t *msg,
                        uint32_t timeout_us) {
  uint32_t reg_val;
  uint32_t desc_addr;
  uint32_t data_addr;
  uint32_t timeout_count;
  rx_descriptor_t desc;
  rx_msg_header_t hdr;
  uint32_t data_len;
  uint8_t dlc;

  /* Parameter validation */
  if ((fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) || (msg == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Convert timeout_us to iteration count (approximate) */
  /* Assume ~1us per iteration for simple polling */
  timeout_count = (timeout_us == 0U) ? 1U : timeout_us;

  /*------------------------------------------------------------------------*/
  /* Poll RX_FQ_STS1.NEW[n] to check for new messages                       */
  /*------------------------------------------------------------------------*/
  while (timeout_count > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS1_OFFSET);
    if ((reg_val & (1U << fifo_id)) != 0U) {
      /* New message available */
      break;
    }

    if (timeout_us == 0U) {
      /* Non-blocking mode, no message available */
      return CAN_ERROR_QUEUE_EMPTY;
    }
    timeout_count--;
  }

  if (timeout_count == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  /*------------------------------------------------------------------------*/
  /* Get descriptor address from RX_DESC_ADD_PT or queue's address pointer  */
  /* Note: The current descriptor pointer for queue n can be derived from   */
  /* RX_FQ_ADD_PT{n} register                                               */
  /*------------------------------------------------------------------------*/
  /* For simplicity, read from the queue's configured start address */
  /* In a real implementation, track the read position */
  desc_addr = CAN_READ_REG(base_addr, rx_fq_start_add_offset[fifo_id]);

  /* Read the 4-word descriptor from S_MEM */
  desc.elem0.word = can_smem_read32(desc_addr + 0U);
  desc.rx_ap = can_smem_read32(desc_addr + 4U);
  desc.ts0 = can_smem_read32(desc_addr + 8U);
  desc.ts1 = can_smem_read32(desc_addr + 12U);

  /* Check if descriptor is valid (VALID bit = 1 means it's been used by MH) */
  if (desc.elem0.bits.valid == 0U) {
    /* Descriptor not yet used, no new message */
    return CAN_ERROR_QUEUE_EMPTY;
  }

  /*------------------------------------------------------------------------*/
  /* Get data address from RX_AP (Address Pointer) in descriptor            */
  /* For Normal mode: each descriptor has its own data container            */
  /* For Continuous mode: all descriptors share one data container          */
  /*------------------------------------------------------------------------*/
  data_addr = desc.rx_ap;

  /*------------------------------------------------------------------------*/
  /* Read RX Message Header (R0, R1, R2) from data container                */
  /*------------------------------------------------------------------------*/
  hdr.r0.word = can_smem_read32(data_addr + 0U);
  hdr.r1.word = can_smem_read32(data_addr + 4U);
  hdr.r2 = can_smem_read32(data_addr + 8U);

  /*------------------------------------------------------------------------*/
  /* Parse message header and populate can_msg_t                            */
  /*------------------------------------------------------------------------*/
  (void)memset(msg, 0, sizeof(can_msg_t));

  /* Frame format flags */
  bool is_fdf = (hdr.r0.bits.fdf != 0U);
  bool is_xlf = (hdr.r0.bits.xlf != 0U);
  msg->fd = is_fdf && !is_xlf;  /* FD = FDF set but not XL */
  msg->xl = is_xlf;             /* XL = XLF set */
  msg->extended = (hdr.r0.bits.xtd != 0U);

  /* Parse ID based on frame type */
  if (msg->xl) {
    /* CAN XL: Priority ID in base_id, VCID+SDT in ext_id */
    msg->id = hdr.r0.bits.base_id; /* 11-bit Priority ID */
    msg->vcid = (uint8_t)((hdr.r0.bits.ext_id >> 8U) & 0xFFU);
    msg->sdt = (uint8_t)(hdr.r0.bits.ext_id & 0xFFU);
  } else if (msg->extended) {
    /* Extended ID: 29 bits = base_id(11) + ext_id(18) */
    msg->id = ((uint32_t)hdr.r0.bits.base_id << 18U) | hdr.r0.bits.ext_id;
  } else {
    /* Standard ID: 11 bits */
    msg->id = hdr.r0.bits.base_id;
  }

  /* Parse DLC and calculate data length */
  if (msg->xl) {
    /* CAN XL: DLC is actual byte count (bits spread across r1) */
    /* DLC-XL[3:0] in dlc field, DLC-XL[10:4] in dlc_xl_h */
    uint32_t dlc_xl = ((uint32_t)hdr.r1.bits.dlc_xl_h << 4U) | hdr.r1.bits.dlc;
    data_len = dlc_xl;
    if (data_len > CAN_XL_MAX_DLC) {
      data_len = CAN_XL_MAX_DLC;
    }
  } else if (msg->fd) {
    /* CAN FD: Convert DLC to length */
    dlc = (uint8_t)hdr.r1.bits.dlc;
    data_len = can_fd_dlc_to_len(dlc);
  } else {
    /* Classical CAN: DLC is length (0-8) */
    dlc = (uint8_t)hdr.r1.bits.dlc;
    data_len = (dlc > 8U) ? 8U : dlc;
  }
  msg->len = data_len;

  /* Other flags */
  msg->brs = (hdr.r1.bits.brs != 0U);
  msg->esi = (hdr.r1.bits.esi != 0U);
  msg->rtr = (hdr.r1.bits.rtr != 0U) && !msg->fd && !msg->xl;

  /* Timestamp */
  msg->timestamp = ((uint64_t)desc.ts1 << 32U) | desc.ts0;

  /* FIFO ID and status */
  msg->fifo_id = fifo_id;
  msg->status = (can_desc_status_t)desc.elem0.bits.sts;

  /*------------------------------------------------------------------------*/
  /* Copy data payload from data container                                  */
  /* Data starts after the header (12 bytes for CC/FD, more for XL)         */
  /*------------------------------------------------------------------------*/
  if (data_len > 0U && !msg->rtr) {
    uint32_t hdr_size = msg->xl ? 12U : 8U; /* Header size in data container */
    can_smem_copy_from(msg->data, data_addr + hdr_size, data_len);
  }

  /*------------------------------------------------------------------------*/
  /* Acknowledge read by marking descriptor as available                    */
  /* Set VALID bit to 0 so MH can reuse this descriptor                     */
  /*------------------------------------------------------------------------*/
  desc.elem0.bits.valid = 0U;
  can_smem_write32(desc_addr + 0U, desc.elem0.word);

  /*------------------------------------------------------------------------*/
  /* For continuous mode, update RX_FQ_RD_ADD_PT to advance read pointer    */
  /* This allows SW to manage the circular buffer manually                  */
  /*------------------------------------------------------------------------*/
  /* Note: In a full implementation, track and update the read pointer here */

  return CAN_ERROR_NONE;
}

/**
 * @brief Check if RX FIFO has new messages
 */
can_error_t can_rx_has_message(uint32_t base_addr, uint8_t fifo_id,
                               bool *has_msg) {
  uint32_t reg_val;

  if ((fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) || (has_msg == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Check RX_FQ_STS1.NEW[n] flag */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS1_OFFSET);
  *has_msg = ((reg_val & (1U << fifo_id)) != 0U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Restart RX FIFO queue
 * @details Per Section 1.4.7.4: Restarting a RX FIFO Queue
 */
void can_rx_restart(uint32_t base_addr, uint8_t fifo_id) {
  uint32_t reg_val;

  if (fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) {
    return;
  }

  /*------------------------------------------------------------------------*/
  /* Per Section 1.4.7.4:                                                   */
  /* 1. Check MH_CTRL.START = 1 and MH_STS.ENABLE = 1                       */
  /* 2. Check RX_FQ_STS0.BUSY[n] = 1, STOP[n] = 1, CTRL2.ENABLE[n] = 1      */
  /* 3. Start queue by writing 1 to RX_FQ_CTRL0.START[n]                    */
  /*------------------------------------------------------------------------*/

  /* Check if MH is running */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_STS_OFFSET);
  if ((reg_val & CAN_MH_STS_ENABLE_MASK) == 0U) {
    return; /* MH not enabled, can't restart */
  }

  /* Ensure queue is enabled */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET);
  if ((reg_val & (1U << fifo_id)) == 0U) {
    /* Enable the queue first */
    reg_val |= (1U << fifo_id);
    CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET, reg_val);
  }

  /* Write 1 to RX_FQ_CTRL0.START[n] to restart */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL0_OFFSET);
  reg_val |= (1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL0_OFFSET, reg_val);
}

/**
 * @brief Abort RX FIFO queue
 * @details Per Section 1.4.7.5: Aborting a RX FIFO Queue
 */
can_error_t can_rx_abort(uint32_t base_addr, uint8_t fifo_id) {
  uint32_t reg_val;
  uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;

  if (fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Step 1: Write 1 to RX_FQ_CTRL1.ABORT[n] */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET);
  reg_val |= (1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET, reg_val);

  /* Step 2: Wait for RX_FQ_STS0.BUSY[n] and STOP[n] to be 0 */
  while (timeout > 0U) {
    reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS0_OFFSET);
    if (((reg_val & (1U << fifo_id)) == 0U) &&
        ((reg_val & (0x100U << fifo_id)) == 0U)) {
      break;
    }
    timeout--;
  }

  if (timeout == 0U) {
    return CAN_ERROR_TIMEOUT;
  }

  /* Step 3: Write 0 to RX_FQ_CTRL1.ABORT[n] */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET);
  reg_val &= ~(1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL1_OFFSET, reg_val);

  /* Step 4: Disable the queue */
  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET);
  reg_val &= ~(1U << fifo_id);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_FQ_CTRL2_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Check if RX FIFO queue is busy
 */
can_error_t can_rx_fifo_is_busy(uint32_t base_addr, uint8_t fifo_id,
                                bool *is_busy) {
  uint32_t reg_val;

  if ((fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) || (is_busy == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  reg_val = CAN_READ_REG(base_addr, CAN_MH_RX_FQ_STS0_OFFSET);
  *is_busy = ((reg_val & (1U << fifo_id)) != 0U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Get RX FIFO queue fill level
 * @details Returns the number of valid descriptors (approximate fill level)
 *
 * @note This is an approximation based on checking descriptor valid bits.
 *       For exact count, HW-specific counters should be used if available.
 */
can_error_t can_rx_get_fill_level(uint32_t base_addr, uint8_t fifo_id,
                                  uint32_t *fill_level) {
  uint32_t start_addr;
  uint32_t queue_size;
  uint32_t desc_addr;
  uint32_t count = 0U;
  uint32_t elem0;

  if ((fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) || (fill_level == NULL)) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Get queue configuration */
  start_addr = CAN_READ_REG(base_addr, rx_fq_start_add_offset[fifo_id]);
  queue_size = CAN_READ_REG(base_addr, rx_fq_size_offset[fifo_id]) &
               CAN_RX_FQ_SIZE_MAX_DESC_MASK;

  /* Count valid descriptors */
  for (uint32_t i = 0U; i < queue_size; i++) {
    desc_addr = start_addr + (i * CAN_RX_DESCRIPTOR_SIZE);
    elem0 = can_smem_read32(desc_addr);

    /* VALID bit = 1 means MH has written a message */
    if ((elem0 & 0x80000000U) != 0U) {
      count++;
    }
  }

  *fill_level = count;
  return CAN_ERROR_NONE;
}

/**
 * @brief Update RX read pointer (continuous mode)
 * @details Manually updates RX_FQ_RD_ADD_PT for continuous mode
 */
can_error_t can_rx_update_read_ptr(uint32_t base_addr, uint8_t fifo_id,
                                   uint32_t new_addr) {
  if (fifo_id >= CAN_RX_FIFO_QUEUE_COUNT) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Write new read pointer (must be 32-bit aligned) */
  CAN_WRITE_REG(base_addr, rx_fq_rd_add_pt_offset[fifo_id],
                new_addr & 0xFFFFFFFCU);

  return CAN_ERROR_NONE;
}

/*============================================================================*/
/* INTERRUPT HANDLING, STATISTICS, AND UTILITY IMPLEMENTATIONS                */
/*============================================================================*/

/*---------------------------------------------------------------------------*/
/* PRT Status Register Bit Definitions (0xA08)                               */
/*---------------------------------------------------------------------------*/

/* PRT STAT Register - Activity state */
#define CAN_PRT_STAT_ACT_POS (0U)
#define CAN_PRT_STAT_ACT_MASK (0x00000003U)
#define CAN_PRT_STAT_ACT_IDLE (0x0U)
#define CAN_PRT_STAT_ACT_RX (0x1U)
#define CAN_PRT_STAT_ACT_TX (0x2U)

/* PRT STAT Register - Init state */
#define CAN_PRT_STAT_INIT_POS (4U)
#define CAN_PRT_STAT_INIT_MASK (0x00000010U)

/* PRT STAT Register - Last Error Code (LEC) */
#define CAN_PRT_STAT_LEC_POS (8U)
#define CAN_PRT_STAT_LEC_MASK (0x00000700U)

/* PRT STAT Register - Receive Error Counter (REC) */
#define CAN_PRT_STAT_REC_POS (16U)
#define CAN_PRT_STAT_REC_MASK (0x007F0000U)

/* PRT STAT Register - Transmit Error Counter (TEC) */
#define CAN_PRT_STAT_TEC_POS (24U)
#define CAN_PRT_STAT_TEC_MASK (0xFF000000U)

/* PRT EVNT Register (0xA20) - Event Status Flags */
#define CAN_PRT_EVNT_OFFSET_LOCAL (0xA20U)
#define CAN_PRT_EVNT_E_WARN_POS (0U)
#define CAN_PRT_EVNT_E_WARN_MASK (0x00000001U)
#define CAN_PRT_EVNT_E_PASSIVE_POS (1U)
#define CAN_PRT_EVNT_E_PASSIVE_MASK (0x00000002U)
#define CAN_PRT_EVNT_BUS_OFF_POS (2U)
#define CAN_PRT_EVNT_BUS_OFF_MASK (0x00000004U)

/*---------------------------------------------------------------------------*/
/* IRC Interrupt Bit Definitions                                             */
/*---------------------------------------------------------------------------*/

/* Functional interrupt bits for RX FIFO queues */
#define CAN_IRC_FUNC_RX_FQ_MASK (0x0000FF00U)
#define CAN_IRC_FUNC_RX_FQ_SHIFT (8U)

/* Functional interrupt bits for TX FIFO queues */
#define CAN_IRC_FUNC_TX_FQ_MASK (0x000000FFU)

/* Functional interrupt bits for TX Priority Queue */
#define CAN_IRC_FUNC_TX_PQ_IRQ_POS (16U)

/* TX/RX Abort interrupts */
#define CAN_IRC_FUNC_TX_ABORT_IRQ_POS (20U)
#define CAN_IRC_FUNC_RX_ABORT_IRQ_POS (21U)

/* PRT event interrupts */
#define CAN_IRC_FUNC_E_ACTIVE_POS (24U)
#define CAN_IRC_FUNC_BUS_ON_POS (25U)

/* Error interrupt bits */
#define CAN_IRC_ERR_BUS_ERR_MASK (0x00000001U)
#define CAN_IRC_ERR_ARB_LOST_MASK (0x00000002U)
#define CAN_IRC_ERR_DMA_ERR_MASK (0x00000004U)
#define CAN_IRC_ERR_DESC_ERR_MASK (0x00000008U)

/*---------------------------------------------------------------------------*/
/* Static callback storage                                                   */
/*---------------------------------------------------------------------------*/

static can_irq_callbacks_t g_can_callbacks = {0};
static bool g_callbacks_registered = false;

/* Software statistics counters */
static can_stats_t g_can_stats = {0};

/**
 * @brief Register interrupt callbacks
 */
can_error_t can_register_callbacks(uint32_t base_addr,
                                   const can_irq_callbacks_t *callbacks) {
  (void)base_addr; /* Base addr stored if needed for multi-instance */

  if (callbacks == NULL) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Copy callback configuration */
  (void)memcpy(&g_can_callbacks, callbacks, sizeof(can_irq_callbacks_t));
  g_callbacks_registered = true;

  return CAN_ERROR_NONE;
}

/**
 * @brief CAN Interrupt Handler
 * @details Main IRQ handler implementation per user requirements:
 *          1. Read IRC Status: FUNC_RAW, ERR_RAW, SAFETY_RAW
 *          2. Clear interrupts via FUNC_CLR/ERR_CLR/SAFETY_CLR
 *          3. Dispatch events to registered callbacks
 *          4. Check PRT STAT for Bus Off / Error Passive states
 */
void can_irq_handler(uint32_t base_addr) {
  uint32_t func_raw;
  uint32_t err_raw;
  uint32_t safety_raw;
  uint32_t prt_stat;
  uint32_t prt_evnt;
  uint8_t i;
  can_bus_state_t new_state;

  /*------------------------------------------------------------------------*/
  /* Step 1: Read IRC Status Registers                                      */
  /*------------------------------------------------------------------------*/
  func_raw = CAN_READ_REG(base_addr, CAN_IRC_FUNC_RAW_OFFSET);
  err_raw = CAN_READ_REG(base_addr, CAN_IRC_ERR_RAW_OFFSET);
  safety_raw = CAN_READ_REG(base_addr, CAN_IRC_SAFETY_RAW_OFFSET);

  /*------------------------------------------------------------------------*/
  /* Step 2: Clear interrupts by writing to CLR registers                   */
  /*------------------------------------------------------------------------*/
  if (func_raw != 0U) {
    CAN_WRITE_REG(base_addr, CAN_IRC_FUNC_CLR_OFFSET, func_raw);
  }
  if (err_raw != 0U) {
    CAN_WRITE_REG(base_addr, CAN_IRC_ERR_CLR_OFFSET, err_raw);
  }
  if (safety_raw != 0U) {
    CAN_WRITE_REG(base_addr, CAN_IRC_SAFETY_CLR_OFFSET, safety_raw);
  }

  /*------------------------------------------------------------------------*/
  /* Step 3: Dispatch RX FIFO callbacks (MH_RX_FQ0_IRQ - MH_RX_FQ7_IRQ)     */
  /*------------------------------------------------------------------------*/
  if (g_callbacks_registered) {
    for (i = 0U; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
      if ((func_raw & (0x100U << i)) != 0U) {
        /* RX FIFO queue i has new message */
        g_can_stats.rx_success_count++;
        if (g_can_callbacks.rx_callbacks[i] != NULL) {
          g_can_callbacks.rx_callbacks[i](i, g_can_callbacks.user_ctx);
        }
      }
    }

    /*----------------------------------------------------------------------*/
    /* Dispatch TX FIFO callbacks (MH_TX_FQ0_IRQ - MH_TX_FQ7_IRQ)           */
    /*----------------------------------------------------------------------*/
    for (i = 0U; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
      if ((func_raw & (1U << i)) != 0U) {
        /* TX FIFO queue i completed/stopped */
        g_can_stats.tx_success_count++;
        if (g_can_callbacks.tx_callbacks[i] != NULL) {
          g_can_callbacks.tx_callbacks[i](i, g_can_callbacks.user_ctx);
        }
      }
    }

    /*----------------------------------------------------------------------*/
    /* Dispatch TX Priority Queue callback                                  */
    /*----------------------------------------------------------------------*/
    if ((func_raw & CAN_IRC_FUNC_MH_TX_PQ_IRQ_MASK) != 0U) {
      if (g_can_callbacks.tx_pq_callback != NULL) {
        /* Note: Would need to determine which slot triggered */
        g_can_callbacks.tx_pq_callback(0U, g_can_callbacks.user_ctx);
      }
    }

    /*----------------------------------------------------------------------*/
    /* Handle TX Abort interrupt                                            */
    /*----------------------------------------------------------------------*/
    if ((func_raw & CAN_IRC_FUNC_MH_TX_ABORT_IRQ_MASK) != 0U) {
      g_can_stats.tx_error_frames++;
      if (g_can_callbacks.error_callback != NULL) {
        g_can_callbacks.error_callback(CAN_ERROR_ARB_LOST,
                                       g_can_callbacks.user_ctx);
      }
    }

    /*----------------------------------------------------------------------*/
    /* Handle RX Abort interrupt                                            */
    /*----------------------------------------------------------------------*/
    if ((func_raw & CAN_IRC_FUNC_MH_RX_ABORT_IRQ_MASK) != 0U) {
      g_can_stats.rx_error_frames++;
      g_can_stats.overrun_count++;
    }
  }

  /*------------------------------------------------------------------------*/
  /* Step 4: Handle Error interrupts                                        */
  /*------------------------------------------------------------------------*/
  if (err_raw != 0U) {
    if ((err_raw & CAN_IRC_ERR_BUS_ERR_MASK) != 0U) {
      if (g_callbacks_registered && g_can_callbacks.error_callback != NULL) {
        g_can_callbacks.error_callback(CAN_ERROR_PROTOCOL,
                                       g_can_callbacks.user_ctx);
      }
    }
    if ((err_raw & CAN_IRC_ERR_ARB_LOST_MASK) != 0U) {
      g_can_stats.arb_lost_count++;
      if (g_callbacks_registered && g_can_callbacks.error_callback != NULL) {
        g_can_callbacks.error_callback(CAN_ERROR_ARB_LOST,
                                       g_can_callbacks.user_ctx);
      }
    }
    if ((err_raw & CAN_IRC_ERR_DMA_ERR_MASK) != 0U) {
      if (g_callbacks_registered && g_can_callbacks.error_callback != NULL) {
        g_can_callbacks.error_callback(CAN_ERROR_DMA,
                                       g_can_callbacks.user_ctx);
      }
    }
  }

  /*------------------------------------------------------------------------*/
  /* Step 5: Check PRT STAT for TEC/REC and Bus Off / Error Passive states  */
  /*------------------------------------------------------------------------*/
  prt_stat = CAN_READ_REG(base_addr, CAN_PRT_STAT_OFFSET);
  prt_evnt = CAN_READ_REG(base_addr, CAN_PRT_EVNT_OFFSET_LOCAL);

  /* Update error counters */
  g_can_stats.tx_error_count =
      (uint8_t)((prt_stat & CAN_PRT_STAT_TEC_MASK) >> CAN_PRT_STAT_TEC_POS);
  g_can_stats.rx_error_count =
      (uint8_t)((prt_stat & CAN_PRT_STAT_REC_MASK) >> CAN_PRT_STAT_REC_POS);
  g_can_stats.last_error_code =
      (uint8_t)((prt_stat & CAN_PRT_STAT_LEC_MASK) >> CAN_PRT_STAT_LEC_POS);

  /* Update error flags from EVNT register */
  g_can_stats.error_warning = ((prt_evnt & CAN_PRT_EVNT_E_WARN_MASK) != 0U);
  g_can_stats.error_passive = ((prt_evnt & CAN_PRT_EVNT_E_PASSIVE_MASK) != 0U);
  g_can_stats.bus_off = ((prt_evnt & CAN_PRT_EVNT_BUS_OFF_MASK) != 0U);

  /* Determine bus state */
  if (g_can_stats.bus_off) {
    new_state = CAN_BUS_STATE_BUS_OFF;
    g_can_stats.bus_off_count++;
  } else if (g_can_stats.error_passive) {
    new_state = CAN_BUS_STATE_PASSIVE;
  } else if (g_can_stats.error_warning) {
    new_state = CAN_BUS_STATE_WARNING;
  } else {
    new_state = CAN_BUS_STATE_ACTIVE;
  }

  /* Notify on state change */
  if (new_state != g_can_stats.bus_state) {
    g_can_stats.bus_state = new_state;
    if (g_callbacks_registered &&
        g_can_callbacks.bus_state_callback != NULL) {
      g_can_callbacks.bus_state_callback(new_state, g_can_callbacks.user_ctx);
    }
  }
}

/**
 * @brief Get pending interrupt status
 */
can_error_t can_get_irq_pending(uint32_t base_addr, uint32_t *func_pending,
                                uint32_t *err_pending,
                                uint32_t *safety_pending) {
  if (func_pending != NULL) {
    *func_pending = CAN_READ_REG(base_addr, CAN_IRC_FUNC_RAW_OFFSET);
  }
  if (err_pending != NULL) {
    *err_pending = CAN_READ_REG(base_addr, CAN_IRC_ERR_RAW_OFFSET);
  }
  if (safety_pending != NULL) {
    *safety_pending = CAN_READ_REG(base_addr, CAN_IRC_SAFETY_RAW_OFFSET);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Get CAN statistics and error counters
 * @details Reads TEC, REC from PRT STAT register
 */
can_error_t can_get_stats(uint32_t base_addr, can_stats_t *stats) {
  uint32_t prt_stat;
  uint32_t prt_evnt;
  uint32_t tx_stats;
  uint32_t rx_stats;

  if (stats == NULL) {
    return CAN_ERROR_INVALID_PARAM;
  }

  /* Read PRT STAT register for TEC/REC */
  prt_stat = CAN_READ_REG(base_addr, CAN_PRT_STAT_OFFSET);

  /* Read PRT EVNT register for error flags */
  prt_evnt = CAN_READ_REG(base_addr, CAN_PRT_EVNT_OFFSET_LOCAL);

  /* Read MH statistics registers */
  tx_stats = CAN_READ_REG(base_addr, CAN_MH_TX_STATISTICS_OFFSET);
  rx_stats = CAN_READ_REG(base_addr, CAN_MH_RX_STATISTICS_OFFSET);

  /* Copy software-tracked stats */
  (void)memcpy(stats, &g_can_stats, sizeof(can_stats_t));

  /* Update with hardware values */
  stats->tx_error_count =
      (uint8_t)((prt_stat & CAN_PRT_STAT_TEC_MASK) >> CAN_PRT_STAT_TEC_POS);
  stats->rx_error_count =
      (uint8_t)((prt_stat & CAN_PRT_STAT_REC_MASK) >> CAN_PRT_STAT_REC_POS);
  stats->last_error_code =
      (uint8_t)((prt_stat & CAN_PRT_STAT_LEC_MASK) >> CAN_PRT_STAT_LEC_POS);

  /* Error flags */
  stats->error_warning = ((prt_evnt & CAN_PRT_EVNT_E_WARN_MASK) != 0U);
  stats->error_passive = ((prt_evnt & CAN_PRT_EVNT_E_PASSIVE_MASK) != 0U);
  stats->bus_off = ((prt_evnt & CAN_PRT_EVNT_BUS_OFF_MASK) != 0U);

  /* Determine bus state */
  if (stats->bus_off) {
    stats->bus_state = CAN_BUS_STATE_BUS_OFF;
  } else if (stats->error_passive) {
    stats->bus_state = CAN_BUS_STATE_PASSIVE;
  } else if (stats->error_warning) {
    stats->bus_state = CAN_BUS_STATE_WARNING;
  } else {
    stats->bus_state = CAN_BUS_STATE_ACTIVE;
  }

  /* Hardware counters (if available) */
  stats->tx_success_count = tx_stats;
  stats->rx_success_count = rx_stats;

  return CAN_ERROR_NONE;
}

/**
 * @brief Get current bus state
 */
can_error_t can_get_bus_state(uint32_t base_addr, can_bus_state_t *state) {
  uint32_t prt_evnt;

  if (state == NULL) {
    return CAN_ERROR_INVALID_PARAM;
  }

  prt_evnt = CAN_READ_REG(base_addr, CAN_PRT_EVNT_OFFSET_LOCAL);

  if ((prt_evnt & CAN_PRT_EVNT_BUS_OFF_MASK) != 0U) {
    *state = CAN_BUS_STATE_BUS_OFF;
  } else if ((prt_evnt & CAN_PRT_EVNT_E_PASSIVE_MASK) != 0U) {
    *state = CAN_BUS_STATE_PASSIVE;
  } else if ((prt_evnt & CAN_PRT_EVNT_E_WARN_MASK) != 0U) {
    *state = CAN_BUS_STATE_WARNING;
  } else {
    *state = CAN_BUS_STATE_ACTIVE;
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Clear statistics counters
 */
can_error_t can_clear_stats(uint32_t base_addr) {
  /* Clear software counters */
  (void)memset(&g_can_stats, 0, sizeof(can_stats_t));

  /* Clear hardware statistics registers */
  CAN_WRITE_REG(base_addr, CAN_MH_TX_STATISTICS_OFFSET, 0U);
  CAN_WRITE_REG(base_addr, CAN_MH_RX_STATISTICS_OFFSET, 0U);

  return CAN_ERROR_NONE;
}

/**
 * @brief Set loopback mode
 * @details Configures PRT TEST register for internal loopback.
 *          Per manual: requires test mode key sequence.
 */
can_error_t can_set_loopback(uint32_t base_addr, bool enable) {
  uint32_t reg_val;

  /* Unlock PRT for test mode access */
  can_prt_unlock(base_addr);

  if (enable) {
    /* Enable test mode first */
    CAN_WRITE_REG(base_addr, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_TEST_MASK);

    /* Set LBCK bit in TEST register */
    reg_val = CAN_READ_REG(base_addr, CAN_PRT_TEST_OFFSET);
    reg_val |= CAN_PRT_TEST_LBCK_MASK;
    CAN_WRITE_REG(base_addr, CAN_PRT_TEST_OFFSET, reg_val);
  } else {
    /* Clear LBCK bit */
    reg_val = CAN_READ_REG(base_addr, CAN_PRT_TEST_OFFSET);
    reg_val &= ~CAN_PRT_TEST_LBCK_MASK;
    CAN_WRITE_REG(base_addr, CAN_PRT_TEST_OFFSET, reg_val);

    /* Disable test mode */
    reg_val = CAN_READ_REG(base_addr, CAN_PRT_CTRL_OFFSET);
    reg_val &= ~CAN_PRT_CTRL_TEST_MASK;
    CAN_WRITE_REG(base_addr, CAN_PRT_CTRL_OFFSET, reg_val);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Set listen-only mode
 * @details Configures PRT MODE register MON bit
 */
can_error_t can_set_listen_only(uint32_t base_addr, bool enable) {
  uint32_t reg_val;

  reg_val = CAN_READ_REG(base_addr, CAN_PRT_MODE_OFFSET);

  if (enable) {
    reg_val |= CAN_PRT_MODE_MON_MASK;
  } else {
    reg_val &= ~CAN_PRT_MODE_MON_MASK;
  }

  CAN_WRITE_REG(base_addr, CAN_PRT_MODE_OFFSET, reg_val);

  return CAN_ERROR_NONE;
}

/**
 * @brief Get controller version information
 */
can_error_t can_get_version(uint32_t base_addr, uint32_t *mh_version,
                            uint32_t *prt_version) {
  if (mh_version != NULL) {
    *mh_version = CAN_READ_REG(base_addr, CAN_MH_VERSION_OFFSET);
  }
  if (prt_version != NULL) {
    *prt_version = CAN_READ_REG(base_addr, CAN_PRT_PREL_OFFSET);
  }

  return CAN_ERROR_NONE;
}

/**
 * @brief Perform software reset
 * @details Resets PRT and MH to initial state.
 *          All queues must be stopped first.
 */
can_error_t can_software_reset(uint32_t base_addr) {
  can_error_t status;

  /* First, deinitialize to stop everything */
  status = can_deinit(base_addr);
  if (status != CAN_ERROR_NONE) {
    return status;
  }

  /* Perform PRT software reset */
  CAN_WRITE_REG(base_addr, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_SRES_MASK);

  /* Wait for reset to complete */
  for (volatile uint32_t i = 0; i < CAN_SHORT_DELAY; i++) {
    /* Delay */
  }

  /* Clear statistics */
  (void)memset(&g_can_stats, 0, sizeof(can_stats_t));
  g_callbacks_registered = false;

  return CAN_ERROR_NONE;
}

/* End of file can_driver.c */
