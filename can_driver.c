// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * can_driver.c - CAN Controller Driver Implementation
 *
 * Implementation of CAN CC/FD/XL driver following X_CAN v3.9 manual.
 * Programming Guidelines: Section 1.4.7, 1.5.5
 *
 * Copyright (C) 2026
 */

#include "can_driver.h"
#include <stdlib.h>
#include <string.h>

/*
 * Private constants
 */
#define CAN_POLL_TIMEOUT_COUNT		100000U
#define CAN_SHORT_DELAY			100U
#define CAN_PRT_UNLOCK_KEY1		0x000000CEU
#define CAN_PRT_UNLOCK_KEY2		0x000000ADU
#define CAN_DEFAULT_AXI_PARAMS		0x00000404U
#define CAN_DEFAULT_DMA_TIMEOUT		0xFFU
#define CAN_DEFAULT_MEM_TIMEOUT		0xFFU
#define CAN_DEFAULT_PRT_TIMEOUT		0x3FFFU
#define CAN_DEFAULT_PRESCALER		0x0U

/*
 * MH register bit field definitions (internal)
 */

/* MH_CTRL Register (0x004) */
#define CAN_MH_CTRL_START_POS		0U
#define CAN_MH_CTRL_START_MASK		0x00000001U

/* MH_CFG Register (0x008) */
#define CAN_MH_CFG_RX_CONT_DC_POS	0U
#define CAN_MH_CFG_RX_CONT_DC_MASK	0x00000001U
#define CAN_MH_CFG_MAX_RETRANS_POS	8U
#define CAN_MH_CFG_MAX_RETRANS_MASK	0x00000700U
#undef CAN_MH_CFG_INST_NUM_POS
#undef CAN_MH_CFG_INST_NUM_MASK
#define CAN_MH_CFG_INST_NUM_POS		16U
#define CAN_MH_CFG_INST_NUM_MASK	0x00070000U

/* MH_STS Register (0x00C) */
#define CAN_MH_STS_BUSY_POS		0U
#define CAN_MH_STS_BUSY_MASK		0x00000001U
#define CAN_MH_STS_ENABLE_POS		4U
#define CAN_MH_STS_ENABLE_MASK		0x00000010U
#define CAN_MH_STS_CLOCK_ACTIVE_POS	8U
#define CAN_MH_STS_CLOCK_ACTIVE_MASK	0x00000100U

/* MH_SFTY_CFG Register (0x010) */
#define CAN_MH_SFTY_CFG_DMA_TO_VAL_POS		0U
#define CAN_MH_SFTY_CFG_DMA_TO_VAL_MASK		0x000000FFU
#define CAN_MH_SFTY_CFG_MEM_TO_VAL_POS		8U
#define CAN_MH_SFTY_CFG_MEM_TO_VAL_MASK		0x0000FF00U
#define CAN_MH_SFTY_CFG_PRT_TO_VAL_POS		16U
#define CAN_MH_SFTY_CFG_PRT_TO_VAL_MASK		0x3FFF0000U
#define CAN_MH_SFTY_CFG_PRESCALER_POS		30U
#define CAN_MH_SFTY_CFG_PRESCALER_MASK		0xC0000000U

/* MH_SFTY_CTRL Register (0x014) */
#define CAN_MH_SFTY_CTRL_DMA_TO_EN_POS		8U
#define CAN_MH_SFTY_CTRL_DMA_TO_EN_MASK		0x00000100U
#define CAN_MH_SFTY_CTRL_MEM_TO_EN_POS		9U
#define CAN_MH_SFTY_CTRL_MEM_TO_EN_MASK		0x00000200U
#define CAN_MH_SFTY_CTRL_PRT_TO_EN_POS		10U
#define CAN_MH_SFTY_CTRL_PRT_TO_EN_MASK		0x00000400U

/* RX_FILTER_MEM_ADD Register (0x018) */
#define CAN_RX_FILTER_MEM_ADD_BASE_ADDR_POS	0U
#define CAN_RX_FILTER_MEM_ADD_BASE_ADDR_MASK	0x0000FFFFU

/* TX_DESC_MEM_ADD Register (0x01C) */
#define CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_POS	0U
#define CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_MASK	0x0000FFFFU
#define CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_POS	16U
#define CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_MASK	0xFFFF0000U

/* TX_FQ_SIZE Register */
#define CAN_TX_FQ_SIZE_MAX_DESC_POS	0U
#define CAN_TX_FQ_SIZE_MAX_DESC_MASK	0x000003FFU

/* RX_FQ_SIZE Register */
#define CAN_RX_FQ_SIZE_MAX_DESC_POS	0U
#define CAN_RX_FQ_SIZE_MAX_DESC_MASK	0x000003FFU
#define CAN_RX_FQ_SIZE_DC_SIZE_POS	16U
#define CAN_RX_FQ_SIZE_DC_SIZE_MASK	0x007F0000U

/* TX/RX_FQ_CTRL0 Register */
#define CAN_FQ_CTRL0_START_MASK		0x000000FFU

/* TX/RX_FQ_CTRL2 Register */
#define CAN_FQ_CTRL2_ENABLE_MASK	0x000000FFU

/* TX/RX_FQ_STS0 Register */
#define CAN_FQ_STS0_BUSY_MASK		0x000000FFU
#define CAN_FQ_STS0_STOP_MASK		0x0000FF00U

/*
 * PRT register bit fields
 */

/* PRT CTRL Register (0xA44) */
#define CAN_PRT_CTRL_STOP_POS		0U
#define CAN_PRT_CTRL_STOP_MASK		0x00000001U
#define CAN_PRT_CTRL_IMMD_POS		1U
#define CAN_PRT_CTRL_IMMD_MASK		0x00000002U
#define CAN_PRT_CTRL_STRT_POS		4U
#define CAN_PRT_CTRL_STRT_MASK		0x00000010U
#define CAN_PRT_CTRL_SRES_POS		8U
#define CAN_PRT_CTRL_SRES_MASK		0x00000100U

/*
 * Register offset arrays
 */
static const uint32_t tx_fq_start_add_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
	CAN_MH_TX_FQ_START_ADD0_OFFSET, CAN_MH_TX_FQ_START_ADD1_OFFSET,
	CAN_MH_TX_FQ_START_ADD2_OFFSET, CAN_MH_TX_FQ_START_ADD3_OFFSET,
	CAN_MH_TX_FQ_START_ADD4_OFFSET, CAN_MH_TX_FQ_START_ADD5_OFFSET,
	CAN_MH_TX_FQ_START_ADD6_OFFSET, CAN_MH_TX_FQ_START_ADD7_OFFSET
};

static const uint32_t tx_fq_size_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
	CAN_MH_TX_FQ_SIZE0_OFFSET, CAN_MH_TX_FQ_SIZE1_OFFSET,
	CAN_MH_TX_FQ_SIZE2_OFFSET, CAN_MH_TX_FQ_SIZE3_OFFSET,
	CAN_MH_TX_FQ_SIZE4_OFFSET, CAN_MH_TX_FQ_SIZE5_OFFSET,
	CAN_MH_TX_FQ_SIZE6_OFFSET, CAN_MH_TX_FQ_SIZE7_OFFSET
};

static const uint32_t rx_fq_start_add_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
	CAN_MH_RX_FQ_START_ADD0_OFFSET, CAN_MH_RX_FQ_START_ADD1_OFFSET,
	CAN_MH_RX_FQ_START_ADD2_OFFSET, CAN_MH_RX_FQ_START_ADD3_OFFSET,
	CAN_MH_RX_FQ_START_ADD4_OFFSET, CAN_MH_RX_FQ_START_ADD5_OFFSET,
	CAN_MH_RX_FQ_START_ADD6_OFFSET, CAN_MH_RX_FQ_START_ADD7_OFFSET
};

static const uint32_t rx_fq_size_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
	CAN_MH_RX_FQ_SIZE0_OFFSET, CAN_MH_RX_FQ_SIZE1_OFFSET,
	CAN_MH_RX_FQ_SIZE2_OFFSET, CAN_MH_RX_FQ_SIZE3_OFFSET,
	CAN_MH_RX_FQ_SIZE4_OFFSET, CAN_MH_RX_FQ_SIZE5_OFFSET,
	CAN_MH_RX_FQ_SIZE6_OFFSET, CAN_MH_RX_FQ_SIZE7_OFFSET
};

static const uint32_t rx_fq_dc_start_add_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
	CAN_MH_RX_FQ_DC_START_ADD0_OFFSET, CAN_MH_RX_FQ_DC_START_ADD1_OFFSET,
	CAN_MH_RX_FQ_DC_START_ADD2_OFFSET, CAN_MH_RX_FQ_DC_START_ADD3_OFFSET,
	CAN_MH_RX_FQ_DC_START_ADD4_OFFSET, CAN_MH_RX_FQ_DC_START_ADD5_OFFSET,
	CAN_MH_RX_FQ_DC_START_ADD6_OFFSET, CAN_MH_RX_FQ_DC_START_ADD7_OFFSET
};

static const uint32_t rx_fq_rd_add_pt_offset[CAN_RX_FIFO_QUEUE_COUNT] = {
	CAN_MH_RX_FQ_RD_ADD_PT0_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT1_OFFSET,
	CAN_MH_RX_FQ_RD_ADD_PT2_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT3_OFFSET,
	CAN_MH_RX_FQ_RD_ADD_PT4_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT5_OFFSET,
	CAN_MH_RX_FQ_RD_ADD_PT6_OFFSET, CAN_MH_RX_FQ_RD_ADD_PT7_OFFSET
};

/*
 * Private function prototypes
 */
static enum can_error can_wait_for_clock_active(uint32_t base, uint32_t timeout);
static enum can_error can_prt_software_reset(uint32_t base);
static enum can_error can_configure_mh_global(uint32_t base,
					      const struct can_config *cfg);
static enum can_error can_configure_mh_safety(uint32_t base,
					      const struct can_config *cfg);
static enum can_error can_configure_rx_filter(uint32_t base,
					      const struct can_config *cfg);
static enum can_error can_configure_tx_fifo_queues(uint32_t base,
						   const struct can_config *cfg);
static enum can_error can_configure_rx_fifo_queues(uint32_t base,
						   const struct can_config *cfg);
static enum can_error can_start_mh(uint32_t base, uint32_t timeout);
static enum can_error can_start_rx_fifo_queues(uint32_t base,
					       const struct can_config *cfg);
static enum can_error can_start_tx_fifo_queues(uint32_t base,
					       const struct can_config *cfg);

/*
 * can_init - Initialize CAN controller
 * @config: Pointer to configuration structure
 *
 * Follows programming guidelines from Manual Section 1.4.7.1.
 *
 * Return: enum can_error code
 */
enum can_error can_init(const struct can_config *cfg)
{
	enum can_error status = CAN_ERROR_NONE;
	uint32_t base;

	if (!cfg)
		return CAN_ERROR_INVALID_PARAM;

	base = cfg->base_addr;

	/* Step 1: Verify MH core clock is active */
	status = can_wait_for_clock_active(base, CAN_POLL_TIMEOUT_COUNT);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 2: Perform PRT Software Reset */
	status = can_prt_software_reset(base);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 3: Configure MH Global Registers */
	status = can_configure_mh_global(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 4: Configure MH Safety Registers */
	status = can_configure_mh_safety(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 5: Configure RX Filter */
	status = can_configure_rx_filter(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 6: Configure RX FIFO Queues */
	status = can_configure_rx_fifo_queues(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 7: Configure TX FIFO Queues */
	status = can_configure_tx_fifo_queues(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 8: Start MH */
	status = can_start_mh(base, CAN_POLL_TIMEOUT_COUNT);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 9: Start RX FIFO Queues */
	status = can_start_rx_fifo_queues(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	/* Step 10: Start TX FIFO Queues */
	status = can_start_tx_fifo_queues(base, cfg);
	if (status != CAN_ERROR_NONE)
		return status;

	return CAN_ERROR_NONE;
}

/*
 * Private function implementations
 */

static enum can_error can_wait_for_clock_active(uint32_t base, uint32_t timeout)
{
	uint32_t reg_val;

	while (timeout > 0U) {
		reg_val = CAN_READ_REG(base, CAN_MH_STS_OFFSET);
		if (reg_val & CAN_MH_STS_CLOCK_ACTIVE_MASK)
			return CAN_ERROR_NONE;
		timeout--;
	}

	return CAN_ERROR_TIMEOUT;
}

static enum can_error can_prt_software_reset(uint32_t base)
{
	volatile uint32_t i;

	CAN_WRITE_REG(base, CAN_PRT_CTRL_OFFSET, CAN_PRT_CTRL_SRES_MASK);

	for (i = 0; i < CAN_SHORT_DELAY; i++)
		;

	return CAN_ERROR_NONE;
}

static enum can_error can_configure_mh_global(uint32_t base,
					      const struct can_config *cfg)
{
	uint32_t mh_cfg_val = 0U;
	uint8_t i;

	mh_cfg_val |= ((uint32_t)cfg->instance_num << CAN_MH_CFG_INST_NUM_POS) &
		      CAN_MH_CFG_INST_NUM_MASK;

	for (i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
		if (cfg->rx_fifo_queues[i].enabled &&
		    cfg->rx_fifo_queues[i].continuous) {
			mh_cfg_val |= CAN_MH_CFG_RX_CONT_DC_MASK;
			break;
		}
	}

	mh_cfg_val |= (0x7U << CAN_MH_CFG_MAX_RETRANS_POS) &
		      CAN_MH_CFG_MAX_RETRANS_MASK;

	CAN_WRITE_REG(base, CAN_MH_CFG_OFFSET, mh_cfg_val);
	CAN_WRITE_REG(base, CAN_MH_AXI_PARAMS_OFFSET, CAN_DEFAULT_AXI_PARAMS);
	CAN_WRITE_REG(base, CAN_MH_TX_STATISTICS_OFFSET, 0U);
	CAN_WRITE_REG(base, CAN_MH_RX_STATISTICS_OFFSET, 0U);

	return CAN_ERROR_NONE;
}

static enum can_error can_configure_mh_safety(uint32_t base,
					      const struct can_config *cfg)
{
	uint32_t sfty_cfg_val = 0U;
	uint32_t sfty_ctrl_val = 0U;

	sfty_cfg_val |= (CAN_DEFAULT_DMA_TIMEOUT <<
			 CAN_MH_SFTY_CFG_DMA_TO_VAL_POS) &
			CAN_MH_SFTY_CFG_DMA_TO_VAL_MASK;
	sfty_cfg_val |= (CAN_DEFAULT_MEM_TIMEOUT <<
			 CAN_MH_SFTY_CFG_MEM_TO_VAL_POS) &
			CAN_MH_SFTY_CFG_MEM_TO_VAL_MASK;
	sfty_cfg_val |= (CAN_DEFAULT_PRT_TIMEOUT <<
			 CAN_MH_SFTY_CFG_PRT_TO_VAL_POS) &
			CAN_MH_SFTY_CFG_PRT_TO_VAL_MASK;
	sfty_cfg_val |= (CAN_DEFAULT_PRESCALER <<
			 CAN_MH_SFTY_CFG_PRESCALER_POS) &
			CAN_MH_SFTY_CFG_PRESCALER_MASK;

	CAN_WRITE_REG(base, CAN_MH_SFTY_CFG_OFFSET, sfty_cfg_val);

	sfty_ctrl_val |= CAN_MH_SFTY_CTRL_DMA_TO_EN_MASK;
	sfty_ctrl_val |= CAN_MH_SFTY_CTRL_MEM_TO_EN_MASK;
	sfty_ctrl_val |= CAN_MH_SFTY_CTRL_PRT_TO_EN_MASK;

	if (cfg->tx_desc_crc_enable)
		sfty_ctrl_val |= CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_MASK;
	if (cfg->rx_desc_crc_enable)
		sfty_ctrl_val |= CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_MASK;

	CAN_WRITE_REG(base, CAN_MH_SFTY_CTRL_OFFSET, sfty_ctrl_val);

	return CAN_ERROR_NONE;
}

static enum can_error can_configure_rx_filter(uint32_t base,
					      const struct can_config *cfg)
{
	uint32_t filter_mem_add_val;

	filter_mem_add_val = cfg->rx_filter_base_addr &
			     CAN_RX_FILTER_MEM_ADD_BASE_ADDR_MASK;
	CAN_WRITE_REG(base, CAN_MH_RX_FILTER_MEM_ADD_OFFSET, filter_mem_add_val);

	if (cfg->rx_filter_count > 0U)
		CAN_WRITE_REG(base, CAN_MH_RX_FILTER_CTRL_OFFSET, 0x00000001U);

	return CAN_ERROR_NONE;
}

static enum can_error can_configure_tx_fifo_queues(uint32_t base,
						   const struct can_config *cfg)
{
	uint32_t tx_desc_mem_add_val;
	uint32_t size_val;
	uint8_t i;

	tx_desc_mem_add_val = cfg->tx_desc_base_addr &
			      CAN_TX_DESC_MEM_ADD_FQ_BASE_ADDR_MASK;
	tx_desc_mem_add_val |= ((cfg->txpq_start_addr << 16U) &
				CAN_TX_DESC_MEM_ADD_PQ_BASE_ADDR_MASK);

	CAN_WRITE_REG(base, CAN_MH_TX_DESC_MEM_ADD_OFFSET, tx_desc_mem_add_val);

	for (i = 0; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
		if (cfg->tx_fifo_queues[i].enabled) {
			CAN_WRITE_REG(base, tx_fq_start_add_offset[i],
				      cfg->tx_fifo_queues[i].start_addr);

			size_val = (uint32_t)cfg->tx_fifo_queues[i].size &
				   CAN_TX_FQ_SIZE_MAX_DESC_MASK;
			CAN_WRITE_REG(base, tx_fq_size_offset[i], size_val);
		}
	}

	return CAN_ERROR_NONE;
}

static enum can_error can_configure_rx_fifo_queues(uint32_t base,
						   const struct can_config *cfg)
{
	uint32_t size_val;
	uint8_t i;

	for (i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
		if (cfg->rx_fifo_queues[i].enabled) {
			CAN_WRITE_REG(base, rx_fq_start_add_offset[i],
				      cfg->rx_fifo_queues[i].start_addr);

			size_val = ((uint32_t)cfg->rx_fifo_queues[i].size &
				    CAN_RX_FQ_SIZE_MAX_DESC_MASK);
			size_val |= (((uint32_t)cfg->rx_fifo_queues[i].dc_size
				      << CAN_RX_FQ_SIZE_DC_SIZE_POS) &
				     CAN_RX_FQ_SIZE_DC_SIZE_MASK);
			CAN_WRITE_REG(base, rx_fq_size_offset[i], size_val);

			if (cfg->rx_fifo_queues[i].continuous) {
				CAN_WRITE_REG(base, rx_fq_dc_start_add_offset[i],
					cfg->rx_fifo_queues[i].dc_start_addr);
				CAN_WRITE_REG(base, rx_fq_rd_add_pt_offset[i],
					cfg->rx_fifo_queues[i].dc_start_addr &
					0xFFFFFFFCU);
			}
		}
	}

	return CAN_ERROR_NONE;
}

static enum can_error can_start_mh(uint32_t base, uint32_t timeout)
{
	(void)timeout;
	CAN_WRITE_REG(base, CAN_MH_CTRL_OFFSET, CAN_MH_CTRL_START_MASK);
	return CAN_ERROR_NONE;
}

static enum can_error can_start_rx_fifo_queues(uint32_t base,
					       const struct can_config *cfg)
{
	uint32_t enable_mask = 0U;
	uint32_t start_mask = 0U;
	uint8_t i;

	for (i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
		if (cfg->rx_fifo_queues[i].enabled) {
			enable_mask |= (1U << i);
			start_mask |= (1U << i);
		}
	}

	CAN_WRITE_REG(base, CAN_MH_RX_FQ_CTRL2_OFFSET, enable_mask);
	CAN_WRITE_REG(base, CAN_MH_RX_FQ_CTRL0_OFFSET, start_mask);

	return CAN_ERROR_NONE;
}

static enum can_error can_start_tx_fifo_queues(uint32_t base,
					       const struct can_config *cfg)
{
	uint32_t enable_mask = 0U;
	uint32_t start_mask = 0U;
	uint8_t i;

	for (i = 0; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
		if (cfg->tx_fifo_queues[i].enabled) {
			enable_mask |= (1U << i);
			start_mask |= (1U << i);
		}
	}

	CAN_WRITE_REG(base, CAN_MH_TX_FQ_CTRL2_OFFSET, enable_mask);
	CAN_WRITE_REG(base, CAN_MH_TX_FQ_CTRL0_OFFSET, start_mask);

	return CAN_ERROR_NONE;
}
