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

/*
 * TX API Helper Functions
 */

static const uint32_t tx_fq_add_pt_offset[CAN_TX_FIFO_QUEUE_COUNT] = {
	CAN_MH_TX_FQ_ADD_PT0_OFFSET, CAN_MH_TX_FQ_ADD_PT1_OFFSET,
	CAN_MH_TX_FQ_ADD_PT2_OFFSET, CAN_MH_TX_FQ_ADD_PT3_OFFSET,
	CAN_MH_TX_FQ_ADD_PT4_OFFSET, CAN_MH_TX_FQ_ADD_PT5_OFFSET,
	CAN_MH_TX_FQ_ADD_PT6_OFFSET, CAN_MH_TX_FQ_ADD_PT7_OFFSET
};

static uint8_t can_len_to_fd_dlc(uint32_t len)
{
	if (len <= 8U)
		return (uint8_t)len;
	if (len <= 12U)
		return 9U;
	if (len <= 16U)
		return 10U;
	if (len <= 20U)
		return 11U;
	if (len <= 24U)
		return 12U;
	if (len <= 32U)
		return 13U;
	if (len <= 48U)
		return 14U;
	return 15U;
}

static uint16_t can_calc_buffer_size(uint32_t len, bool xl)
{
	uint32_t size_bytes;

	if (xl)
		size_bytes = 8U + len;
	else
		size_bytes = 8U + len;

	return (uint16_t)((size_bytes + 31U) / 32U);
}

static inline void can_smem_write32(uint32_t addr, uint32_t value)
{
	*((volatile uint32_t *)(uintptr_t)addr) = value;
}

static void can_smem_copy_data(uint32_t dst_addr, const uint8_t *src,
			       uint32_t len)
{
	uint32_t i, word, offset = 0U;

	for (i = 0U; i < (len / 4U); i++) {
		word = ((uint32_t)src[offset + 0U]) |
		       ((uint32_t)src[offset + 1U] << 8U) |
		       ((uint32_t)src[offset + 2U] << 16U) |
		       ((uint32_t)src[offset + 3U] << 24U);
		can_smem_write32(dst_addr + offset, word);
		offset += 4U;
	}

	if ((len % 4U) != 0U) {
		word = 0U;
		for (i = 0U; i < (len % 4U); i++)
			word |= ((uint32_t)src[offset + i] << (i * 8U));
		can_smem_write32(dst_addr + offset, word);
	}
}

/*
 * can_tx_fifo_push - Push a message to TX FIFO queue
 * @base_addr: CAN controller base address
 * @fifo_id: TX FIFO queue index (0-7)
 * @id: CAN message identifier
 * @data: Pointer to data payload
 * @len: Data length in bytes
 * @fd: True for CAN FD frame
 * @xl: True for CAN XL frame
 * @remote: True for remote frame (CC only)
 *
 * Return: enum can_error code
 */
enum can_error can_tx_fifo_push(uint32_t base_addr, uint8_t fifo_id,
				uint32_t id, const uint8_t *data, uint32_t len,
				bool fd, bool xl, bool remote)
{
	bool extended = (id > 0x7FFU);
	bool brs = fd || xl;

	return can_tx_fifo_push_ext(base_addr, fifo_id, id, extended, data,
				    len, fd, xl, brs, remote);
}

/*
 * can_tx_fifo_push_ext - Push message with extended options
 * @base_addr: CAN controller base address
 * @fifo_id: TX FIFO queue index (0-7)
 * @id: CAN message identifier
 * @extended: True for 29-bit extended ID
 * @data: Pointer to data payload
 * @len: Data length in bytes
 * @fd: True for CAN FD frame
 * @xl: True for CAN XL frame
 * @brs: Bit Rate Switch
 * @remote: True for remote frame
 *
 * Return: enum can_error code
 */
enum can_error can_tx_fifo_push_ext(uint32_t base_addr, uint8_t fifo_id,
				    uint32_t id, bool extended,
				    const uint8_t *data, uint32_t len,
				    bool fd, bool xl, bool brs, bool remote)
{
	uint32_t reg_val, desc_addr, start_addr, queue_size, data_addr;
	uint32_t next_desc_addr, queue_end;
	struct can_tx_descriptor desc;
	uint8_t dlc;
	uint16_t buf_size;
	uint32_t i;

	if (fifo_id >= CAN_TX_FIFO_QUEUE_COUNT)
		return CAN_ERROR_INVALID_PARAM;

	if (xl) {
		if (len > CAN_XL_MAX_DLC)
			return CAN_ERROR_INVALID_PARAM;
	} else if (fd) {
		if (len > CAN_FD_MAX_DLC)
			return CAN_ERROR_INVALID_PARAM;
	} else {
		if (len > CAN_CC_MAX_DLC)
			return CAN_ERROR_INVALID_PARAM;
	}

	start_addr = CAN_READ_REG(base_addr, tx_fq_start_add_offset[fifo_id]);
	queue_size = CAN_READ_REG(base_addr, tx_fq_size_offset[fifo_id]) &
		     CAN_TX_FQ_SIZE_MAX_DESC_MASK;

	desc_addr = CAN_READ_REG(base_addr, tx_fq_add_pt_offset[fifo_id]);
	if (desc_addr == 0U)
		desc_addr = start_addr;

	buf_size = can_calc_buffer_size(len, xl);
	data_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;

	memset(&desc, 0, sizeof(desc));

	/* Element 0: DMA Info Control 1 */
	desc.elem0.bits.valid = 1U;
	desc.elem0.bits.hd = 1U;
	desc.elem0.bits.irq = 1U;
	desc.elem0.bits.fqn = fifo_id & 0x0FU;

	/* Element 1: DMA Info Control 2 */
	desc.elem1.bits.size = buf_size;

	/* Element 4: TX Message Header T0 */
	if (xl) {
		desc.t0 = CAN_BUILD_T0_XL(id & 0x7FFU, 0U, 0U, 0U, 0U);
	} else if (extended) {
		desc.t0 = CAN_BUILD_T0_EXT((id >> 18U) & 0x7FFU,
					   id & 0x3FFFFU, fd ? 1U : 0U);
	} else {
		desc.t0 = CAN_BUILD_T0_STD(id & 0x7FFU, fd ? 1U : 0U);
	}

	/* Element 5: TX Message Header T1 */
	if (xl) {
		desc.t1 = CAN_BUILD_T1_XL(len);
	} else if (fd) {
		dlc = can_len_to_fd_dlc(len);
		desc.t1 = CAN_BUILD_T1_FD(dlc, brs ? 1U : 0U, 0U);
	} else {
		dlc = (len > 8U) ? 8U : (uint8_t)len;
		desc.t1 = CAN_BUILD_T1_CC(dlc, remote ? 1U : 0U);
	}

	/* Element 6-7: Data */
	if (len <= 8U && !fd && !xl) {
		if (data && len > 0U) {
			desc.t2_td0 = 0U;
			for (i = 0U; i < len && i < 4U; i++)
				desc.t2_td0 |= ((uint32_t)data[i] << (i * 8U));
			desc.tx_ap_td1 = 0U;
			for (i = 4U; i < len && i < 8U; i++)
				desc.tx_ap_td1 |= ((uint32_t)data[i] <<
						   ((i - 4U) * 8U));
		}
	} else {
		desc.tx_ap_td1 = data_addr;
		if (data && len > 0U)
			can_smem_copy_data(data_addr, data, len);
	}

	/* Write descriptor to SMEM */
	can_smem_write32(desc_addr + 0U, desc.elem0.word);
	can_smem_write32(desc_addr + 4U, desc.elem1.word);
	can_smem_write32(desc_addr + 8U, desc.ts0);
	can_smem_write32(desc_addr + 12U, desc.ts1);
	can_smem_write32(desc_addr + 16U, desc.t0);
	can_smem_write32(desc_addr + 20U, desc.t1);
	can_smem_write32(desc_addr + 24U, desc.t2_td0);
	can_smem_write32(desc_addr + 28U, desc.tx_ap_td1);

	/* Calculate next descriptor address */
	next_desc_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;
	if (len > 8U || fd || xl)
		next_desc_addr += (uint32_t)buf_size * 32U;

	queue_end = start_addr + (queue_size * CAN_TX_DESCRIPTOR_SIZE);
	if (next_desc_addr >= queue_end) {
		next_desc_addr = start_addr;
		desc.elem0.bits.wrap = 1U;
		can_smem_write32(desc_addr + 0U, desc.elem0.word);
	}

	can_smem_write32(next_desc_addr, 0U);

	/* Ensure queue is enabled */
	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET);
	if (!(reg_val & (1U << fifo_id))) {
		reg_val |= (1U << fifo_id);
		CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, reg_val);
	}

	/* Restart queue if stopped */
	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
	if (reg_val & (0x100U << fifo_id)) {
		reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET);
		reg_val |= (1U << fifo_id);
		CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL0_OFFSET, reg_val);
	}

	return CAN_ERROR_NONE;
}

/*
 * can_tx_priority_slot - Send message via TX Priority Queue slot
 * @base_addr: CAN controller base address
 * @slot_id: TX Priority Queue slot index (0-31)
 * @id: CAN message identifier
 * @extended: True for extended ID
 * @data: Pointer to data payload
 * @len: Data length
 * @fd: True for CAN FD
 * @xl: True for CAN XL
 * @brs: Bit Rate Switch
 *
 * Return: enum can_error code
 */
enum can_error can_tx_priority_slot(uint32_t base_addr, uint8_t slot_id,
				    uint32_t id, bool extended,
				    const uint8_t *data, uint32_t len,
				    bool fd, bool xl, bool brs)
{
	uint32_t reg_val, pq_base_addr, desc_addr, data_addr;
	struct can_tx_descriptor desc;
	uint8_t dlc;
	uint16_t buf_size;
	uint32_t i;

	if (slot_id >= CAN_TX_PQ_SLOT_COUNT)
		return CAN_ERROR_INVALID_PARAM;

	if (xl && len > CAN_XL_MAX_DLC)
		return CAN_ERROR_INVALID_PARAM;
	if (fd && !xl && len > CAN_FD_MAX_DLC)
		return CAN_ERROR_INVALID_PARAM;
	if (!fd && !xl && len > CAN_CC_MAX_DLC)
		return CAN_ERROR_INVALID_PARAM;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
	if (reg_val & (1U << slot_id))
		return CAN_ERROR_QUEUE_FULL;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_DESC_MEM_ADD_OFFSET);
	pq_base_addr = (reg_val >> 16U) & 0xFFFFU;

	desc_addr = pq_base_addr + (slot_id * CAN_TX_DESCRIPTOR_SIZE);
	buf_size = can_calc_buffer_size(len, xl);
	data_addr = desc_addr + CAN_TX_DESCRIPTOR_SIZE;

	memset(&desc, 0, sizeof(desc));

	desc.elem0.bits.valid = 1U;
	desc.elem0.bits.hd = 1U;
	desc.elem0.bits.irq = 1U;
	desc.elem0.bits.pq = 1U;
	desc.elem0.bits.fqn = (slot_id >> 1U) & 0x0FU;
	desc.elem0.bits.pqsn0 = slot_id & 0x01U;

	desc.elem1.bits.size = buf_size;
	desc.elem1.bits.plsrc = 1U;

	if (xl) {
		desc.t0 = CAN_BUILD_T0_XL(id & 0x7FFU, 0U, 0U, 0U, 0U);
	} else if (extended) {
		desc.t0 = CAN_BUILD_T0_EXT((id >> 18U) & 0x7FFU,
					   id & 0x3FFFFU, fd ? 1U : 0U);
	} else {
		desc.t0 = CAN_BUILD_T0_STD(id & 0x7FFU, fd ? 1U : 0U);
	}

	if (xl) {
		desc.t1 = CAN_BUILD_T1_XL(len);
	} else if (fd) {
		dlc = can_len_to_fd_dlc(len);
		desc.t1 = CAN_BUILD_T1_FD(dlc, brs ? 1U : 0U, 0U);
	} else {
		dlc = (len > 8U) ? 8U : (uint8_t)len;
		desc.t1 = CAN_BUILD_T1_CC(dlc, 0U);
	}

	if (len <= 8U && !fd && !xl) {
		if (data) {
			for (i = 0U; i < len && i < 4U; i++)
				desc.t2_td0 |= ((uint32_t)data[i] << (i * 8U));
			for (i = 4U; i < len && i < 8U; i++)
				desc.tx_ap_td1 |= ((uint32_t)data[i] <<
						   ((i - 4U) * 8U));
		}
	} else {
		desc.tx_ap_td1 = data_addr;
		if (data && len > 0U)
			can_smem_copy_data(data_addr, data, len);
	}

	can_smem_write32(desc_addr + 0U, desc.elem0.word);
	can_smem_write32(desc_addr + 4U, desc.elem1.word);
	can_smem_write32(desc_addr + 8U, desc.ts0);
	can_smem_write32(desc_addr + 12U, desc.ts1);
	can_smem_write32(desc_addr + 16U, desc.t0);
	can_smem_write32(desc_addr + 20U, desc.t1);
	can_smem_write32(desc_addr + 24U, desc.t2_td0);
	can_smem_write32(desc_addr + 28U, desc.tx_ap_td1);

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET);
	reg_val |= (1U << slot_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET, reg_val);

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL0_OFFSET);
	reg_val |= (1U << slot_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL0_OFFSET, reg_val);

	return CAN_ERROR_NONE;
}

/*
 * can_tx_abort - Abort TX FIFO queue
 * @base_addr: CAN controller base address
 * @fifo_id: TX FIFO queue index (0-7)
 *
 * Return: enum can_error code
 */
enum can_error can_tx_abort(uint32_t base_addr, uint8_t fifo_id)
{
	uint32_t reg_val;
	uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;

	if (fifo_id >= CAN_TX_FIFO_QUEUE_COUNT)
		return CAN_ERROR_INVALID_PARAM;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET);
	reg_val |= (1U << fifo_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, reg_val);

	while (timeout > 0U) {
		reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
		if (!(reg_val & (1U << fifo_id)) &&
		    !(reg_val & (0x100U << fifo_id)))
			break;
		timeout--;
	}

	if (timeout == 0U)
		return CAN_ERROR_TIMEOUT;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET);
	reg_val &= ~(1U << fifo_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL1_OFFSET, reg_val);

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET);
	reg_val &= ~(1U << fifo_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_FQ_CTRL2_OFFSET, reg_val);

	return CAN_ERROR_NONE;
}

/*
 * can_tx_priority_abort - Abort TX Priority Queue slot
 * @base_addr: CAN controller base address
 * @slot_id: TX Priority Queue slot (0-31)
 *
 * Return: enum can_error code
 */
enum can_error can_tx_priority_abort(uint32_t base_addr, uint8_t slot_id)
{
	uint32_t reg_val;
	uint32_t timeout = CAN_POLL_TIMEOUT_COUNT;

	if (slot_id >= CAN_TX_PQ_SLOT_COUNT)
		return CAN_ERROR_INVALID_PARAM;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET);
	reg_val |= (1U << slot_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET, reg_val);

	while (timeout > 0U) {
		reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
		if (!(reg_val & (1U << slot_id)))
			break;
		timeout--;
	}

	if (timeout == 0U)
		return CAN_ERROR_TIMEOUT;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET);
	reg_val &= ~(1U << slot_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL1_OFFSET, reg_val);

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET);
	reg_val &= ~(1U << slot_id);
	CAN_WRITE_REG(base_addr, CAN_MH_TX_PQ_CTRL2_OFFSET, reg_val);

	return CAN_ERROR_NONE;
}

/*
 * can_tx_fifo_is_busy - Check if TX FIFO queue is busy
 * @base_addr: CAN controller base address
 * @fifo_id: TX FIFO queue index (0-7)
 * @is_busy: Pointer to receive busy status
 *
 * Return: enum can_error code
 */
enum can_error can_tx_fifo_is_busy(uint32_t base_addr, uint8_t fifo_id,
				   bool *is_busy)
{
	uint32_t reg_val;

	if (fifo_id >= CAN_TX_FIFO_QUEUE_COUNT || !is_busy)
		return CAN_ERROR_INVALID_PARAM;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_FQ_STS0_OFFSET);
	*is_busy = !!(reg_val & (1U << fifo_id));

	return CAN_ERROR_NONE;
}

/*
 * can_tx_priority_is_busy - Check if TX PQ slot is busy
 * @base_addr: CAN controller base address
 * @slot_id: TX PQ slot (0-31)
 * @is_busy: Pointer to receive busy status
 *
 * Return: enum can_error code
 */
enum can_error can_tx_priority_is_busy(uint32_t base_addr, uint8_t slot_id,
				       bool *is_busy)
{
	uint32_t reg_val;

	if (slot_id >= CAN_TX_PQ_SLOT_COUNT || !is_busy)
		return CAN_ERROR_INVALID_PARAM;

	reg_val = CAN_READ_REG(base_addr, CAN_MH_TX_PQ_STS0_OFFSET);
	*is_busy = !!(reg_val & (1U << slot_id));

	return CAN_ERROR_NONE;
}
