/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * can_driver.h - CAN Controller Driver Header
 *
 * Generic CAN driver for CAN CC/FD/XL protocol controller.
 * Based on Bosch X_CAN IP v3.9 User Manual specifications.
 *
 * Copyright (C) 2026
 */

#ifndef _CAN_DRIVER_H
#define _CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Type definitions for register access (MISRA-C compliant)
 */
typedef volatile uint32_t	reg32_t;
typedef volatile uint16_t	reg16_t;
typedef volatile uint8_t	reg8_t;
typedef volatile const uint32_t	reg32_ro_t;

/*
 * Base address definition - must be configured for target platform
 */
#ifndef CAN_BASE
#define CAN_BASE		((uint32_t)0x40000000U)
#endif

/*
 * Architecture constants
 * Reference: X_CAN User Manual v3.9, Section 1.4
 */
#define CAN_TX_FIFO_QUEUE_COUNT		8U
#define CAN_RX_FIFO_QUEUE_COUNT		8U
#define CAN_FIFO_QUEUE_MAX_MSGS		1024U
#define CAN_TX_PQ_SLOT_COUNT		32U
#define CAN_TX_FILTER_MAX		16U
#define CAN_RX_FILTER_MAX		255U

#define CAN_CC_MAX_DLC			8U
#define CAN_FD_MAX_DLC			64U
#define CAN_XL_MAX_DLC			2048U

#define CAN_TX_DESCRIPTOR_SIZE		32U
#define CAN_RX_DESCRIPTOR_SIZE		16U

/*
 * Message Handler (MH) Register Offsets
 * Reference: X_CAN User Manual v3.9, Section 1.4.4.1
 */
#define CAN_MH_VERSION_OFFSET		0x000U
#define CAN_MH_CTRL_OFFSET		0x004U
#define CAN_MH_CFG_OFFSET		0x008U
#define CAN_MH_STS_OFFSET		0x00CU
#define CAN_MH_SFTY_CFG_OFFSET		0x010U
#define CAN_MH_SFTY_CTRL_OFFSET		0x014U
#define CAN_MH_RX_FILTER_MEM_ADD_OFFSET	0x018U
#define CAN_MH_TX_DESC_MEM_ADD_OFFSET	0x01CU
#define CAN_MH_AXI_ADD_EXT_OFFSET	0x020U
#define CAN_MH_AXI_PARAMS_OFFSET	0x024U
#define CAN_MH_LOCK_OFFSET		0x028U

/* Lock register unlock sequence constants (Section 1.4.4.2.2) */
#define CAN_MH_LOCK_ULK_KEY1		0x00001234U
#define CAN_MH_LOCK_ULK_KEY2		0x00004321U
#define CAN_MH_LOCK_TMK_KEY1		0x67890000U
#define CAN_MH_LOCK_TMK_KEY2		0x98760000U

/* TX FIFO Queue Registers */
#define CAN_MH_TX_DESC_ADD_PT_OFFSET	0x100U
#define CAN_MH_TX_STATISTICS_OFFSET	0x104U
#define CAN_MH_TX_FQ_STS0_OFFSET	0x108U
#define CAN_MH_TX_FQ_STS1_OFFSET	0x10CU
#define CAN_MH_TX_FQ_CTRL0_OFFSET	0x110U
#define CAN_MH_TX_FQ_CTRL1_OFFSET	0x114U
#define CAN_MH_TX_FQ_CTRL2_OFFSET	0x118U

#define CAN_MH_TX_FQ_ADD_PT0_OFFSET	0x120U
#define CAN_MH_TX_FQ_START_ADD0_OFFSET	0x124U
#define CAN_MH_TX_FQ_SIZE0_OFFSET	0x128U

#define CAN_MH_TX_FQ_ADD_PT1_OFFSET	0x130U
#define CAN_MH_TX_FQ_START_ADD1_OFFSET	0x134U
#define CAN_MH_TX_FQ_SIZE1_OFFSET	0x138U

#define CAN_MH_TX_FQ_ADD_PT2_OFFSET	0x140U
#define CAN_MH_TX_FQ_START_ADD2_OFFSET	0x144U
#define CAN_MH_TX_FQ_SIZE2_OFFSET	0x148U

#define CAN_MH_TX_FQ_ADD_PT3_OFFSET	0x150U
#define CAN_MH_TX_FQ_START_ADD3_OFFSET	0x154U
#define CAN_MH_TX_FQ_SIZE3_OFFSET	0x158U

#define CAN_MH_TX_FQ_ADD_PT4_OFFSET	0x160U
#define CAN_MH_TX_FQ_START_ADD4_OFFSET	0x164U
#define CAN_MH_TX_FQ_SIZE4_OFFSET	0x168U

#define CAN_MH_TX_FQ_ADD_PT5_OFFSET	0x170U
#define CAN_MH_TX_FQ_START_ADD5_OFFSET	0x174U
#define CAN_MH_TX_FQ_SIZE5_OFFSET	0x178U

#define CAN_MH_TX_FQ_ADD_PT6_OFFSET	0x180U
#define CAN_MH_TX_FQ_START_ADD6_OFFSET	0x184U
#define CAN_MH_TX_FQ_SIZE6_OFFSET	0x188U

#define CAN_MH_TX_FQ_ADD_PT7_OFFSET	0x190U
#define CAN_MH_TX_FQ_START_ADD7_OFFSET	0x194U
#define CAN_MH_TX_FQ_SIZE7_OFFSET	0x198U

/* TX Priority Queue Registers */
#define CAN_MH_TX_PQ_STS0_OFFSET	0x300U
#define CAN_MH_TX_PQ_STS1_OFFSET	0x304U
#define CAN_MH_TX_PQ_CTRL0_OFFSET	0x30CU
#define CAN_MH_TX_PQ_CTRL1_OFFSET	0x310U
#define CAN_MH_TX_PQ_CTRL2_OFFSET	0x314U
#define CAN_MH_TX_PQ_START_ADD_OFFSET	0x318U

/* RX FIFO Queue Registers */
#define CAN_MH_RX_DESC_ADD_PT_OFFSET	0x400U
#define CAN_MH_RX_STATISTICS_OFFSET	0x404U
#define CAN_MH_RX_FQ_STS0_OFFSET	0x408U
#define CAN_MH_RX_FQ_STS1_OFFSET	0x40CU
#define CAN_MH_RX_FQ_STS2_OFFSET	0x410U
#define CAN_MH_RX_FQ_CTRL0_OFFSET	0x414U
#define CAN_MH_RX_FQ_CTRL1_OFFSET	0x418U
#define CAN_MH_RX_FQ_CTRL2_OFFSET	0x41CU

#define CAN_MH_RX_FQ_ADD_PT0_OFFSET		0x420U
#define CAN_MH_RX_FQ_START_ADD0_OFFSET		0x424U
#define CAN_MH_RX_FQ_SIZE0_OFFSET		0x428U
#define CAN_MH_RX_FQ_DC_START_ADD0_OFFSET	0x42CU
#define CAN_MH_RX_FQ_RD_ADD_PT0_OFFSET		0x430U

#define CAN_MH_RX_FQ_ADD_PT1_OFFSET		0x438U
#define CAN_MH_RX_FQ_START_ADD1_OFFSET		0x43CU
#define CAN_MH_RX_FQ_SIZE1_OFFSET		0x440U
#define CAN_MH_RX_FQ_DC_START_ADD1_OFFSET	0x444U
#define CAN_MH_RX_FQ_RD_ADD_PT1_OFFSET		0x448U

#define CAN_MH_RX_FQ_ADD_PT2_OFFSET		0x450U
#define CAN_MH_RX_FQ_START_ADD2_OFFSET		0x454U
#define CAN_MH_RX_FQ_SIZE2_OFFSET		0x458U
#define CAN_MH_RX_FQ_DC_START_ADD2_OFFSET	0x45CU
#define CAN_MH_RX_FQ_RD_ADD_PT2_OFFSET		0x460U

#define CAN_MH_RX_FQ_ADD_PT3_OFFSET		0x468U
#define CAN_MH_RX_FQ_START_ADD3_OFFSET		0x46CU
#define CAN_MH_RX_FQ_SIZE3_OFFSET		0x470U
#define CAN_MH_RX_FQ_DC_START_ADD3_OFFSET	0x474U
#define CAN_MH_RX_FQ_RD_ADD_PT3_OFFSET		0x478U

#define CAN_MH_RX_FQ_ADD_PT4_OFFSET		0x480U
#define CAN_MH_RX_FQ_START_ADD4_OFFSET		0x484U
#define CAN_MH_RX_FQ_SIZE4_OFFSET		0x488U
#define CAN_MH_RX_FQ_DC_START_ADD4_OFFSET	0x48CU
#define CAN_MH_RX_FQ_RD_ADD_PT4_OFFSET		0x490U

#define CAN_MH_RX_FQ_ADD_PT5_OFFSET		0x498U
#define CAN_MH_RX_FQ_START_ADD5_OFFSET		0x49CU
#define CAN_MH_RX_FQ_SIZE5_OFFSET		0x4A0U
#define CAN_MH_RX_FQ_DC_START_ADD5_OFFSET	0x4A4U
#define CAN_MH_RX_FQ_RD_ADD_PT5_OFFSET		0x4A8U

#define CAN_MH_RX_FQ_ADD_PT6_OFFSET		0x4B0U
#define CAN_MH_RX_FQ_START_ADD6_OFFSET		0x4B4U
#define CAN_MH_RX_FQ_SIZE6_OFFSET		0x4B8U
#define CAN_MH_RX_FQ_DC_START_ADD6_OFFSET	0x4BCU
#define CAN_MH_RX_FQ_RD_ADD_PT6_OFFSET		0x4C0U

#define CAN_MH_RX_FQ_ADD_PT7_OFFSET		0x4C8U
#define CAN_MH_RX_FQ_START_ADD7_OFFSET		0x4CCU
#define CAN_MH_RX_FQ_SIZE7_OFFSET		0x4D0U
#define CAN_MH_RX_FQ_DC_START_ADD7_OFFSET	0x4D4U
#define CAN_MH_RX_FQ_RD_ADD_PT7_OFFSET		0x4D8U

/* TX/RX Filter Registers */
#define CAN_MH_TX_FILTER_CTRL0_OFFSET		0x600U
#define CAN_MH_TX_FILTER_CTRL1_OFFSET		0x604U
#define CAN_MH_TX_FILTER_REFVAL0_OFFSET		0x608U
#define CAN_MH_TX_FILTER_REFVAL1_OFFSET		0x60CU
#define CAN_MH_TX_FILTER_REFVAL2_OFFSET		0x610U
#define CAN_MH_TX_FILTER_REFVAL3_OFFSET		0x614U
#define CAN_MH_RX_FILTER_CTRL_OFFSET		0x680U

/* MH Interrupt Registers */
#define CAN_MH_TX_FQ_INT_STS_OFFSET	0x700U
#define CAN_MH_RX_FQ_INT_STS_OFFSET	0x704U
#define CAN_MH_TX_PQ_INT_STS0_OFFSET	0x708U
#define CAN_MH_TX_PQ_INT_STS1_OFFSET	0x70CU
#define CAN_MH_STATS_INT_STS_OFFSET	0x710U
#define CAN_MH_ERR_INT_STS_OFFSET	0x714U
#define CAN_MH_SFTY_INT_STS_OFFSET	0x718U
#define CAN_MH_AXI_ERR_INFO_OFFSET	0x71CU
#define CAN_MH_DESC_ERR_INFO0_OFFSET	0x720U
#define CAN_MH_DESC_ERR_INFO1_OFFSET	0x724U
#define CAN_MH_TX_FILTER_ERR_INFO_OFFSET	0x728U

/* MH Debug Registers */
#define CAN_MH_DEBUG_TEST_CTRL_OFFSET	0x800U
#define CAN_MH_INT_TEST0_OFFSET		0x804U
#define CAN_MH_INT_TEST1_OFFSET		0x808U
#define CAN_MH_TX_SCAN_FC_OFFSET	0x810U
#define CAN_MH_TX_SCAN_BC_OFFSET	0x814U
#define CAN_MH_TX_FQ_DESC_VALID_OFFSET	0x818U
#define CAN_MH_TX_PQ_DESC_VALID_OFFSET	0x81CU
#define CAN_MH_CRC_CTRL_OFFSET		0x880U
#define CAN_MH_CRC_REG_OFFSET		0x884U

/*
 * MH Register Bit Field Definitions
 */

/* MH_CTRL Register (0x004) */
#define CAN_MH_CTRL_EN_POS		0U
#define CAN_MH_CTRL_EN_MASK		0x00000001U
#define CAN_MH_CTRL_SWRST_POS		1U
#define CAN_MH_CTRL_SWRST_MASK		0x00000002U

/* MH_CFG Register (0x008) */
#define CAN_MH_CFG_INST_NUM_POS		0U
#define CAN_MH_CFG_INST_NUM_MASK	0x00000007U

/* MH_STS Register (0x00C) */
#define CAN_MH_STS_RDY_POS		0U
#define CAN_MH_STS_RDY_MASK		0x00000001U

/* MH_SFTY_CTRL Register (0x014) */
#define CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_POS	0U
#define CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_MASK	0x00000001U
#define CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_POS	1U
#define CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_MASK	0x00000002U

/*
 * Enumerations
 */
enum can_mode {
	CAN_MODE_NORMAL		= 0U,
	CAN_MODE_CONTINUOUS	= 1U,
	CAN_MODE_LOOPBACK	= 2U,
	CAN_MODE_LISTEN		= 3U,
	CAN_MODE_SLEEP		= 4U,
};

enum can_protocol {
	CAN_PROTOCOL_CC		= 0U,
	CAN_PROTOCOL_FD		= 1U,
	CAN_PROTOCOL_XL		= 2U,
};

enum can_error {
	CAN_ERROR_NONE		= 0x00U,
	CAN_ERROR_STUFF		= 0x01U,
	CAN_ERROR_FORM		= 0x02U,
	CAN_ERROR_ACK		= 0x03U,
	CAN_ERROR_BIT1		= 0x04U,
	CAN_ERROR_BIT0		= 0x05U,
	CAN_ERROR_CRC		= 0x06U,
	CAN_ERROR_PROTOCOL	= 0x07U,
	CAN_ERROR_BUS_OFF	= 0x08U,
	CAN_ERROR_PASSIVE	= 0x09U,
	CAN_ERROR_WARNING	= 0x0AU,
	CAN_ERROR_ARB_LOST	= 0x0BU,
	CAN_ERROR_DMA		= 0x0CU,
	CAN_ERROR_TIMEOUT	= 0x0DU,
	CAN_ERROR_INVALID_PARAM	= 0x0EU,
	CAN_ERROR_QUEUE_FULL	= 0x0FU,
	CAN_ERROR_QUEUE_EMPTY	= 0x10U,
	CAN_ERROR_DESC_CRC	= 0x11U,
};

enum can_desc_status {
	CAN_DESC_STS_NONE		= 0x0U,
	CAN_DESC_STS_SUCCESS		= 0x1U,
	CAN_DESC_STS_NOT_SENT		= 0x2U,
	CAN_DESC_STS_NOT_FILTERED	= 0x2U,
	CAN_DESC_STS_SKIPPED_HFI	= 0x3U,
	CAN_DESC_STS_TX_FILTER_REJECTED	= 0x4U,
};

enum can_frame_type {
	CAN_FRAME_DATA		= 0U,
	CAN_FRAME_REMOTE	= 1U,
};

enum can_id_type {
	CAN_ID_STANDARD		= 0U,
	CAN_ID_EXTENDED		= 1U,
};

enum can_bus_state {
	CAN_BUS_STATE_ACTIVE	= 0U,
	CAN_BUS_STATE_WARNING	= 1U,
	CAN_BUS_STATE_PASSIVE	= 2U,
	CAN_BUS_STATE_BUS_OFF	= 3U,
};

/*
 * TX Descriptor Structures (8 x 32-bit words = 32 bytes)
 * Reference: X_CAN User Manual v3.9, Section 1.4.5.5
 */
union can_tx_desc_elem0 {
	uint32_t word;
	struct {
		uint32_t sts	: 4;	/* [3:0]   Status */
		uint32_t rc	: 5;	/* [8:4]   Rolling Counter */
		uint32_t rsvd0	: 2;	/* [10:9]  Reserved */
		uint32_t pqsn0	: 1;	/* [11]    PQSN[0] for PQ */
		uint32_t fqn	: 4;	/* [15:12] FQN[3:0] */
		uint32_t crc	: 9;	/* [24:16] CRC[8:0] */
		uint32_t end	: 1;	/* [25]    END */
		uint32_t pq	: 1;	/* [26]    Priority Queue flag */
		uint32_t irq	: 1;	/* [27]    IRQ request */
		uint32_t next	: 1;	/* [28]    Must be 0 */
		uint32_t wrap	: 1;	/* [29]    Wrap to start */
		uint32_t hd	: 1;	/* [30]    Header Descriptor */
		uint32_t valid	: 1;	/* [31]    Valid flag */
	} bits;
};

union can_tx_desc_elem1 {
	uint32_t word;
	struct {
		uint32_t rsvd0		: 2;	/* [1:0]   Reserved */
		uint32_t nhdo_tdo	: 10;	/* [11:2]  NHDO/TDO */
		uint32_t rsvd1		: 1;	/* [12]    Reserved */
		uint32_t in		: 3;	/* [15:13] Instance Number */
		uint32_t size		: 10;	/* [25:16] Buffer size */
		uint32_t plsrc		: 1;	/* [26]    Payload source */
		uint32_t rsvd2		: 5;	/* [31:27] Reserved */
	} bits;
};

union can_tx_msg_hdr_t0 {
	uint32_t word;
	struct {
		uint32_t ext_id		: 18;	/* [17:0]  Extended ID */
		uint32_t base_id	: 11;	/* [28:18] Base ID */
		uint32_t xtd		: 1;	/* [29]    Extended ID flag */
		uint32_t xlf		: 1;	/* [30]    XL Format */
		uint32_t fdf		: 1;	/* [31]    FD Format */
	} bits;
};

union can_tx_msg_hdr_t0_xl {
	uint32_t word;
	struct {
		uint32_t sdt		: 8;	/* [7:0]   SDU Type */
		uint32_t vcid		: 8;	/* [15:8]  Virtual CAN ID */
		uint32_t sec		: 1;	/* [16]    SEC */
		uint32_t rrs		: 1;	/* [17]    RRS */
		uint32_t prio_id	: 11;	/* [28:18] Priority ID */
		uint32_t xtd		: 1;	/* [29]    Must be 0 */
		uint32_t xlf		: 1;	/* [30]    XL Format */
		uint32_t fdf		: 1;	/* [31]    FD Format */
	} bits;
};

union can_tx_msg_hdr_t1_cc {
	uint32_t word;
	struct {
		uint32_t rsvd0	: 16;	/* [15:0]  Reserved */
		uint32_t dlc	: 4;	/* [19:16] DLC */
		uint32_t rsvd1	: 6;	/* [25:20] Reserved */
		uint32_t rtr	: 1;	/* [26]    RTR */
		uint32_t rsvd2	: 3;	/* [29:27] Reserved */
		uint32_t fir	: 1;	/* [30]    FIR */
		uint32_t rsvd3	: 1;	/* [31]    Reserved */
	} bits;
};

union can_tx_msg_hdr_t1_fd {
	uint32_t word;
	struct {
		uint32_t rsvd0	: 16;	/* [15:0]  Reserved */
		uint32_t dlc	: 4;	/* [19:16] DLC */
		uint32_t esi	: 1;	/* [20]    ESI */
		uint32_t rsvd1	: 4;	/* [24:21] Reserved */
		uint32_t brs	: 1;	/* [25]    BRS */
		uint32_t rsvd2	: 1;	/* [26]    Reserved */
		uint32_t rsvd3	: 3;	/* [29:27] Reserved */
		uint32_t fir	: 1;	/* [30]    FIR */
		uint32_t rsvd4	: 1;	/* [31]    Reserved */
	} bits;
};

union can_tx_msg_hdr_t1_xl {
	uint32_t word;
	struct {
		uint32_t rsvd0		: 16;	/* [15:0]  Reserved */
		uint32_t dlc_xl		: 11;	/* [26:16] DLC-XL */
		uint32_t rsvd1		: 3;	/* [29:27] Reserved */
		uint32_t fir		: 1;	/* [30]    FIR */
		uint32_t rsvd2		: 1;	/* [31]    Reserved */
	} bits;
};

struct can_tx_descriptor {
	union can_tx_desc_elem0	elem0;
	union can_tx_desc_elem1	elem1;
	uint32_t		ts0;
	uint32_t		ts1;
	uint32_t		t0;
	uint32_t		t1;
	uint32_t		t2_td0;
	uint32_t		tx_ap_td1;
} __attribute__((packed, aligned(4)));

/*
 * RX Descriptor Structures (4 x 32-bit words = 16 bytes)
 * Reference: X_CAN User Manual v3.9, Section 1.4.5.7
 */
union can_rx_desc_elem0 {
	uint32_t word;
	struct {
		uint32_t sts	: 4;	/* [3:0]   Status */
		uint32_t rc	: 5;	/* [8:4]   Rolling Counter */
		uint32_t in	: 3;	/* [11:9]  Instance Number */
		uint32_t fqn	: 4;	/* [15:12] FIFO Queue Number */
		uint32_t crc	: 7;	/* [22:16] CRC (7-bit) */
		uint32_t rsvd0	: 4;	/* [26:23] Reserved */
		uint32_t irq	: 1;	/* [27]    IRQ */
		uint32_t next	: 1;	/* [28]    NEXT */
		uint32_t rsvd1	: 1;	/* [29]    Reserved */
		uint32_t hd	: 1;	/* [30]    Header Descriptor */
		uint32_t valid	: 1;	/* [31]    Valid */
	} bits;
};

struct can_rx_descriptor {
	union can_rx_desc_elem0	elem0;
	uint32_t		rx_ap;
	uint32_t		ts0;
	uint32_t		ts1;
} __attribute__((packed, aligned(4)));

/*
 * RX Message Header Structure
 */
union can_rx_msg_hdr_r0 {
	uint32_t word;
	struct {
		uint32_t ext_id		: 18;	/* [17:0]  ExtID / SDT+VCID */
		uint32_t base_id	: 11;	/* [28:18] BaseID / PrioID */
		uint32_t xtd		: 1;	/* [29]    XTD */
		uint32_t xlf		: 1;	/* [30]    XLF */
		uint32_t fdf		: 1;	/* [31]    FDF */
	} bits;
};

union can_rx_msg_hdr_r1 {
	uint32_t word;
	struct {
		uint32_t rsvd0		: 16;	/* [15:0]  Reserved / AF */
		uint32_t dlc		: 4;	/* [19:16] DLC */
		uint32_t esi		: 1;	/* [20]    ESI */
		uint32_t rsvd1		: 4;	/* [24:21] Reserved */
		uint32_t brs		: 1;	/* [25]    BRS */
		uint32_t rtr		: 1;	/* [26]    RTR */
		uint32_t dlc_xl_h	: 7;	/* [26:27]+[31:28] DLC-XL high */
	} bits;
};

struct can_rx_msg_header {
	union can_rx_msg_hdr_r0	r0;
	union can_rx_msg_hdr_r1	r1;
	uint32_t		r2;
} __attribute__((packed, aligned(4)));

/*
 * Protocol Controller (PRT) Register Offsets
 * Reference: X_CAN User Manual v3.9, Section 1.5.4.1
 */
#define CAN_PRT_BASE_OFFSET		0xA00U
#define CAN_PRT_ENDN_OFFSET		0xA00U
#define CAN_PRT_PREL_OFFSET		0xA04U
#define CAN_PRT_STAT_OFFSET		0xA08U
#define CAN_PRT_EVNT_OFFSET		0xA20U
#define CAN_PRT_LOCK_OFFSET		0xA40U
#define CAN_PRT_CTRL_OFFSET		0xA44U
#define CAN_PRT_FIMC_OFFSET		0xA48U
#define CAN_PRT_TEST_OFFSET		0xA4CU
#define CAN_PRT_MODE_OFFSET		0xA60U
#define CAN_PRT_NBTP_OFFSET		0xA64U
#define CAN_PRT_DBTP_OFFSET		0xA68U
#define CAN_PRT_XBTP_OFFSET		0xA6CU
#define CAN_PRT_PCFG_OFFSET		0xA70U

/*
 * Configuration Structures
 */
struct can_bit_timing {
	uint8_t		brp;
	uint16_t	tseg1;
	uint8_t		tseg2;
	uint8_t		sjw;
	uint8_t		tdco;
	uint8_t		padding[1];
};

struct can_queue_config {
	uint32_t	start_addr;
	uint32_t	dc_start_addr;
	uint16_t	size;
	uint16_t	dc_size;
	bool		enabled;
	bool		continuous;
	uint8_t		padding[2];
};

struct can_config {
	uint32_t		base_addr;
	uint32_t		lmem_base_addr;
	uint32_t		lmem_size;

	enum can_protocol	protocol;
	enum can_mode		mode;

	struct can_bit_timing	nominal_timing;
	struct can_bit_timing	data_timing;
	struct can_bit_timing	xl_timing;

	struct can_queue_config	tx_fifo_queues[CAN_TX_FIFO_QUEUE_COUNT];
	struct can_queue_config	rx_fifo_queues[CAN_RX_FIFO_QUEUE_COUNT];

	uint32_t		txpq_start_addr;
	uint8_t			txpq_slot_count;
	uint8_t			padding1[3];

	uint32_t		rx_filter_base_addr;
	uint32_t		tx_desc_base_addr;
	uint8_t			rx_filter_count;
	uint8_t			instance_num;
	uint8_t			padding2[2];

	uint32_t		func_int_enable;
	uint32_t		err_int_enable;
	uint32_t		safety_int_enable;

	bool			loopback_enable;
	bool			listen_only;
	bool			tx_desc_crc_enable;
	bool			rx_desc_crc_enable;
};

/*
 * Function prototypes - Core API
 */
enum can_error can_init(const struct can_config *config);

/*
 * Register access macros
 */
#define CAN_REG_READ(addr) \
	(*((volatile uint32_t *)(uintptr_t)(addr)))

#define CAN_REG_WRITE(addr, val) \
	(*((volatile uint32_t *)(uintptr_t)(addr)) = (val))

#define CAN_READ_REG(base, offset) \
	CAN_REG_READ((base) + (offset))

#define CAN_WRITE_REG(base, offset, val) \
	CAN_REG_WRITE((base) + (offset), (val))

#define CAN_SET_BITS(base, offset, mask) \
	CAN_WRITE_REG(base, offset, CAN_READ_REG(base, offset) | (mask))

#define CAN_CLR_BITS(base, offset, mask) \
	CAN_WRITE_REG(base, offset, CAN_READ_REG(base, offset) & ~(mask))

/*
 * Static assertions for structure sizes
 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(struct can_tx_descriptor) == 32U,
	       "TX descriptor must be 32 bytes");
_Static_assert(sizeof(struct can_rx_descriptor) == 16U,
	       "RX descriptor must be 16 bytes");
#endif

#endif /* _CAN_DRIVER_H */
