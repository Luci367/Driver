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

#endif /* _CAN_DRIVER_H */
