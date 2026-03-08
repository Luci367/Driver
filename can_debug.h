/******************************************************************************
 *  File: can_debug.h
 *  Module: CAN Debug Logging
 *
 *  Compile-time filtered debug logging for all CAN driver layers (HAL/HCL/HDL).
 *  Provides 4 severity levels: ERROR, WARN, INFO, VERBOSE.
 *
 *  Usage:
 *    - Set CAN_DEBUG_LEVEL in your build system (0=NONE .. 4=VERBOSE).
 *    - Implement Can_DebugPrint() for your platform (UART, semihosting, etc.).
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef CAN_DEBUG_H
#define CAN_DEBUG_H

/******************************************************************************
 *  Log levels
 *****************************************************************************/

#define CAN_DBG_LVL_NONE    0u
#define CAN_DBG_LVL_ERROR   1u
#define CAN_DBG_LVL_WARN    2u
#define CAN_DBG_LVL_INFO    3u
#define CAN_DBG_LVL_VERBOSE 4u

#ifndef CAN_DEBUG_LEVEL
#define CAN_DEBUG_LEVEL  CAN_DBG_LVL_INFO
#endif

/******************************************************************************
 *  User-provided output function (implement for your platform)
 *****************************************************************************/

extern void Can_DebugPrint(const char *layer, const char *func,
                           const char *fmt, ...);

/******************************************************************************
 *  Per-level macros — compiled out when level is below threshold
 *****************************************************************************/

#if (CAN_DEBUG_LEVEL >= CAN_DBG_LVL_ERROR)
#define CAN_DBG_ERR(layer, fmt, ...) \
    Can_DebugPrint(layer, __func__, "[ERR] " fmt, ##__VA_ARGS__)
#else
#define CAN_DBG_ERR(layer, fmt, ...)
#endif

#if (CAN_DEBUG_LEVEL >= CAN_DBG_LVL_WARN)
#define CAN_DBG_WARN(layer, fmt, ...) \
    Can_DebugPrint(layer, __func__, "[WARN] " fmt, ##__VA_ARGS__)
#else
#define CAN_DBG_WARN(layer, fmt, ...)
#endif

#if (CAN_DEBUG_LEVEL >= CAN_DBG_LVL_INFO)
#define CAN_DBG_INFO(layer, fmt, ...) \
    Can_DebugPrint(layer, __func__, "[INFO] " fmt, ##__VA_ARGS__)
#else
#define CAN_DBG_INFO(layer, fmt, ...)
#endif

#if (CAN_DEBUG_LEVEL >= CAN_DBG_LVL_VERBOSE)
#define CAN_DBG_VERB(layer, fmt, ...) \
    Can_DebugPrint(layer, __func__, "[VERB] " fmt, ##__VA_ARGS__)
#else
#define CAN_DBG_VERB(layer, fmt, ...)
#endif

/******************************************************************************
 *  Layer tag strings
 *****************************************************************************/

#define DBG_HAL  "HAL"
#define DBG_HCL  "HCL"
#define DBG_HDL  "HDL"

#endif /* CAN_DEBUG_H */
