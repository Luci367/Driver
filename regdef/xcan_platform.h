/*
 * XCAN Platform Abstraction Header
 *
 * Provides SoC-agnostic register access macros and standard type includes.
 * The integrator must define the register read/write implementations
 * appropriate for their target platform before including any regdef header.
 *
 * Adapted from xcan_sw_example (Bosch) for AUTOSAR driver integration.
 * Original platform-specific code targeted Altera/Xilinx FPGA eval boards.
 */

#ifndef XCAN_PLATFORM_H
#define XCAN_PLATFORM_H

#include <stdint.h>

/*
 * Register access macros.
 * These MUST be provided by the SoC integration layer.
 * Default implementations use volatile pointer dereference which is
 * correct for memory-mapped I/O on most ARM Cortex-R/A platforms.
 *
 * To override: #define XCAN_REG_READ32 / XCAN_REG_WRITE32 before
 * including this header, or provide a platform-specific xcan_platform_cfg.h.
 */
#ifndef XCAN_REG_READ32
#define XCAN_REG_READ32(addr)           (*((volatile uint32_t *)(addr)))
#endif

#ifndef XCAN_REG_WRITE32
#define XCAN_REG_WRITE32(addr, val)     (*((volatile uint32_t *)(addr)) = (val))
#endif

/*
 * Endianness: The XCAN descriptor structs use bit-field layouts that assume
 * little-endian byte order. Most automotive ARM SoCs (Cortex-R5, Cortex-A53)
 * run in little-endian mode. Define __LITTLE_ENDIAN to 1 if not already set.
 */
#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1
#endif

#endif /* XCAN_PLATFORM_H */
