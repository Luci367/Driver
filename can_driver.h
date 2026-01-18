/**
 * @file    can_driver.h
 * @brief   CAN Controller Driver Header - Production Ready
 * @details Generic CAN driver header for CAN CC/FD/XL protocol controller.
 *          Based on Bosch X_CAN IP v3.9 User Manual specifications.
 *          Conforms to ISO11898-1:2015 and CiA610-1.
 *
 * @version 1.0.0
 * @date    2026-01-18
 *
 * @note    MISRA-C:2012 Compliant
 * @note    AXI4-Lite 32-bit aligned structures
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/

#include <stdbool.h>
#include <stdint.h>

/*============================================================================*/
/* MISRA-C COMPLIANT TYPE DEFINITIONS                                         */
/*============================================================================*/

/** @brief Volatile 32-bit register type for memory-mapped I/O */
typedef volatile uint32_t reg32_t;

/** @brief Volatile 16-bit register type for memory-mapped I/O */
typedef volatile uint16_t reg16_t;

/** @brief Volatile 8-bit register type for memory-mapped I/O */
typedef volatile uint8_t reg8_t;

/** @brief Read-only 32-bit register type */
typedef volatile const uint32_t reg32_ro_t;

/** @brief Write-only 32-bit register type */
typedef volatile uint32_t reg32_wo_t;

/*============================================================================*/
/* BASE ADDRESS DEFINITION                                                    */
/*============================================================================*/

/**
 * @brief   CAN Controller Base Address
 * @note    Must be configured for target platform during system integration.
 *          This is a placeholder - replace with actual memory-mapped address.
 */
#ifndef CAN_BASE
#define CAN_BASE ((uint32_t)0x40000000U)
#endif

/*============================================================================*/
/* ARCHITECTURE CONSTANTS                                                     */
/*============================================================================*/

/** @brief Maximum number of TX FIFO queues */
#define CAN_TX_FIFO_QUEUE_COUNT (8U)

/** @brief Maximum number of RX FIFO queues */
#define CAN_RX_FIFO_QUEUE_COUNT (8U)

/** @brief Maximum messages per FIFO queue */
#define CAN_FIFO_QUEUE_MAX_MSGS (1024U)

/** @brief Maximum TX Priority Queue slots */
#define CAN_TX_PQ_SLOT_COUNT (32U)

/** @brief Maximum TX filter elements */
#define CAN_TX_FILTER_MAX (16U)

/** @brief Maximum RX filter elements */
#define CAN_RX_FILTER_MAX (255U)

/** @brief CAN CC maximum data length */
#define CAN_CC_MAX_DLC (8U)

/** @brief CAN FD maximum data length */
#define CAN_FD_MAX_DLC (64U)

/** @brief CAN XL maximum data length */
#define CAN_XL_MAX_DLC (2048U)

/** @brief TX Descriptor size in bytes (8 x 32-bit words) */
#define CAN_TX_DESCRIPTOR_SIZE (32U)

/** @brief RX Descriptor size in bytes (4 x 32-bit words) */
#define CAN_RX_DESCRIPTOR_SIZE (16U)

/*============================================================================*/
/* REGISTER OFFSET DEFINITIONS - MESSAGE HANDLER (MH)                         */
/* Reference: X_CAN User Manual v3.9, Section 1.4.4.1                         */
/*============================================================================*/

/** @defgroup MH_REG_OFFSETS Message Handler Register Offsets
 *  @brief MH register offsets from CAN_BASE
 *  @{
 */

/** @brief Release Identification Register */
#define CAN_MH_VERSION_OFFSET (0x000U)

/** @brief MH Control Register */
#define CAN_MH_CTRL_OFFSET (0x004U)

/** @brief MH Configuration Register */
#define CAN_MH_CFG_OFFSET (0x008U)

/** @brief MH Status Register */
#define CAN_MH_STS_OFFSET (0x00CU)

/** @brief MH Safety Configuration Register */
#define CAN_MH_SFTY_CFG_OFFSET (0x010U)

/** @brief MH Safety Control Register */
#define CAN_MH_SFTY_CTRL_OFFSET (0x014U)

/** @brief RX Filter Base Address Register */
#define CAN_MH_RX_FILTER_MEM_ADD_OFFSET (0x018U)

/** @brief TX Descriptor Base Address Register */
#define CAN_MH_TX_DESC_MEM_ADD_OFFSET (0x01CU)

/** @brief AXI Address Extension Register */
#define CAN_MH_AXI_ADD_EXT_OFFSET (0x020U)

/** @brief AXI Parameter Register */
#define CAN_MH_AXI_PARAMS_OFFSET (0x024U)

/** @brief MH Lock Register */
#define CAN_MH_LOCK_OFFSET (0x028U)

/*---------------------------------------------------------------------------*/
/* TX FIFO Queue Registers (Section 1.4.4.1)                                 */
/*---------------------------------------------------------------------------*/

/** @brief TX Descriptor Current Address Pointer Register */
#define CAN_MH_TX_DESC_ADD_PT_OFFSET (0x100U)

/** @brief TX Statistics Register */
#define CAN_MH_TX_STATISTICS_OFFSET (0x104U)

/** @brief TX FIFO Queue Status Register 0 */
#define CAN_MH_TX_FQ_STS0_OFFSET (0x108U)

/** @brief TX FIFO Queue Status Register 1 */
#define CAN_MH_TX_FQ_STS1_OFFSET (0x10CU)

/** @brief TX FIFO Queue Control Register 0 */
#define CAN_MH_TX_FQ_CTRL0_OFFSET (0x110U)

/** @brief TX FIFO Queue Control Register 1 */
#define CAN_MH_TX_FQ_CTRL1_OFFSET (0x114U)

/** @brief TX FIFO Queue Control Register 2 */
#define CAN_MH_TX_FQ_CTRL2_OFFSET (0x118U)

/* TX FIFO Queue 0-7 Registers */
#define CAN_MH_TX_FQ_ADD_PT0_OFFSET (0x120U)
#define CAN_MH_TX_FQ_START_ADD0_OFFSET (0x124U)
#define CAN_MH_TX_FQ_SIZE0_OFFSET (0x128U)

#define CAN_MH_TX_FQ_ADD_PT1_OFFSET (0x130U)
#define CAN_MH_TX_FQ_START_ADD1_OFFSET (0x134U)
#define CAN_MH_TX_FQ_SIZE1_OFFSET (0x138U)

#define CAN_MH_TX_FQ_ADD_PT2_OFFSET (0x140U)
#define CAN_MH_TX_FQ_START_ADD2_OFFSET (0x144U)
#define CAN_MH_TX_FQ_SIZE2_OFFSET (0x148U)

#define CAN_MH_TX_FQ_ADD_PT3_OFFSET (0x150U)
#define CAN_MH_TX_FQ_START_ADD3_OFFSET (0x154U)
#define CAN_MH_TX_FQ_SIZE3_OFFSET (0x158U)

#define CAN_MH_TX_FQ_ADD_PT4_OFFSET (0x160U)
#define CAN_MH_TX_FQ_START_ADD4_OFFSET (0x164U)
#define CAN_MH_TX_FQ_SIZE4_OFFSET (0x168U)

#define CAN_MH_TX_FQ_ADD_PT5_OFFSET (0x170U)
#define CAN_MH_TX_FQ_START_ADD5_OFFSET (0x174U)
#define CAN_MH_TX_FQ_SIZE5_OFFSET (0x178U)

#define CAN_MH_TX_FQ_ADD_PT6_OFFSET (0x180U)
#define CAN_MH_TX_FQ_START_ADD6_OFFSET (0x184U)
#define CAN_MH_TX_FQ_SIZE6_OFFSET (0x188U)

#define CAN_MH_TX_FQ_ADD_PT7_OFFSET (0x190U)
#define CAN_MH_TX_FQ_START_ADD7_OFFSET (0x194U)
#define CAN_MH_TX_FQ_SIZE7_OFFSET (0x198U)

/*---------------------------------------------------------------------------*/
/* TX Priority Queue Registers (Section 1.4.4.1)                             */
/*---------------------------------------------------------------------------*/

/** @brief TX Priority Queue Status Register 0 */
#define CAN_MH_TX_PQ_STS0_OFFSET (0x300U)

/** @brief TX Priority Queue Status Register 1 */
#define CAN_MH_TX_PQ_STS1_OFFSET (0x304U)

/** @brief TX Priority Queue Control Register 0 */
#define CAN_MH_TX_PQ_CTRL0_OFFSET (0x30CU)

/** @brief TX Priority Queue Control Register 1 */
#define CAN_MH_TX_PQ_CTRL1_OFFSET (0x310U)

/** @brief TX Priority Queue Control Register 2 */
#define CAN_MH_TX_PQ_CTRL2_OFFSET (0x314U)

/** @brief TX Priority Queue Start Address Register */
#define CAN_MH_TX_PQ_START_ADD_OFFSET (0x318U)

/*---------------------------------------------------------------------------*/
/* RX FIFO Queue Registers (Section 1.4.4.1)                                 */
/*---------------------------------------------------------------------------*/

/** @brief RX Descriptor Current Address Pointer Register */
#define CAN_MH_RX_DESC_ADD_PT_OFFSET (0x400U)

/** @brief RX Statistics Register */
#define CAN_MH_RX_STATISTICS_OFFSET (0x404U)

/** @brief RX FIFO Queue Status Register 0 */
#define CAN_MH_RX_FQ_STS0_OFFSET (0x408U)

/** @brief RX FIFO Queue Status Register 1 */
#define CAN_MH_RX_FQ_STS1_OFFSET (0x40CU)

/** @brief RX FIFO Queue Status Register 2 */
#define CAN_MH_RX_FQ_STS2_OFFSET (0x410U)

/** @brief RX FIFO Queue Control Register 0 */
#define CAN_MH_RX_FQ_CTRL0_OFFSET (0x414U)

/** @brief RX FIFO Queue Control Register 1 */
#define CAN_MH_RX_FQ_CTRL1_OFFSET (0x418U)

/** @brief RX FIFO Queue Control Register 2 */
#define CAN_MH_RX_FQ_CTRL2_OFFSET (0x41CU)

/* RX FIFO Queue 0-7 Registers */
#define CAN_MH_RX_FQ_ADD_PT0_OFFSET (0x420U)
#define CAN_MH_RX_FQ_START_ADD0_OFFSET (0x424U)
#define CAN_MH_RX_FQ_SIZE0_OFFSET (0x428U)
#define CAN_MH_RX_FQ_DC_START_ADD0_OFFSET (0x42CU)
#define CAN_MH_RX_FQ_RD_ADD_PT0_OFFSET (0x430U)

#define CAN_MH_RX_FQ_ADD_PT1_OFFSET (0x438U)
#define CAN_MH_RX_FQ_START_ADD1_OFFSET (0x43CU)
#define CAN_MH_RX_FQ_SIZE1_OFFSET (0x440U)
#define CAN_MH_RX_FQ_DC_START_ADD1_OFFSET (0x444U)
#define CAN_MH_RX_FQ_RD_ADD_PT1_OFFSET (0x448U)

#define CAN_MH_RX_FQ_ADD_PT2_OFFSET (0x450U)
#define CAN_MH_RX_FQ_START_ADD2_OFFSET (0x454U)
#define CAN_MH_RX_FQ_SIZE2_OFFSET (0x458U)
#define CAN_MH_RX_FQ_DC_START_ADD2_OFFSET (0x45CU)
#define CAN_MH_RX_FQ_RD_ADD_PT2_OFFSET (0x460U)

#define CAN_MH_RX_FQ_ADD_PT3_OFFSET (0x468U)
#define CAN_MH_RX_FQ_START_ADD3_OFFSET (0x46CU)
#define CAN_MH_RX_FQ_SIZE3_OFFSET (0x470U)
#define CAN_MH_RX_FQ_DC_START_ADD3_OFFSET (0x474U)
#define CAN_MH_RX_FQ_RD_ADD_PT3_OFFSET (0x478U)

#define CAN_MH_RX_FQ_ADD_PT4_OFFSET (0x480U)
#define CAN_MH_RX_FQ_START_ADD4_OFFSET (0x484U)
#define CAN_MH_RX_FQ_SIZE4_OFFSET (0x488U)
#define CAN_MH_RX_FQ_DC_START_ADD4_OFFSET (0x48CU)
#define CAN_MH_RX_FQ_RD_ADD_PT4_OFFSET (0x490U)

#define CAN_MH_RX_FQ_ADD_PT5_OFFSET (0x498U)
#define CAN_MH_RX_FQ_START_ADD5_OFFSET (0x49CU)
#define CAN_MH_RX_FQ_SIZE5_OFFSET (0x4A0U)
#define CAN_MH_RX_FQ_DC_START_ADD5_OFFSET (0x4A4U)
#define CAN_MH_RX_FQ_RD_ADD_PT5_OFFSET (0x4A8U)

#define CAN_MH_RX_FQ_ADD_PT6_OFFSET (0x4B0U)
#define CAN_MH_RX_FQ_START_ADD6_OFFSET (0x4B4U)
#define CAN_MH_RX_FQ_SIZE6_OFFSET (0x4B8U)
#define CAN_MH_RX_FQ_DC_START_ADD6_OFFSET (0x4BCU)
#define CAN_MH_RX_FQ_RD_ADD_PT6_OFFSET (0x4C0U)

#define CAN_MH_RX_FQ_ADD_PT7_OFFSET (0x4C8U)
#define CAN_MH_RX_FQ_START_ADD7_OFFSET (0x4CCU)
#define CAN_MH_RX_FQ_SIZE7_OFFSET (0x4D0U)
#define CAN_MH_RX_FQ_DC_START_ADD7_OFFSET (0x4D4U)
#define CAN_MH_RX_FQ_RD_ADD_PT7_OFFSET (0x4D8U)

/*---------------------------------------------------------------------------*/
/* TX/RX Filter Registers (Section 1.4.4.1)                                  */
/*---------------------------------------------------------------------------*/

/** @brief TX Filter Control Register 0 */
#define CAN_MH_TX_FILTER_CTRL0_OFFSET (0x600U)

/** @brief TX Filter Control Register 1 */
#define CAN_MH_TX_FILTER_CTRL1_OFFSET (0x604U)

/** @brief TX Filter Reference Value Register 0 */
#define CAN_MH_TX_FILTER_REFVAL0_OFFSET (0x608U)

/** @brief TX Filter Reference Value Register 1 */
#define CAN_MH_TX_FILTER_REFVAL1_OFFSET (0x60CU)

/** @brief TX Filter Reference Value Register 2 */
#define CAN_MH_TX_FILTER_REFVAL2_OFFSET (0x610U)

/** @brief TX Filter Reference Value Register 3 */
#define CAN_MH_TX_FILTER_REFVAL3_OFFSET (0x614U)

/** @brief RX Filter Control Register */
#define CAN_MH_RX_FILTER_CTRL_OFFSET (0x680U)

/*---------------------------------------------------------------------------*/
/* MH Interrupt Registers (Section 1.4.4.1)                                  */
/*---------------------------------------------------------------------------*/

/** @brief TX FIFO Queue Interrupt Status Register */
#define CAN_MH_TX_FQ_INT_STS_OFFSET (0x700U)

/** @brief RX FIFO Queue Interrupt Status Register */
#define CAN_MH_RX_FQ_INT_STS_OFFSET (0x704U)

/** @brief TX Priority Queue Interrupt Status Register 0 */
#define CAN_MH_TX_PQ_INT_STS0_OFFSET (0x708U)

/** @brief TX Priority Queue Interrupt Status Register 1 */
#define CAN_MH_TX_PQ_INT_STS1_OFFSET (0x70CU)

/** @brief Statistics Interrupt Status Register */
#define CAN_MH_STATS_INT_STS_OFFSET (0x710U)

/** @brief Error Interrupt Status Register */
#define CAN_MH_ERR_INT_STS_OFFSET (0x714U)

/** @brief Safety Interrupt Status Register */
#define CAN_MH_SFTY_INT_STS_OFFSET (0x718U)

/** @brief AXI Error Information Register */
#define CAN_MH_AXI_ERR_INFO_OFFSET (0x71CU)

/** @brief Descriptor Error Information Register 0 */
#define CAN_MH_DESC_ERR_INFO0_OFFSET (0x720U)

/** @brief Descriptor Error Information Register 1 */
#define CAN_MH_DESC_ERR_INFO1_OFFSET (0x724U)

/** @brief TX Filter Error Information Register */
#define CAN_MH_TX_FILTER_ERR_INFO_OFFSET (0x728U)

/*---------------------------------------------------------------------------*/
/* MH Debug Registers (Section 1.4.4.1)                                      */
/*---------------------------------------------------------------------------*/

/** @brief Debug Test Control Register */
#define CAN_MH_DEBUG_TEST_CTRL_OFFSET (0x800U)

/** @brief Interrupt Test Register 0 */
#define CAN_MH_INT_TEST0_OFFSET (0x804U)

/** @brief Interrupt Test Register 1 */
#define CAN_MH_INT_TEST1_OFFSET (0x808U)

/** @brief TX-SCAN First Candidates Register */
#define CAN_MH_TX_SCAN_FC_OFFSET (0x810U)

/** @brief TX-SCAN Best Candidates Register */
#define CAN_MH_TX_SCAN_BC_OFFSET (0x814U)

/** @brief Valid TX FIFO Queue Descriptors Register */
#define CAN_MH_TX_FQ_DESC_VALID_OFFSET (0x818U)

/** @brief Valid TX Priority Queue Descriptors Register */
#define CAN_MH_TX_PQ_DESC_VALID_OFFSET (0x81CU)

/** @brief CRC Control Register */
#define CAN_MH_CRC_CTRL_OFFSET (0x880U)

/** @brief CRC Register */
#define CAN_MH_CRC_REG_OFFSET (0x884U)

/** @} */ /* End of MH_REG_OFFSETS */

/*============================================================================*/
/* REGISTER OFFSET DEFINITIONS - INTERRUPT CONTROLLER (IRC)                   */
/* Reference: X_CAN User Manual v3.9, Section 1.7.2.1                         */
/*============================================================================*/

/** @defgroup IRC_REG_OFFSETS Interrupt Controller Register Offsets
 *  @brief IRC register offsets from IRC base (CAN_BASE + IRC_BASE_OFFSET)
 *  @{
 */

/** @brief IRC Base Offset from CAN_BASE */
#define CAN_IRC_BASE_OFFSET (0x900U)

/** @brief Functional Raw Event Status Register */
#define CAN_IRC_FUNC_RAW_OFFSET (0x900U)

/** @brief Error Raw Event Status Register */
#define CAN_IRC_ERR_RAW_OFFSET (0x904U)

/** @brief Safety Raw Event Status Register */
#define CAN_IRC_SAFETY_RAW_OFFSET (0x908U)

/** @brief Functional Raw Event Clear Register */
#define CAN_IRC_FUNC_CLR_OFFSET (0x910U)

/** @brief Error Raw Event Clear Register */
#define CAN_IRC_ERR_CLR_OFFSET (0x914U)

/** @brief Safety Raw Event Clear Register */
#define CAN_IRC_SAFETY_CLR_OFFSET (0x918U)

/** @brief Functional Raw Event Enable Register */
#define CAN_IRC_FUNC_ENA_OFFSET (0x920U)

/** @brief Error Raw Event Enable Register */
#define CAN_IRC_ERR_ENA_OFFSET (0x924U)

/** @brief Safety Raw Event Enable Register */
#define CAN_IRC_SAFETY_ENA_OFFSET (0x928U)

/** @brief IRC Configuration Register (Capturing Mode) */
#define CAN_IRC_CAPTURING_MODE_OFFSET (0x930U)

/** @brief Hardware Debug Port Control Register */
#define CAN_IRC_HDP_OFFSET (0x940U)

/** @} */ /* End of IRC_REG_OFFSETS */

/*============================================================================*/
/* REGISTER OFFSET DEFINITIONS - PROTOCOL CONTROLLER (PRT)                    */
/* Reference: X_CAN User Manual v3.9, Section 1.5.4.1                         */
/*============================================================================*/

/** @defgroup PRT_REG_OFFSETS Protocol Controller Register Offsets
 *  @brief PRT register offsets from PRT base (CAN_BASE + PRT_BASE_OFFSET)
 *  @{
 */

/** @brief PRT Base Offset from CAN_BASE */
#define CAN_PRT_BASE_OFFSET (0xA00U)

/** @brief Endianness Test Register */
#define CAN_PRT_ENDN_OFFSET (0xA00U)

/** @brief PRT Release Identification Register */
#define CAN_PRT_PREL_OFFSET (0xA04U)

/** @brief PRT Status Register */
#define CAN_PRT_STAT_OFFSET (0xA08U)

/** @brief Event Status Flags Register */
#define CAN_PRT_EVNT_OFFSET (0xA20U)

/** @brief Unlock Sequence Register */
#define CAN_PRT_LOCK_OFFSET (0xA40U)

/** @brief PRT Control Register */
#define CAN_PRT_CTRL_OFFSET (0xA44U)

/** @brief Fault Injection Module Control Register */
#define CAN_PRT_FIMC_OFFSET (0xA48U)

/** @brief Hardware Test Functions Register */
#define CAN_PRT_TEST_OFFSET (0xA4CU)

/** @brief Operating Mode Register */
#define CAN_PRT_MODE_OFFSET (0xA60U)

/** @brief Arbitration Phase Nominal Bit Timing Register */
#define CAN_PRT_NBTP_OFFSET (0xA64U)

/** @brief CAN FD Data Phase Bit Timing Register */
#define CAN_PRT_DBTP_OFFSET (0xA68U)

/** @brief CAN XL Data Phase Bit Timing Register */
#define CAN_PRT_XBTP_OFFSET (0xA6CU)

/** @brief PWME Configuration Register */
#define CAN_PRT_PCFG_OFFSET (0xA70U)

/** @} */ /* End of PRT_REG_OFFSETS */

/*============================================================================*/
/* REGISTER BIT FIELD DEFINITIONS - MH                                        */
/*============================================================================*/

/*---------------------------------------------------------------------------*/
/* MH_CTRL Register Bit Fields (0x004)                                       */
/*---------------------------------------------------------------------------*/

/** @brief MH Enable bit - enables the Message Handler */
#define CAN_MH_CTRL_EN_POS (0U)
#define CAN_MH_CTRL_EN_MASK (0x00000001U)

/** @brief MH Software Reset bit */
#define CAN_MH_CTRL_SWRST_POS (1U)
#define CAN_MH_CTRL_SWRST_MASK (0x00000002U)

/*---------------------------------------------------------------------------*/
/* MH_CFG Register Bit Fields (0x008)                                        */
/*---------------------------------------------------------------------------*/

/** @brief Instance Number bit field */
#define CAN_MH_CFG_INST_NUM_POS (0U)
#define CAN_MH_CFG_INST_NUM_MASK (0x00000007U)

/*---------------------------------------------------------------------------*/
/* MH_STS Register Bit Fields (0x00C)                                        */
/*---------------------------------------------------------------------------*/

/** @brief MH Ready Status */
#define CAN_MH_STS_RDY_POS (0U)
#define CAN_MH_STS_RDY_MASK (0x00000001U)

/*---------------------------------------------------------------------------*/
/* MH_SFTY_CTRL Register Bit Fields (0x014)                                  */
/*---------------------------------------------------------------------------*/

/** @brief TX Descriptor CRC Enable */
#define CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_POS (0U)
#define CAN_MH_SFTY_CTRL_TX_DESC_CRC_EN_MASK (0x00000001U)

/** @brief RX Descriptor CRC Enable */
#define CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_POS (1U)
#define CAN_MH_SFTY_CTRL_RX_DESC_CRC_EN_MASK (0x00000002U)

/*============================================================================*/
/* REGISTER BIT FIELD DEFINITIONS - PRT                                       */
/*============================================================================*/

/*---------------------------------------------------------------------------*/
/* NBTP Register Bit Fields (0xA64) - Nominal Bit Timing                     */
/*---------------------------------------------------------------------------*/

/** @brief Nominal Synchronization Jump Width (NSJW) - Bits [6:0] */
#define CAN_NBTP_NSJW_POS (0U)
#define CAN_NBTP_NSJW_MASK (0x0000007FU)

/** @brief Nominal Time Segment 2 (Phase_Seg2) - Bits [14:8] */
#define CAN_NBTP_NTSEG2_POS (8U)
#define CAN_NBTP_NTSEG2_MASK (0x00007F00U)

/** @brief Nominal Time Segment 1 (Prop_Seg + Phase_Seg1) - Bits [24:16] */
#define CAN_NBTP_NTSEG1_POS (16U)
#define CAN_NBTP_NTSEG1_MASK (0x01FF0000U)

/** @brief Bit Rate Prescaler - Bits [29:25] */
#define CAN_NBTP_BRP_POS (25U)
#define CAN_NBTP_BRP_MASK (0x3E000000U)

/*---------------------------------------------------------------------------*/
/* DBTP Register Bit Fields (0xA68) - CAN FD Data Phase Bit Timing           */
/*---------------------------------------------------------------------------*/

/** @brief FD Data Phase Synchronization Jump Width (DSJW) - Bits [6:0] */
#define CAN_DBTP_DSJW_POS (0U)
#define CAN_DBTP_DSJW_MASK (0x0000007FU)

/** @brief FD Data Phase Time Segment 2 - Bits [14:8] */
#define CAN_DBTP_DTSEG2_POS (8U)
#define CAN_DBTP_DTSEG2_MASK (0x00007F00U)

/** @brief FD Data Phase Time Segment 1 - Bits [23:16] */
#define CAN_DBTP_DTSEG1_POS (16U)
#define CAN_DBTP_DTSEG1_MASK (0x00FF0000U)

/** @brief FD Transmitter Delay Compensation Offset - Bits [31:24] */
#define CAN_DBTP_DTDCO_POS (24U)
#define CAN_DBTP_DTDCO_MASK (0xFF000000U)

/*---------------------------------------------------------------------------*/
/* XBTP Register Bit Fields (0xA6C) - CAN XL Data Phase Bit Timing           */
/*---------------------------------------------------------------------------*/

/** @brief XL Data Phase Synchronization Jump Width (XSJW) - Bits [6:0] */
#define CAN_XBTP_XSJW_POS (0U)
#define CAN_XBTP_XSJW_MASK (0x0000007FU)

/** @brief XL Data Phase Time Segment 2 - Bits [14:8] */
#define CAN_XBTP_XTSEG2_POS (8U)
#define CAN_XBTP_XTSEG2_MASK (0x00007F00U)

/** @brief XL Data Phase Time Segment 1 - Bits [23:16] */
#define CAN_XBTP_XTSEG1_POS (16U)
#define CAN_XBTP_XTSEG1_MASK (0x00FF0000U)

/** @brief XL Transmitter Delay Compensation Offset - Bits [31:24] */
#define CAN_XBTP_XTDCO_POS (24U)
#define CAN_XBTP_XTDCO_MASK (0xFF000000U)

/*============================================================================*/
/* REGISTER BIT FIELD DEFINITIONS - IRC                                       */
/*============================================================================*/

/*---------------------------------------------------------------------------*/
/* FUNC_RAW / FUNC_ENA Bit Fields                                            */
/*---------------------------------------------------------------------------*/

/** @brief MH TX FIFO Queue 0 Interrupt */
#define CAN_IRC_FUNC_MH_TX_FQ0_IRQ_POS (0U)
#define CAN_IRC_FUNC_MH_TX_FQ0_IRQ_MASK (0x00000001U)

/** @brief MH TX FIFO Queue 1-7 Interrupts */
#define CAN_IRC_FUNC_MH_TX_FQ1_IRQ_MASK (0x00000002U)
#define CAN_IRC_FUNC_MH_TX_FQ2_IRQ_MASK (0x00000004U)
#define CAN_IRC_FUNC_MH_TX_FQ3_IRQ_MASK (0x00000008U)
#define CAN_IRC_FUNC_MH_TX_FQ4_IRQ_MASK (0x00000010U)
#define CAN_IRC_FUNC_MH_TX_FQ5_IRQ_MASK (0x00000020U)
#define CAN_IRC_FUNC_MH_TX_FQ6_IRQ_MASK (0x00000040U)
#define CAN_IRC_FUNC_MH_TX_FQ7_IRQ_MASK (0x00000080U)

/** @brief MH RX FIFO Queue 0-7 Interrupts */
#define CAN_IRC_FUNC_MH_RX_FQ0_IRQ_MASK (0x00000100U)
#define CAN_IRC_FUNC_MH_RX_FQ1_IRQ_MASK (0x00000200U)
#define CAN_IRC_FUNC_MH_RX_FQ2_IRQ_MASK (0x00000400U)
#define CAN_IRC_FUNC_MH_RX_FQ3_IRQ_MASK (0x00000800U)
#define CAN_IRC_FUNC_MH_RX_FQ4_IRQ_MASK (0x00001000U)
#define CAN_IRC_FUNC_MH_RX_FQ5_IRQ_MASK (0x00002000U)
#define CAN_IRC_FUNC_MH_RX_FQ6_IRQ_MASK (0x00004000U)
#define CAN_IRC_FUNC_MH_RX_FQ7_IRQ_MASK (0x00008000U)

/** @brief MH TX Priority Queue Interrupt */
#define CAN_IRC_FUNC_MH_TX_PQ_IRQ_MASK (0x00010000U)

/** @brief MH Stop Interrupt */
#define CAN_IRC_FUNC_MH_STOP_IRQ_MASK (0x00020000U)

/** @brief MH RX Filter Interrupt */
#define CAN_IRC_FUNC_MH_RX_FILTER_IRQ_MASK (0x00040000U)

/** @brief MH TX Filter Interrupt */
#define CAN_IRC_FUNC_MH_TX_FILTER_IRQ_MASK (0x00080000U)

/** @brief MH TX Abort Interrupt */
#define CAN_IRC_FUNC_MH_TX_ABORT_IRQ_MASK (0x00100000U)

/** @brief MH RX Abort Interrupt */
#define CAN_IRC_FUNC_MH_RX_ABORT_IRQ_MASK (0x00200000U)

/** @brief MH Statistics Interrupt */
#define CAN_IRC_FUNC_MH_STATS_IRQ_MASK (0x00400000U)

/** @brief PRT Error Active */
#define CAN_IRC_FUNC_PRT_E_ACTIVE_MASK (0x01000000U)

/** @brief PRT Bus On */
#define CAN_IRC_FUNC_PRT_BUS_ON_MASK (0x02000000U)

/** @brief PRT TX Event */
#define CAN_IRC_FUNC_PRT_TX_EVT_MASK (0x04000000U)

/** @brief PRT RX Event */
#define CAN_IRC_FUNC_PRT_RX_EVT_MASK (0x08000000U)

/*============================================================================*/
/* ENUMERATIONS                                                               */
/*============================================================================*/

/**
 * @brief CAN operating mode enumeration
 */
typedef enum {
  CAN_MODE_NORMAL = 0U,     /**< Normal mode operation */
  CAN_MODE_CONTINUOUS = 1U, /**< Continuous mode for RX FIFO */
  CAN_MODE_LOOPBACK = 2U,   /**< Internal loopback test mode */
  CAN_MODE_LISTEN = 3U,     /**< Listen only mode */
  CAN_MODE_SLEEP = 4U       /**< Power-down/sleep mode */
} can_mode_t;

/**
 * @brief CAN protocol type enumeration
 */
typedef enum {
  CAN_PROTOCOL_CC = 0U, /**< CAN Classic - up to 8 bytes, 1 Mbit/s */
  CAN_PROTOCOL_FD = 1U, /**< CAN FD - up to 64 bytes, 8 Mbit/s */
  CAN_PROTOCOL_XL = 2U  /**< CAN XL - up to 2048 bytes, 20 Mbit/s */
} can_protocol_t;

/**
 * @brief CAN error type enumeration
 */
typedef enum {
  CAN_ERROR_NONE = 0x00U,          /**< No error */
  CAN_ERROR_STUFF = 0x01U,         /**< Bit stuffing error */
  CAN_ERROR_FORM = 0x02U,          /**< Form error */
  CAN_ERROR_ACK = 0x03U,           /**< Acknowledgement error */
  CAN_ERROR_BIT1 = 0x04U,          /**< Bit 1 error */
  CAN_ERROR_BIT0 = 0x05U,          /**< Bit 0 error */
  CAN_ERROR_CRC = 0x06U,           /**< CRC error */
  CAN_ERROR_PROTOCOL = 0x07U,      /**< Protocol error */
  CAN_ERROR_BUS_OFF = 0x08U,       /**< Bus off state */
  CAN_ERROR_PASSIVE = 0x09U,       /**< Error passive state */
  CAN_ERROR_WARNING = 0x0AU,       /**< Error warning state */
  CAN_ERROR_ARB_LOST = 0x0BU,      /**< Arbitration lost */
  CAN_ERROR_DMA = 0x0CU,           /**< DMA error */
  CAN_ERROR_TIMEOUT = 0x0DU,       /**< Timeout error */
  CAN_ERROR_INVALID_PARAM = 0x0EU, /**< Invalid parameter */
  CAN_ERROR_QUEUE_FULL = 0x0FU,    /**< Queue full */
  CAN_ERROR_QUEUE_EMPTY = 0x10U,   /**< Queue empty */
  CAN_ERROR_DESC_CRC = 0x11U       /**< Descriptor CRC error */
} can_error_t;

/**
 * @brief TX/RX Descriptor Status (STS[3:0]) enumeration
 */
typedef enum {
  CAN_DESC_STS_NONE = 0x0U,     /**< No status */
  CAN_DESC_STS_SUCCESS = 0x1U,  /**< Message sent/received successfully */
  CAN_DESC_STS_NOT_SENT = 0x2U, /**< TX: Message not sent after retries */
  CAN_DESC_STS_NOT_FILTERED =
      0x2U,                        /**< RX: Message received but not filtered */
  CAN_DESC_STS_SKIPPED_HFI = 0x3U, /**< TX: Message skipped due to HFI */
  CAN_DESC_STS_TX_FILTER_REJECTED =
      0x4U /**< TX: Message rejected by TX filter */
} can_desc_status_t;

/**
 * @brief CAN frame type enumeration
 */
typedef enum {
  CAN_FRAME_DATA = 0U,  /**< Data frame */
  CAN_FRAME_REMOTE = 1U /**< Remote frame (CAN CC only) */
} can_frame_type_t;

/**
 * @brief CAN ID type enumeration
 */
typedef enum {
  CAN_ID_STANDARD = 0U, /**< Standard 11-bit ID */
  CAN_ID_EXTENDED = 1U  /**< Extended 29-bit ID */
} can_id_type_t;

/*============================================================================*/
/* TX DESCRIPTOR STRUCTURES (8 x 32-bit words = 32 bytes)                    */
/* Reference: X_CAN User Manual v3.9, Section 1.4.5.5                        */
/*============================================================================*/

/**
 * @brief TX Descriptor Element 0 - DMA Info Control 1
 * @details Bit field definitions for TX descriptor word 0
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t sts : 4;   /**< [3:0]   STS - Status (MH writes, SW reads) */
    uint32_t rc : 5;    /**< [8:4]   RC[4:0] - Rolling Counter */
    uint32_t rsvd0 : 2; /**< [10:9]  Reserved (set to 0) */
    uint32_t pqsn0 : 1; /**< [11]    PQSN[0] for PQ / reserved for FQ */
    uint32_t fqn : 4;   /**< [15:12] FQN[3:0] / PQSN[4:1] */
    uint32_t crc : 9;   /**< [24:16] CRC[8:0] */
    uint32_t end : 1;   /**< [25]    END - End of TX FIFO Queue (FQ only) */
    uint32_t pq : 1;    /**< [26]    PQ - Priority Queue flag */
    uint32_t irq : 1;   /**< [27]    IRQ - Interrupt request */
    uint32_t next : 1;  /**< [28]    NEXT - Must be 0 */
    uint32_t wrap : 1;  /**< [29]    WRAP - Wrap to start */
    uint32_t hd : 1;    /**< [30]    HD - Header Descriptor (must be 1) */
    uint32_t valid : 1; /**< [31]    VALID - Descriptor valid flag */
  } bits;
} can_tx_desc_elem0_t;

/**
 * @brief TX Descriptor Element 1 - DMA Info Control 2
 * @details Bit field definitions for TX descriptor word 1
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t rsvd0 : 2;     /**< [1:0]   Reserved (set to 0) */
    uint32_t nhdo_tdo : 10; /**< [11:2]  NHDO[9:0] for FQ / TDO[9:0] for PQ */
    uint32_t rsvd1 : 1;     /**< [12]    Reserved (set to 0) */
    uint32_t in : 3;        /**< [15:13] IN[2:0] - Instance Number */
    uint32_t
        size : 10; /**< [25:16] SIZE[9:0] - TX buffer size in 32-byte units */
    uint32_t
        plsrc : 1; /**< [26]    PLSRC - Payload source (0=S_MEM, 1=L_MEM) */
    uint32_t rsvd2 : 5; /**< [31:27] Reserved (set to 0) */
  } bits;
} can_tx_desc_elem1_t;

/**
 * @brief TX Message Header T0 - Classical CAN / CAN FD
 * @details Element 4 of TX descriptor
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t ext_id : 18;  /**< [17:0]  ExtID[17:0] - Extended ID bits */
    uint32_t base_id : 11; /**< [28:18] BaseID[10:0] - Base ID (11-bit) */
    uint32_t xtd : 1;      /**< [29]    XTD - Extended Identifier flag */
    uint32_t xlf : 1;      /**< [30]    XLF - XL Format (0 for CC/FD) */
    uint32_t fdf : 1;      /**< [31]    FDF - FD Format (0=CC, 1=FD) */
  } bits;
} can_tx_msg_hdr_t0_t;

/**
 * @brief TX Message Header T0 - CAN XL
 * @details Element 4 of TX descriptor for CAN XL frames
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t sdt : 8;      /**< [7:0]   SDT[7:0] - SDU Type */
    uint32_t vcid : 8;     /**< [15:8]  VCID[7:0] - Virtual CAN Network ID */
    uint32_t sec : 1;      /**< [16]    SEC - Simple Extended Content */
    uint32_t rrs : 1;      /**< [17]    RRS - Remote Request Substitution */
    uint32_t prio_id : 11; /**< [28:18] Priority ID[10:0] */
    uint32_t xtd : 1;      /**< [29]    XTD - Must be 0 for XL */
    uint32_t xlf : 1;      /**< [30]    XLF - XL Format (1 for XL) */
    uint32_t fdf : 1;      /**< [31]    FDF - FD Format (1 for XL) */
  } bits;
} can_tx_msg_hdr_t0_xl_t;

/**
 * @brief TX Message Header T1 - Classical CAN
 * @details Element 5 of TX descriptor for Classical CAN
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t rsvd0 : 16; /**< [15:0]  Reserved */
    uint32_t dlc : 4;    /**< [19:16] DLC[3:0] - Data Length Code */
    uint32_t rsvd1 : 6;  /**< [25:20] Reserved */
    uint32_t rtr : 1;    /**< [26]    RTR - Remote Transmission Request */
    uint32_t rsvd2 : 3;  /**< [29:27] Reserved */
    uint32_t fir : 1;    /**< [30]    FIR - Fault Injection Request */
    uint32_t rsvd3 : 1;  /**< [31]    Reserved */
  } bits;
} can_tx_msg_hdr_t1_cc_t;

/**
 * @brief TX Message Header T1 - CAN FD
 * @details Element 5 of TX descriptor for CAN FD
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t rsvd0 : 16; /**< [15:0]  Reserved */
    uint32_t dlc : 4;    /**< [19:16] DLC[3:0] - Data Length Code */
    uint32_t esi : 1;    /**< [20]    ESI - Error State Indicator */
    uint32_t rsvd1 : 4;  /**< [24:21] Reserved */
    uint32_t brs : 1;    /**< [25]    BRS - Bit Rate Switch */
    uint32_t rsvd2 : 1;  /**< [26]    Must be 0 */
    uint32_t rsvd3 : 3;  /**< [29:27] Reserved */
    uint32_t fir : 1;    /**< [30]    FIR - Fault Injection Request */
    uint32_t rsvd4 : 1;  /**< [31]    Reserved */
  } bits;
} can_tx_msg_hdr_t1_fd_t;

/**
 * @brief TX Message Header T1 - CAN XL
 * @details Element 5 of TX descriptor for CAN XL
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t rsvd0 : 16;  /**< [15:0]  Reserved */
    uint32_t dlc_xl : 11; /**< [26:16] DLC-XL[10:0] - Data Length Code */
    uint32_t rsvd1 : 3;   /**< [29:27] Reserved */
    uint32_t fir : 1;     /**< [30]    FIR - Fault Injection Request */
    uint32_t rsvd2 : 1;   /**< [31]    Reserved */
  } bits;
} can_tx_msg_hdr_t1_xl_t;

/**
 * @brief Complete TX Descriptor Structure (8 x 32-bit words)
 * @details Full TX FIFO Queue / TX Priority Queue descriptor
 * @note 32-byte aligned for AXI4-Lite and burst access
 */
typedef struct __attribute__((packed, aligned(4))) {
  can_tx_desc_elem0_t elem0; /**< Element 0: DMA Info Control 1 */
  can_tx_desc_elem1_t elem1; /**< Element 1: DMA Info Control 2 */
  uint32_t ts0;              /**< Element 2: Timestamp[31:0] (MH writes) */
  uint32_t ts1;              /**< Element 3: Timestamp[63:32] (MH writes) */
  uint32_t t0;               /**< Element 4: TX Message Header T0 */
  uint32_t t1;               /**< Element 5: TX Message Header T1 */
  uint32_t t2_td0;    /**< Element 6: T2 (XL) or TD0 (CC/FD first data) */
  uint32_t tx_ap_td1; /**< Element 7: TX_AP pointer or TD1 (CC second data) */
} tx_descriptor_t;

/*============================================================================*/
/* RX DESCRIPTOR STRUCTURES (4 x 32-bit words = 16 bytes)                    */
/* Reference: X_CAN User Manual v3.9, Section 1.4.5.7                        */
/*============================================================================*/

/**
 * @brief RX Descriptor Element 0 - DMA Info Control 1
 * @details Bit field definitions for RX descriptor word 0
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t sts : 4;   /**< [3:0]   STS - Status (MH writes) */
    uint32_t rc : 5;    /**< [8:4]   RC[4:0] - Rolling Counter */
    uint32_t in : 3;    /**< [11:9]  IN[2:0] - Instance Number */
    uint32_t fqn : 4;   /**< [15:12] FQN[3:0] - RX FIFO Queue Number */
    uint32_t crc : 9;   /**< [24:16] CRC[8:0] */
    uint32_t rsvd0 : 2; /**< [26:25] Reserved (set to 0) */
    uint32_t irq : 1;   /**< [27]    IRQ - Interrupt request */
    uint32_t next : 1;  /**< [28]    NEXT - More descriptors (MH writes) */
    uint32_t rsvd1 : 1; /**< [29]    Reserved (set to 0) */
    uint32_t hd : 1;    /**< [30]    HD - Header Descriptor (MH writes) */
    uint32_t valid : 1; /**< [31]    VALID - 0=valid, 1=used (SW/MH) */
  } bits;
} can_rx_desc_elem0_t;

/**
 * @brief Complete RX Descriptor Structure (4 x 32-bit words)
 * @details Full RX FIFO Queue descriptor (Normal Mode)
 * @note 16-byte aligned for AXI4-Lite burst access
 */
typedef struct __attribute__((packed, aligned(4))) {
  can_rx_desc_elem0_t elem0; /**< Element 0: DMA Info Control 1 */
  uint32_t rx_ap;            /**< Element 1: RX Address Pointer (S_MEM) */
  uint32_t ts0;              /**< Element 2: Timestamp[31:0] (MH writes) */
  uint32_t ts1;              /**< Element 3: Timestamp[63:32] (MH writes) */
} rx_descriptor_t;

/*============================================================================*/
/* RX MESSAGE HEADER STRUCTURE                                                */
/* Reference: X_CAN User Manual v3.9, Section 1.4.5.8                        */
/*============================================================================*/

/**
 * @brief RX Message Header R0 - Identifier and Flags
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t ext_id : 18;  /**< [17:0]  ExtID[17:0] / SDT+VCID (XL) */
    uint32_t base_id : 11; /**< [28:18] BaseID[10:0] / PrioID (XL) */
    uint32_t xtd : 1;      /**< [29]    XTD - Extended Identifier */
    uint32_t xlf : 1;      /**< [30]    XLF - XL Format */
    uint32_t fdf : 1;      /**< [31]    FDF - FD Format */
  } bits;
} can_rx_msg_hdr_r0_t;

/**
 * @brief RX Message Header R1 - DLC and Flags
 */
typedef union {
  uint32_t word;
  struct {
    uint32_t rsvd0 : 16;   /**< [15:0]  Reserved / AF[15:0] (XL) */
    uint32_t dlc : 4;      /**< [19:16] DLC[3:0] or DLC-XL[3:0] */
    uint32_t esi : 1;      /**< [20]    ESI - Error State Indicator */
    uint32_t rsvd1 : 4;    /**< [24:21] Reserved */
    uint32_t brs : 1;      /**< [25]    BRS - Bit Rate Switch */
    uint32_t rtr : 1;      /**< [26]    RTR - Remote Transmission Request */
    uint32_t dlc_xl_h : 7; /**< [26:27]+[31:28] DLC-XL[10:4] for XL */
  } bits;
} can_rx_msg_hdr_r1_t;

/**
 * @brief Complete RX Message Header (written to S_MEM)
 */
typedef struct __attribute__((packed, aligned(4))) {
  can_rx_msg_hdr_r0_t r0; /**< R0: ID and format flags */
  can_rx_msg_hdr_r1_t r1; /**< R1: DLC and status flags */
  uint32_t r2;            /**< R2: AF[31:16] for XL / Reserved */
} rx_msg_header_t;

/*============================================================================*/
/* CAN MESSAGE STRUCTURE (API-level message representation)                   */
/*============================================================================*/

/**
 * @brief CAN Message Structure for RX/TX API
 * @details Unified message structure for application layer use
 */
typedef struct {
  uint32_t id;              /**< CAN identifier (11 or 29 bits) */
  uint8_t data[2048];       /**< Message data (up to 2048 bytes for XL) */
  uint32_t len;             /**< Data length in bytes */
  uint64_t timestamp;       /**< Hardware timestamp (if available) */
  uint8_t fifo_id;          /**< FIFO queue that received the message */
  bool extended;            /**< True if extended (29-bit) ID */
  bool fd;                  /**< True if CAN FD frame */
  bool xl;                  /**< True if CAN XL frame */
  bool brs;                 /**< Bit Rate Switch flag (FD/XL) */
  bool esi;                 /**< Error State Indicator */
  bool rtr;                 /**< Remote Transmission Request (CC only) */
  uint8_t vcid;             /**< Virtual CAN Network ID (XL only) */
  uint8_t sdt;              /**< SDU Type (XL only) */
  can_desc_status_t status; /**< Descriptor status after reception */
} can_msg_t;

/*============================================================================*/
/* CAN BUS STATE ENUMERATION                                                  */
/*============================================================================*/

/**
 * @brief CAN Bus State Enumeration
 */
typedef enum {
  CAN_BUS_STATE_ACTIVE = 0U,  /**< Error Active state */
  CAN_BUS_STATE_WARNING = 1U, /**< Error Warning state (TEC/REC > 96) */
  CAN_BUS_STATE_PASSIVE = 2U, /**< Error Passive state (TEC/REC > 127) */
  CAN_BUS_STATE_BUS_OFF = 3U  /**< Bus Off state (TEC > 255) */
} can_bus_state_t;

/*============================================================================*/
/* CAN STATISTICS STRUCTURE                                                   */
/*============================================================================*/

/**
 * @brief CAN Statistics Structure
 * @details Contains error counters and status information
 */
typedef struct {
  uint8_t tx_error_count;    /**< Transmit Error Counter (TEC) */
  uint8_t rx_error_count;    /**< Receive Error Counter (REC) */
  can_bus_state_t bus_state; /**< Current bus state */
  uint32_t tx_success_count; /**< Successful TX message count */
  uint32_t rx_success_count; /**< Successful RX message count */
  uint32_t tx_error_frames;  /**< TX error frame count */
  uint32_t rx_error_frames;  /**< RX error frame count */
  uint32_t arb_lost_count;   /**< Arbitration lost count */
  uint32_t bus_off_count;    /**< Bus-off occurrences */
  uint32_t overrun_count;    /**< FIFO overrun count */
  bool error_warning;        /**< Error warning flag */
  bool error_passive;        /**< Error passive flag */
  bool bus_off;              /**< Bus off flag */
  uint8_t last_error_code;   /**< Last error code (LEC) */
} can_stats_t;

/*============================================================================*/
/* INTERRUPT CALLBACK TYPES                                                   */
/*============================================================================*/

/**
 * @brief RX FIFO callback function type
 * @param fifo_id  FIFO queue index that received message
 * @param user_ctx User context pointer
 */
typedef void (*can_rx_callback_t)(uint8_t fifo_id, void *user_ctx);

/**
 * @brief TX FIFO callback function type
 * @param fifo_id  FIFO queue index that completed transmission
 * @param user_ctx User context pointer
 */
typedef void (*can_tx_callback_t)(uint8_t fifo_id, void *user_ctx);

/**
 * @brief TX Priority Queue callback function type
 * @param slot_id  PQ slot that completed transmission
 * @param user_ctx User context pointer
 */
typedef void (*can_tx_pq_callback_t)(uint8_t slot_id, void *user_ctx);

/**
 * @brief Error callback function type
 * @param error    Error type
 * @param user_ctx User context pointer
 */
typedef void (*can_error_callback_t)(can_error_t error, void *user_ctx);

/**
 * @brief Bus state change callback function type
 * @param new_state New bus state
 * @param user_ctx  User context pointer
 */
typedef void (*can_bus_state_callback_t)(can_bus_state_t new_state,
                                         void *user_ctx);

/**
 * @brief Interrupt callback configuration structure
 */
typedef struct {
  can_rx_callback_t rx_callbacks[CAN_RX_FIFO_QUEUE_COUNT];
  can_tx_callback_t tx_callbacks[CAN_TX_FIFO_QUEUE_COUNT];
  can_tx_pq_callback_t tx_pq_callback;
  can_error_callback_t error_callback;
  can_bus_state_callback_t bus_state_callback;
  void *user_ctx; /**< User context passed to all callbacks */
} can_irq_callbacks_t;

/*============================================================================*/
/* CONFIGURATION STRUCTURES                                                   */
/*============================================================================*/

/**
 * @brief Bit Timing Configuration Structure
 * @details Configures bit timing parameters for nominal, data, and XL phases
 */
typedef struct {
  uint8_t brp;    /**< Baud rate prescaler (0-31, actual = BRP+1) */
  uint16_t tseg1; /**< Time segment 1 (1-511 for nominal, 0-255 for data/XL) */
  uint8_t tseg2;  /**< Time segment 2 (1-127) */
  uint8_t sjw;    /**< Synchronization jump width (0-127) */
  uint8_t tdco;   /**< Transmitter delay compensation offset (data/XL only) */
  uint8_t padding[1]; /**< Alignment padding */
} can_bit_timing_t;

/**
 * @brief Queue Configuration Structure
 */
typedef struct {
  uint32_t start_addr;    /**< Queue start address in system memory */
  uint32_t dc_start_addr; /**< Data container start address (RX only) */
  uint16_t size;          /**< Queue size (number of descriptors) */
  uint16_t dc_size;       /**< Data container size in 32-byte units */
  bool enabled;           /**< Queue enable flag */
  bool continuous;        /**< Continuous mode (RX only) */
  uint8_t padding[2];     /**< Alignment padding */
} can_queue_config_t;

/**
 * @brief CAN Controller Configuration Structure
 */
typedef struct {
  /* Base addresses */
  uint32_t base_addr;      /**< CAN controller base address */
  uint32_t lmem_base_addr; /**< Local Memory base address */
  uint32_t lmem_size;      /**< Local Memory size in bytes */

  /* Protocol selection */
  can_protocol_t protocol; /**< CAN CC/FD/XL selection */
  can_mode_t mode;         /**< Operating mode */

  /* Bit timing configuration */
  can_bit_timing_t
      nominal_timing;           /**< Nominal bit timing (arbitration phase) */
  can_bit_timing_t data_timing; /**< Data phase timing (CAN FD) */
  can_bit_timing_t xl_timing;   /**< XL phase timing (CAN XL) */

  /* Queue configurations */
  can_queue_config_t tx_fifo_queues[CAN_TX_FIFO_QUEUE_COUNT];
  can_queue_config_t rx_fifo_queues[CAN_RX_FIFO_QUEUE_COUNT];

  /* TX Priority Queue */
  uint32_t txpq_start_addr; /**< TX Priority Queue start address */
  uint8_t txpq_slot_count;  /**< Number of TX PQ slots (1-32) */
  uint8_t padding1[3];      /**< Alignment padding */

  /* Filter configuration */
  uint32_t rx_filter_base_addr; /**< RX filter base address in LMEM */
  uint32_t tx_desc_base_addr;   /**< TX descriptor base address in LMEM */
  uint8_t rx_filter_count;      /**< Number of RX filter elements */
  uint8_t instance_num;         /**< Instance number (0-7) */
  uint8_t padding2[2];          /**< Alignment padding */

  /* Interrupt configuration */
  uint32_t func_int_enable;   /**< Functional interrupt enable mask */
  uint32_t err_int_enable;    /**< Error interrupt enable mask */
  uint32_t safety_int_enable; /**< Safety interrupt enable mask */

  /* Options */
  bool loopback_enable;    /**< Enable loopback mode */
  bool listen_only;        /**< Enable listen-only mode */
  bool tx_desc_crc_enable; /**< Enable TX descriptor CRC check */
  bool rx_desc_crc_enable; /**< Enable RX descriptor CRC check */
} can_config_t;

/*============================================================================*/
/* REGISTER ACCESS MACROS                                                     */
/*============================================================================*/

/** @brief Direct register read (uses uintptr_t for proper pointer cast) */
#define CAN_REG_READ(addr) (*((volatile uint32_t *)(uintptr_t)(addr)))

/** @brief Direct register write (uses uintptr_t for proper pointer cast) */
#define CAN_REG_WRITE(addr, val)                                               \
  (*((volatile uint32_t *)(uintptr_t)(addr)) = (val))

/** @brief Read register at base + offset */
#define CAN_READ_REG(base, offset) CAN_REG_READ((base) + (offset))

/** @brief Write register at base + offset */
#define CAN_WRITE_REG(base, offset, val) CAN_REG_WRITE((base) + (offset), (val))

/** @brief Set bits in register */
#define CAN_SET_BITS(base, offset, mask)                                       \
  CAN_WRITE_REG(base, offset, CAN_READ_REG(base, offset) | (mask))

/** @brief Clear bits in register */
#define CAN_CLR_BITS(base, offset, mask)                                       \
  CAN_WRITE_REG(base, offset, CAN_READ_REG(base, offset) & ~(mask))

/*============================================================================*/
/* HELPER MACROS FOR BIT TIMING CONFIGURATION                                 */
/*============================================================================*/

/**
 * @brief Build NBTP register value
 */
#define CAN_BUILD_NBTP(brp, tseg1, tseg2, sjw)                                 \
  ((((uint32_t)(brp) << CAN_NBTP_BRP_POS) & CAN_NBTP_BRP_MASK) |               \
   (((uint32_t)(tseg1) << CAN_NBTP_NTSEG1_POS) & CAN_NBTP_NTSEG1_MASK) |       \
   (((uint32_t)(tseg2) << CAN_NBTP_NTSEG2_POS) & CAN_NBTP_NTSEG2_MASK) |       \
   (((uint32_t)(sjw) << CAN_NBTP_NSJW_POS) & CAN_NBTP_NSJW_MASK))

/**
 * @brief Build DBTP register value
 */
#define CAN_BUILD_DBTP(dtdco, dtseg1, dtseg2, dsjw)                            \
  ((((uint32_t)(dtdco) << CAN_DBTP_DTDCO_POS) & CAN_DBTP_DTDCO_MASK) |         \
   (((uint32_t)(dtseg1) << CAN_DBTP_DTSEG1_POS) & CAN_DBTP_DTSEG1_MASK) |      \
   (((uint32_t)(dtseg2) << CAN_DBTP_DTSEG2_POS) & CAN_DBTP_DTSEG2_MASK) |      \
   (((uint32_t)(dsjw) << CAN_DBTP_DSJW_POS) & CAN_DBTP_DSJW_MASK))

/**
 * @brief Build XBTP register value
 */
#define CAN_BUILD_XBTP(xtdco, xtseg1, xtseg2, xsjw)                            \
  ((((uint32_t)(xtdco) << CAN_XBTP_XTDCO_POS) & CAN_XBTP_XTDCO_MASK) |         \
   (((uint32_t)(xtseg1) << CAN_XBTP_XTSEG1_POS) & CAN_XBTP_XTSEG1_MASK) |      \
   (((uint32_t)(xtseg2) << CAN_XBTP_XTSEG2_POS) & CAN_XBTP_XTSEG2_MASK) |      \
   (((uint32_t)(xsjw) << CAN_XBTP_XSJW_POS) & CAN_XBTP_XSJW_MASK))

/*============================================================================*/
/* HELPER MACROS FOR TX MESSAGE HEADER                                        */
/*============================================================================*/

/**
 * @brief Build T0 for Classical CAN / CAN FD with standard ID
 */
#define CAN_BUILD_T0_STD(base_id, fdf)                                         \
  ((((uint32_t)(base_id) << 18U) & 0x1FFC0000U) |                              \
   (((uint32_t)(fdf) << 31U) & 0x80000000U))

/**
 * @brief Build T0 for Classical CAN / CAN FD with extended ID
 */
#define CAN_BUILD_T0_EXT(base_id, ext_id, fdf)                                 \
  (((uint32_t)(ext_id)&0x0003FFFFU) |                                          \
   (((uint32_t)(base_id) << 18U) & 0x1FFC0000U) | (1U << 29U) |                \
   (((uint32_t)(fdf) << 31U) & 0x80000000U))

/**
 * @brief Build T0 for CAN XL
 */
#define CAN_BUILD_T0_XL(prio_id, vcid, sdt, sec, rrs)                          \
  ((((uint32_t)(sdt)&0xFFU)) | (((uint32_t)(vcid)&0xFFU) << 8U) |              \
   (((uint32_t)(sec)&0x1U) << 16U) | (((uint32_t)(rrs)&0x1U) << 17U) |         \
   (((uint32_t)(prio_id)&0x7FFU) << 18U) |                                     \
   (0xC0000000U)) /* FDF=1, XLF=1, XTD=0 */

/**
 * @brief Build T1 for Classical CAN
 */
#define CAN_BUILD_T1_CC(dlc, rtr)                                              \
  ((((uint32_t)(dlc)&0xFU) << 16U) | (((uint32_t)(rtr)&0x1U) << 26U))

/**
 * @brief Build T1 for CAN FD
 */
#define CAN_BUILD_T1_FD(dlc, brs, esi)                                         \
  ((((uint32_t)(dlc)&0xFU) << 16U) | (((uint32_t)(esi)&0x1U) << 20U) |         \
   (((uint32_t)(brs)&0x1U) << 25U))

/**
 * @brief Build T1 for CAN XL
 */
#define CAN_BUILD_T1_XL(dlc_xl) (((uint32_t)(dlc_xl)&0x7FFU) << 16U)

/*============================================================================*/
/* DLC TO BYTE LENGTH CONVERSION                                              */
/*============================================================================*/

/**
 * @brief DLC to data length lookup table for CAN FD
 */
static const uint8_t can_dlc_to_len[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

/**
 * @brief Convert DLC to data length
 */
#define CAN_DLC_TO_LEN(dlc) (can_dlc_to_len[(dlc)&0x0FU])

/**
 * @brief Convert data length to DLC for CAN FD
 */
static inline uint8_t can_len_to_dlc(uint16_t len) {
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
  return 15U; /* 49-64 bytes */
}

/*============================================================================*/
/* STATIC ASSERTIONS FOR STRUCTURE SIZES                                      */
/*============================================================================*/

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(tx_descriptor_t) == 32U,
               "TX descriptor must be 32 bytes");
_Static_assert(sizeof(rx_descriptor_t) == 16U,
               "RX descriptor must be 16 bytes");
_Static_assert(sizeof(can_tx_desc_elem0_t) == 4U,
               "TX desc elem0 must be 4 bytes");
_Static_assert(sizeof(can_tx_desc_elem1_t) == 4U,
               "TX desc elem1 must be 4 bytes");
_Static_assert(sizeof(can_rx_desc_elem0_t) == 4U,
               "RX desc elem0 must be 4 bytes");
#endif

/*============================================================================*/
/* FUNCTION PROTOTYPES (Driver API)                                           */
/*============================================================================*/

/**
 * @brief Initialize CAN controller
 * @param[in] config Pointer to configuration structure
 * @return can_error_t Error code
 */
can_error_t can_init(const can_config_t *config);

/**
 * @brief Deinitialize CAN controller
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_deinit(uint32_t base_addr);

/**
 * @brief Start CAN controller (enable MH)
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_start(uint32_t base_addr);

/**
 * @brief Stop CAN controller
 * @param[in] base_addr CAN controller base address
 * @return can_error_t Error code
 */
can_error_t can_stop(uint32_t base_addr);

/**
 * @brief Configure bit timing
 * @param[in] base_addr CAN controller base address
 * @param[in] nominal Pointer to nominal timing configuration
 * @param[in] data Pointer to data phase timing (NULL for CAN CC)
 * @param[in] xl Pointer to XL phase timing (NULL for CAN CC/FD)
 * @return can_error_t Error code
 */
can_error_t can_set_bit_timing(uint32_t base_addr,
                               const can_bit_timing_t *nominal,
                               const can_bit_timing_t *data,
                               const can_bit_timing_t *xl);

/**
 * @brief Start TX FIFO queue
 * @param[in] base_addr CAN controller base address
 * @param[in] queue_idx Queue index (0-7)
 * @param[in] config Pointer to queue configuration
 * @return can_error_t Error code
 */
can_error_t can_tx_fifo_start(uint32_t base_addr, uint8_t queue_idx,
                              const can_queue_config_t *config);

/**
 * @brief Start RX FIFO queue
 * @param[in] base_addr CAN controller base address
 * @param[in] queue_idx Queue index (0-7)
 * @param[in] config Pointer to queue configuration
 * @return can_error_t Error code
 */
can_error_t can_rx_fifo_start(uint32_t base_addr, uint8_t queue_idx,
                              const can_queue_config_t *config);

/**
 * @brief Enable interrupts
 * @param[in] base_addr CAN controller base address
 * @param[in] func_mask Functional interrupt mask
 * @param[in] err_mask Error interrupt mask
 * @param[in] safety_mask Safety interrupt mask
 * @return can_error_t Error code
 */
can_error_t can_interrupt_enable(uint32_t base_addr, uint32_t func_mask,
                                 uint32_t err_mask, uint32_t safety_mask);

/**
 * @brief Clear interrupt flags
 * @param[in] base_addr CAN controller base address
 * @param[in] func_mask Functional interrupt mask to clear
 * @param[in] err_mask Error interrupt mask to clear
 * @param[in] safety_mask Safety interrupt mask to clear
 * @return can_error_t Error code
 */
can_error_t can_interrupt_clear(uint32_t base_addr, uint32_t func_mask,
                                uint32_t err_mask, uint32_t safety_mask);

/**
 * @brief Get raw interrupt status
 * @param[in] base_addr CAN controller base address
 * @param[out] func_status Pointer to functional interrupt status
 * @param[out] err_status Pointer to error interrupt status
 * @param[out] safety_status Pointer to safety interrupt status
 * @return can_error_t Error code
 */
can_error_t can_interrupt_get_raw_status(uint32_t base_addr,
                                         uint32_t *func_status,
                                         uint32_t *err_status,
                                         uint32_t *safety_status);

/*============================================================================*/
/* TX MESSAGE API FUNCTION PROTOTYPES                                         */
/*============================================================================*/

/**
 * @brief Push a message to TX FIFO queue
 * @details Constructs TX descriptor and writes to SMEM. Handles CAN CC/FD/XL.
 *          Reference: Manual Section 1.4.7.6 TX FIFO Queue Initial Start
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   TX FIFO queue index (0-7)
 * @param[in] id        CAN message identifier (11-bit or 29-bit)
 * @param[in] data      Pointer to data payload
 * @param[in] len       Data length in bytes (0-8 CC, 0-64 FD, 0-2048 XL)
 * @param[in] fd        True for CAN FD frame, false for Classical CAN
 * @param[in] xl        True for CAN XL frame (overrides fd flag)
 * @param[in] remote    True for remote frame (CC only, ignored for FD/XL)
 * @return can_error_t  CAN_ERROR_NONE on success,
 *                      CAN_ERROR_QUEUE_FULL if FIFO is full,
 *                      CAN_ERROR_INVALID_PARAM for invalid parameters
 */
can_error_t can_tx_fifo_push(uint32_t base_addr, uint8_t fifo_id, uint32_t id,
                             const uint8_t *data, uint32_t len, bool fd,
                             bool xl, bool remote);

/**
 * @brief Push a message to TX FIFO queue with extended options
 * @details Same as can_tx_fifo_push but with additional CAN FD/XL options
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   TX FIFO queue index (0-7)
 * @param[in] id        CAN message identifier (11-bit or 29-bit)
 * @param[in] extended  True for 29-bit extended ID, false for 11-bit standard
 * @param[in] data      Pointer to data payload
 * @param[in] len       Data length in bytes
 * @param[in] fd        True for CAN FD frame
 * @param[in] xl        True for CAN XL frame
 * @param[in] brs       Bit Rate Switch (FD/XL only)
 * @param[in] remote    True for remote frame (CC only)
 * @return can_error_t  Error code
 */
can_error_t can_tx_fifo_push_ext(uint32_t base_addr, uint8_t fifo_id,
                                 uint32_t id, bool extended,
                                 const uint8_t *data, uint32_t len, bool fd,
                                 bool xl, bool brs, bool remote);

/**
 * @brief Send a message via TX Priority Queue slot
 * @details Uses slot-based mechanism for high-priority messages.
 *          Reference: Manual Section 1.4.7.9-1.4.7.11
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] slot_id   TX Priority Queue slot index (0-31)
 * @param[in] id        CAN message identifier (11-bit or 29-bit)
 * @param[in] extended  True for 29-bit extended ID
 * @param[in] data      Pointer to data payload
 * @param[in] len       Data length in bytes
 * @param[in] fd        True for CAN FD frame
 * @param[in] xl        True for CAN XL frame
 * @param[in] brs       Bit Rate Switch (FD/XL only)
 * @return can_error_t  CAN_ERROR_NONE on success,
 *                      CAN_ERROR_QUEUE_FULL if slot is busy,
 *                      CAN_ERROR_INVALID_PARAM for invalid parameters
 */
can_error_t can_tx_priority_slot(uint32_t base_addr, uint8_t slot_id,
                                 uint32_t id, bool extended,
                                 const uint8_t *data, uint32_t len, bool fd,
                                 bool xl, bool brs);

/**
 * @brief Abort TX FIFO queue
 * @details Aborts the specified TX FIFO queue. Per Section 1.4.7.8.
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   TX FIFO queue index (0-7)
 * @return can_error_t  CAN_ERROR_NONE on success,
 *                      CAN_ERROR_TIMEOUT if abort does not complete,
 *                      CAN_ERROR_INVALID_PARAM for invalid fifo_id
 */
can_error_t can_tx_abort(uint32_t base_addr, uint8_t fifo_id);

/**
 * @brief Abort TX Priority Queue slot
 * @details Aborts the specified TX PQ slot. Per Section 1.4.7.11.
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] slot_id   TX Priority Queue slot index (0-31)
 * @return can_error_t  CAN_ERROR_NONE on success,
 *                      CAN_ERROR_TIMEOUT if abort does not complete,
 *                      CAN_ERROR_INVALID_PARAM for invalid slot_id
 */
can_error_t can_tx_priority_abort(uint32_t base_addr, uint8_t slot_id);

/**
 * @brief Check if TX FIFO queue is busy
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   TX FIFO queue index (0-7)
 * @param[out] is_busy  Pointer to receive busy status
 * @return can_error_t  Error code
 */
can_error_t can_tx_fifo_is_busy(uint32_t base_addr, uint8_t fifo_id,
                                bool *is_busy);

/**
 * @brief Check if TX Priority Queue slot is busy
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] slot_id   TX PQ slot index (0-31)
 * @param[out] is_busy  Pointer to receive busy status
 * @return can_error_t  Error code
 */
can_error_t can_tx_priority_is_busy(uint32_t base_addr, uint8_t slot_id,
                                    bool *is_busy);

/*============================================================================*/
/* RX MESSAGE API FUNCTION PROTOTYPES                                         */
/*============================================================================*/

/**
 * @brief Setup RX FIFO queue
 * @details Configures RX FIFO queue registers. Must be called before starting.
 *          Reference: Manual Section 1.4.7.3 RX FIFO Queue Initial Start
 *
 * @param[in] base_addr      CAN controller base address
 * @param[in] fifo_id        RX FIFO queue index (0-7)
 * @param[in] desc_phys_addr Physical address of descriptor linked list in S_MEM
 * @param[in] max_desc       Maximum number of descriptors (queue depth)
 * @param[in] dc_size        Data container size in 32-byte units
 * @param[in] continuous     True for continuous mode, false for normal mode
 * @return can_error_t       CAN_ERROR_NONE on success
 *
 * @note Filtering: Specific Message IDs are routed to specific FIFOs via
 *       RX_FILTER_CTRL logic configured during initialization. The filter
 *       configuration determines which messages end up in which FIFO.
 */
can_error_t can_rx_fifo_setup(uint32_t base_addr, uint8_t fifo_id,
                              uint32_t desc_phys_addr, uint16_t max_desc,
                              uint32_t dc_size, bool continuous);

/**
 * @brief Setup RX FIFO queue for continuous mode
 * @details Extended setup with data container base address for continuous mode.
 *
 * @param[in] base_addr      CAN controller base address
 * @param[in] fifo_id        RX FIFO queue index (0-7)
 * @param[in] desc_phys_addr Physical address of descriptor linked list
 * @param[in] max_desc       Maximum number of descriptors
 * @param[in] dc_start_addr  Data container start address (continuous mode)
 * @param[in] dc_size        Data container size in 32-byte units
 * @return can_error_t       CAN_ERROR_NONE on success
 */
can_error_t can_rx_fifo_setup_continuous(uint32_t base_addr, uint8_t fifo_id,
                                         uint32_t desc_phys_addr,
                                         uint16_t max_desc,
                                         uint32_t dc_start_addr,
                                         uint32_t dc_size);

/**
 * @brief Read a message from RX FIFO queue
 * @details Polls for new messages and reads descriptor + data from S_MEM.
 *          Reference: Manual Section 1.4.7.3-1.4.7.4
 *
 * @param[in]  base_addr  CAN controller base address
 * @param[in]  fifo_id    RX FIFO queue index (0-7)
 * @param[out] msg        Pointer to message structure to fill
 * @param[in]  timeout_us Timeout in microseconds (0 = non-blocking)
 * @return can_error_t    CAN_ERROR_NONE on success,
 *                        CAN_ERROR_QUEUE_EMPTY if no message available,
 *                        CAN_ERROR_TIMEOUT if timeout expired
 */
can_error_t can_rx_read(uint32_t base_addr, uint8_t fifo_id, can_msg_t *msg,
                        uint32_t timeout_us);

/**
 * @brief Check if RX FIFO has new messages
 *
 * @param[in]  base_addr  CAN controller base address
 * @param[in]  fifo_id    RX FIFO queue index (0-7)
 * @param[out] has_msg    Pointer to receive result (true if message available)
 * @return can_error_t    Error code
 */
can_error_t can_rx_has_message(uint32_t base_addr, uint8_t fifo_id,
                               bool *has_msg);

/**
 * @brief Restart RX FIFO queue
 * @details Restarts an RX FIFO queue that was stopped or on hold.
 *          Reference: Manual Section 1.4.7.4 Restarting a RX FIFO Queue
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   RX FIFO queue index (0-7)
 */
void can_rx_restart(uint32_t base_addr, uint8_t fifo_id);

/**
 * @brief Abort RX FIFO queue
 * @details Aborts the specified RX FIFO queue. Per Section 1.4.7.5.
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   RX FIFO queue index (0-7)
 * @return can_error_t  CAN_ERROR_NONE on success
 */
can_error_t can_rx_abort(uint32_t base_addr, uint8_t fifo_id);

/**
 * @brief Check if RX FIFO queue is busy
 *
 * @param[in]  base_addr CAN controller base address
 * @param[in]  fifo_id   RX FIFO queue index (0-7)
 * @param[out] is_busy   Pointer to receive busy status
 * @return can_error_t   Error code
 */
can_error_t can_rx_fifo_is_busy(uint32_t base_addr, uint8_t fifo_id,
                                bool *is_busy);

/**
 * @brief Get RX FIFO queue fill level
 * @details Returns approximate number of received messages in queue
 *
 * @param[in]  base_addr  CAN controller base address
 * @param[in]  fifo_id    RX FIFO queue index (0-7)
 * @param[out] fill_level Pointer to receive fill level
 * @return can_error_t    Error code
 */
can_error_t can_rx_get_fill_level(uint32_t base_addr, uint8_t fifo_id,
                                  uint32_t *fill_level);

/**
 * @brief Update RX read pointer (continuous mode)
 * @details Manually updates RX_FQ_RD_ADD_PT for continuous mode operation
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] fifo_id   RX FIFO queue index (0-7)
 * @param[in] new_addr  New read pointer address
 * @return can_error_t  Error code
 */
can_error_t can_rx_update_read_ptr(uint32_t base_addr, uint8_t fifo_id,
                                   uint32_t new_addr);

/*============================================================================*/
/* INTERRUPT HANDLING API                                                     */
/*============================================================================*/

/**
 * @brief Register interrupt callbacks
 * @details Sets up callback functions for various CAN events
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] callbacks Pointer to callback configuration structure
 * @return can_error_t  CAN_ERROR_NONE on success
 */
can_error_t can_register_callbacks(uint32_t base_addr,
                                   const can_irq_callbacks_t *callbacks);

/**
 * @brief CAN Interrupt Handler
 * @details Main IRQ handler - reads IRC status, clears interrupts, dispatches
 *          callbacks. Should be called from the system's CAN ISR.
 *
 * @param[in] base_addr CAN controller base address
 */
void can_irq_handler(uint32_t base_addr);

/**
 * @brief Get pending interrupt status
 * @details Returns the raw interrupt status without clearing
 *
 * @param[in]  base_addr     CAN controller base address
 * @param[out] func_pending  Functional interrupt flags
 * @param[out] err_pending   Error interrupt flags
 * @param[out] safety_pending Safety interrupt flags
 * @return can_error_t       Error code
 */
can_error_t can_get_irq_pending(uint32_t base_addr, uint32_t *func_pending,
                                uint32_t *err_pending,
                                uint32_t *safety_pending);

/*============================================================================*/
/* STATISTICS AND STATUS API                                                  */
/*============================================================================*/

/**
 * @brief Get CAN statistics and error counters
 * @details Reads TEC, REC from PRT STAT register and other status info
 *
 * @param[in]  base_addr CAN controller base address
 * @param[out] stats     Pointer to statistics structure to fill
 * @return can_error_t   CAN_ERROR_NONE on success
 */
can_error_t can_get_stats(uint32_t base_addr, can_stats_t *stats);

/**
 * @brief Get current bus state
 *
 * @param[in]  base_addr CAN controller base address
 * @param[out] state     Pointer to receive bus state
 * @return can_error_t   Error code
 */
can_error_t can_get_bus_state(uint32_t base_addr, can_bus_state_t *state);

/**
 * @brief Clear statistics counters
 *
 * @param[in] base_addr CAN controller base address
 * @return can_error_t  Error code
 */
can_error_t can_clear_stats(uint32_t base_addr);

/*============================================================================*/
/* UTILITY FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Set loopback mode
 * @details Configures PRT TEST register for internal loopback
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] enable    True to enable loopback, false to disable
 * @return can_error_t  CAN_ERROR_NONE on success
 */
can_error_t can_set_loopback(uint32_t base_addr, bool enable);

/**
 * @brief Set listen-only mode
 * @details Configures PRT MODE register for listen-only operation
 *
 * @param[in] base_addr CAN controller base address
 * @param[in] enable    True to enable listen-only, false to disable
 * @return can_error_t  CAN_ERROR_NONE on success
 */
can_error_t can_set_listen_only(uint32_t base_addr, bool enable);

/**
 * @brief Get controller version information
 *
 * @param[in]  base_addr   CAN controller base address
 * @param[out] mh_version  MH release version
 * @param[out] prt_version PRT release version
 * @return can_error_t     Error code
 */
can_error_t can_get_version(uint32_t base_addr, uint32_t *mh_version,
                            uint32_t *prt_version);

/**
 * @brief Perform software reset
 * @details Resets PRT and MH to initial state
 *
 * @param[in] base_addr CAN controller base address
 * @return can_error_t  CAN_ERROR_NONE on success
 */
can_error_t can_software_reset(uint32_t base_addr);

/**
 * @brief Write register with full AXI write strobes
 * @details Ensures all 4 byte lanes are written (strobes = 0xF)
 *
 * @param[in] addr  Register address
 * @param[in] value Value to write
 */
static inline void can_reg32_write(uint32_t addr, uint32_t value) {
  *((volatile uint32_t *)(uintptr_t)addr) = value;
}

/**
 * @brief Read register
 *
 * @param[in] addr Register address
 * @return uint32_t Register value
 */
static inline uint32_t can_reg32_read(uint32_t addr) {
  return *((volatile uint32_t *)(uintptr_t)addr);
}

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */

/* End of file can_driver.h */
