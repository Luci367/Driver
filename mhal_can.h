/******************************************************************************
 *  File: mhal_can.h
 *  Module: CAN MHAL (Hardware Abstraction Layer)
 *
 *  HAL layer API prototypes for CAN CC/FD/XL driver.
 *  Called by MCAL layer (Can.c / CanXL.c).
 *
 *  Types provided by the MCAL (Can_PduType, CanXL_PduType, Can_HwType,
 *  Can_HwHandleType, Can_StateTransitionType, Can_ControllerStateType,
 *  Can_ErrorStateType, Can_ReturnType, PduIdType) are obtained from
 *  Can_GeneralTypes.h.  XCAN-specific configuration types use the
 *  Mhal_Can_ prefix to avoid name conflicts with the MCAL's Can.h.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef MHAL_CAN_H
#define MHAL_CAN_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include "Std_Types.h"
#include "Can_GeneralTypes.h"

/******************************************************************************
 *  VERSION INFO
 *****************************************************************************/

#define MHAL_CAN_SW_MAJOR_VERSION (1U)
#define MHAL_CAN_SW_MINOR_VERSION (1U)
#define MHAL_CAN_SW_PATCH_VERSION (0U)

/******************************************************************************
 *  CAN ID MASKS
 *****************************************************************************/

#define STD_ID_MASK (0x000007FFUL)
#define STD_ID_DATA_MASK (0x00000000UL)

#define EXT_ID_MASK (0x1FFFFFFFUL)
#define EXT_ID_DATA_MASK (0x80000000UL)

#define ID_MASK(x) ((x) << 30UL)

#define FD_STD_MASK (ID_MASK(1UL) | STD_ID_DATA_MASK)
#define FD_EXT_MASK (ID_MASK(3UL) | EXT_ID_DATA_MASK)

#define IS_FD_FRAME(id) (((id)&ID_MASK(1UL)) != 0UL)
#define IS_EXT_ID(id) (((id)&EXT_ID_DATA_MASK) != 0UL)

/******************************************************************************
 *  HAL CONTROLLER COUNT (configurable, must match target system)
 *****************************************************************************/

#ifndef MHAL_CAN_CONTROLLER_CNT
#define MHAL_CAN_CONTROLLER_CNT 8u
#endif

/******************************************************************************
 *  XCAN-SPECIFIC TYPE DEFINITIONS (prefixed to avoid MCAL name conflicts)
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
/* CAN ID Filter Type (Standard / Extended / Mixed)                          */
/*---------------------------------------------------------------------------*/

typedef enum {
  MHAL_CAN_ID_STANDARD = 0U,
  MHAL_CAN_ID_EXTENDED,
  MHAL_CAN_ID_MIXED
} Mhal_Can_IdFilterType;

/*---------------------------------------------------------------------------*/
/* Object Type (TX/RX)                                                       */
/*---------------------------------------------------------------------------*/

typedef enum {
  MHAL_CAN_OBJECT_TYPE_RECEIVE = 0U,
  MHAL_CAN_OBJECT_TYPE_TRANSMIT
} Mhal_Can_ObjectType;

/*---------------------------------------------------------------------------*/
/* Payload Length Type (Classic vs FD)                                       */
/*---------------------------------------------------------------------------*/

typedef enum {
  MHAL_CAN_PL_8 = 0U,
  MHAL_CAN_PL_12,
  MHAL_CAN_PL_16,
  MHAL_CAN_PL_20,
  MHAL_CAN_PL_24,
  MHAL_CAN_PL_32,
  MHAL_CAN_PL_48,
  MHAL_CAN_PL_64
} Mhal_Can_ObjectPLType;

/*---------------------------------------------------------------------------*/
/* Bit Timing Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 BaudRateConfigId;
  uint8 PropSeg;
  uint8 PhaseSeg1;
  uint8 PhaseSeg2;
  uint8 SyncJumpWidth;
  uint16 Prescaler;
} Mhal_Can_BaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* FD Bit Timing Configuration                                               */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 CanControllerFdBaudRate;
  uint8 CanControllerPropSeg;
  uint8 CanControllerSeg1;
  uint8 CanControllerSeg2;
  uint8 CanControllerSyncJumpWidth;
  boolean CanControllerTxBitRateSwitch;
} Mhal_Can_FdBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* XL Bit Timing Configuration                                               */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 CanControllerXlPropSeg;
  uint8 CanControllerXlSeg1;
  uint8 CanControllerXlSeg2;
  uint8 CanControllerXlSyncJumpWidth;
  uint16 CanControllerXlTdcOffset;
} Mhal_Can_XlBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* PWME Configuration (XL pulse-width modulation)                            */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 Pwmo;
  uint16 Pwms;
  uint16 Pwml;
} Mhal_Can_PwmeCfgType;

/*---------------------------------------------------------------------------*/
/* Controller Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 ControllerId;
  uint32 BaseAddress;
  uint32 ClockFrequency;
  P2CONST(Mhal_Can_BaudrateCfgType, AUTOMATIC, CAN_CONST) BaudrateCfg;
  P2CONST(Mhal_Can_FdBaudrateCfgType, AUTOMATIC, CAN_CONST)
  CanControllerFdBaudrateConfig;
  P2CONST(Mhal_Can_XlBaudrateCfgType, AUTOMATIC, CAN_CONST)
  CanControllerXlBaudrateConfig;
  P2CONST(Mhal_Can_PwmeCfgType, AUTOMATIC, CAN_CONST) CanControllerPwmeConfig;
  uint8 BaudrateCfgCount;
  uint8 DefaultBaudrateIdx;
  boolean XlEnable;

  /* XCAN-specific hardware config */
  uint32 LmemBaseAddress;
  uint32 LmemSizeWords;
  uint8 RetransMax;
  boolean RxContinuousMode;
  uint32 IrcFuncEnaMask;
  uint32 IrcErrEnaMask;
  uint32 IrcSafetyEnaMask;
  uint32 LmemFqBase;
  uint32 LmemPqBase;
  uint32 LmemRxFilterBase;
  uint8 TxPqNumSlots;
  uint32 TxPqDescArrayAddr;
} Mhal_Can_ControllerCfgType;

/*---------------------------------------------------------------------------*/
/* Hardware Filter Type (for RX acceptance filtering)                         */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint32 CanHwFilterCode;
  uint32 CanHwFilterMask;
} Mhal_Can_HwFilterType;

/*---------------------------------------------------------------------------*/
/* Hardware Object Configuration                                             */
/*---------------------------------------------------------------------------*/

typedef struct {
  P2CONST(Mhal_Can_ControllerCfgType, AUTOMATIC, CAN_CONST) CanControllerRef;
  uint16 CanObjectId;
  Mhal_Can_ObjectType CanObjectType;
  Mhal_Can_IdFilterType CanIdType;
  uint8 HwFifoId;
  uint16 HwObjectCount;
  boolean PollingMode;
  Mhal_Can_ObjectPLType CanObjectPayloadLength;
  uint8 CanFdPaddingValue;

  /* XCAN FIFO memory configuration (provided by integrator) */
  uint32 DescArrayAddr;
  uint32 DcStartAddr;
  uint32 DcSizeWord;
  uint32 FifoSize;

  /* RX filter configuration (only used for RX objects) */
  uint8 HwFilterCount;
  P2CONST(Mhal_Can_HwFilterType, AUTOMATIC, CAN_CONST) CanHwFilter;
} Mhal_Can_HardwareObjectType;

/*---------------------------------------------------------------------------*/
/* Main Configuration Type                                                   */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 ControllerCfgNum;
  P2CONST(Mhal_Can_ControllerCfgType, AUTOMATIC, CAN_CONST) CanController;
  uint16 HwObjCfgNum;
  uint16 HwObjTxstartIdx;
  P2CONST(Mhal_Can_HardwareObjectType, AUTOMATIC, CAN_CONST) CanHardwareObject;
} Mhal_Can_ConfigType;

/******************************************************************************
 *  HAL FUNCTION PROTOTYPES - CLASSIC CAN
 *****************************************************************************/

/******************************************************************************
 *  Function    : can_hal_set_config
 *  Description : Set HAL configuration pointer. Must be called by MCAL
 *                before calling can_hal_init for any controller.
 *  Parameters  : Config - Pointer to configuration
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE)
can_hal_set_config(
    P2CONST(Mhal_Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) Config);

/******************************************************************************
 *  Function    : can_hal_init
 *  Description : Initialize specific CAN controller hardware.
 *                Config must be set via can_hal_set_config() first.
 *  Parameters  : cid - Controller ID (0 to MHAL_CAN_CONTROLLER_CNT-1)
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_init(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_deinit
 *  Description : De-initialize specific CAN controller.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_deinit(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_set_controller_mode
 *  Description : Set controller mode (start/stop).
 *                XCAN does not support sleep/wakeup — those transitions
 *                return E_NOT_OK.
 *  Parameters  : cid - Controller ID
 *                Transition - Target state
 *  Return      : E_OK on success, E_NOT_OK on failure
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            VAR(Can_StateTransitionType, AUTOMATIC) Transition);

/******************************************************************************
 *  Function    : can_hal_get_controller_mode
 *  Description : Get current controller mode.
 *  Parameters  : cid - Controller ID
 *                ControllerModePtr - Output for mode
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            P2VAR(Can_ControllerStateType, AUTOMATIC,
                                  CAN_APPL_DATA) ControllerModePtr);

/******************************************************************************
 *  Function    : can_hal_write
 *  Description : Transmit CAN message (Classic or FD).
 *  Parameters  : Hth - Hardware Transmit Handle
 *                PduInfo - Message data (id may have FD mask)
 *  Return      : E_OK if queued, E_NOT_OK on error, CAN_BUSY if full
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
              P2CONST(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo);

/******************************************************************************
 *  Function    : can_hal_read
 *  Description : Read received CAN message (Classic or FD).
 *  Parameters  : Hrh - Hardware Receive Handle
 *                PduInfo - Output for message
 *  Return      : E_OK if message read, E_NOT_OK if empty/error
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
             P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo);

/******************************************************************************
 *  Function    : can_hal_enable_controller_interrupts
 *  Description : Enable interrupts for controller.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE)
can_hal_enable_controller_interrupts(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_disable_controller_interrupts
 *  Description : Disable interrupts for controller.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE)
can_hal_disable_controller_interrupts(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_get_controller_error_state
 *  Description : Get controller error state.
 *  Parameters  : cid - Controller ID
 *                ErrorStatePtr - Output for error state
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_controller_error_state(VAR(uint8, AUTOMATIC) cid,
                                   P2VAR(Can_ErrorStateType, AUTOMATIC,
                                         CAN_APPL_DATA) ErrorStatePtr);

/******************************************************************************
 *  Function    : can_hal_get_rx_error_count
 *  Description : Get receive error counter value.
 *  Parameters  : cid - Controller ID
 *  Return      : Current REC value (uint8)
 *****************************************************************************/
FUNC(uint8, CAN_CODE)
can_hal_get_rx_error_count(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_get_tx_error_count
 *  Description : Get transmit error counter value.
 *  Parameters  : cid - Controller ID
 *  Return      : Current TEC value (uint8)
 *****************************************************************************/
FUNC(uint8, CAN_CODE)
can_hal_get_tx_error_count(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_set_baudrate
 *  Description : Set controller baud rate. Controller must be STOPPED.
 *  Parameters  : cid - Controller ID
 *                arb_baudrate - Arbitration baud rate config ID
 *  Return      : E_OK / E_NOT_OK
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_baudrate(VAR(uint8, AUTOMATIC) cid,
                     VAR(uint16, AUTOMATIC) arb_baudrate);

/******************************************************************************
 *  Function    : can_hal_main_function_read
 *  Description : Polling function for RX processing.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_read(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_main_function_write
 *  Description : Polling function for TX confirmation.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_write(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  Function    : can_hal_main_function_busoff
 *  Description : Polling function for bus-off handling.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_busoff(VAR(uint8, AUTOMATIC) cid);

/******************************************************************************
 *  HAL FUNCTION PROTOTYPES - CAN XL
 *****************************************************************************/

FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                P2CONST(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo);

FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
               P2VAR(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo);

/******************************************************************************
 *  INTERNAL / HELPER FUNCTIONS
 *****************************************************************************/

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hth_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsFd,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) Brs,
                        P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) PaddingValue);

FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hrh_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
                        P2VAR(Mhal_Can_ObjectPLType, AUTOMATIC, CAN_APPL_DATA)
                            MaxPayload);

/******************************************************************************
 *  INTERRUPT HANDLER
 *****************************************************************************/

FUNC(void, CAN_CODE) can_hal_irq_handler(VAR(uint8, AUTOMATIC) cid);

#endif /* MHAL_CAN_H */
