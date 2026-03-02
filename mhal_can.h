/******************************************************************************
 *  File: mhal_can.h
 *  Module: CAN MHAL (Hardware Abstraction Layer)
 *
 *  HAL layer API prototypes for CAN CC/FD/XL driver.
 *  Called by MCAL layer (Can.c / CanXL.c).
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef MHAL_CAN_H
#define MHAL_CAN_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include "Std_Types.h"

/******************************************************************************
 *  VERSION INFO
 *****************************************************************************/

#define MHAL_CAN_SW_MAJOR_VERSION (1U)
#define MHAL_CAN_SW_MINOR_VERSION (0U)
#define MHAL_CAN_SW_PATCH_VERSION (0U)

/******************************************************************************
 *  CAN ID MASKS
 *****************************************************************************/

/* Standard ID mask (11-bit) */
#define STD_ID_MASK (0x000007FFUL)
#define STD_ID_DATA_MASK (0x00000000UL)

/* Extended ID mask (29-bit) */
#define EXT_ID_MASK (0x1FFFFFFFUL)
#define EXT_ID_DATA_MASK (0x80000000UL)

/* ID type indicator mask */
#define ID_MASK(x) ((x) << 30UL)

/* FD frame ID masks - OR with CAN ID to indicate FD frame */
#define FD_STD_MASK (ID_MASK(1UL) | STD_ID_DATA_MASK)
#define FD_EXT_MASK (ID_MASK(3UL) | EXT_ID_DATA_MASK)

/* Check if ID indicates FD frame */
#define IS_FD_FRAME(id) (((id)&ID_MASK(1UL)) != 0UL)
#define IS_EXT_ID(id) (((id)&EXT_ID_DATA_MASK) != 0UL)

/******************************************************************************
 *  TYPE DEFINITIONS
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
/* Hardware Handle Type                                                      */
/*---------------------------------------------------------------------------*/

typedef uint16 Can_HwHandleType;

/*---------------------------------------------------------------------------*/
/* Controller ID Type                                                        */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_CTRL_0 = 0U,
  CAN_CTRL_1,
  CAN_CTRL_2,
  CAN_CTRL_3,
  CAN_CTRL_4,
  CAN_CTRL_5,
  CAN_CTRL_6,
  CAN_CTRL_7,
  CAN_CONTROLLER_CNT
} CanControllerIdType;

/*---------------------------------------------------------------------------*/
/* Controller State Type                                                     */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_CS_UNINIT = 0U,
  CAN_CS_STARTED,
  CAN_CS_STOPPED,
  CAN_CS_UNKNOWN
} Can_ControllerStateType;

/*---------------------------------------------------------------------------*/
/* State Transition Type                                                     */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_T_START = 0U,
  CAN_T_STOP
} Can_StateTransitionType;

/*---------------------------------------------------------------------------*/
/* Return Type                                                               */
/*---------------------------------------------------------------------------*/

typedef enum { CAN_OK = 0U, CAN_NOT_OK, CAN_BUSY } Can_ReturnType;

/*---------------------------------------------------------------------------*/
/* Error State Type                                                          */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_ERRORSTATE_ACTIVE = 0U,
  CAN_ERRORSTATE_PASSIVE,
  CAN_ERRORSTATE_BUSOFF
} Can_ErrorStateType;

/*---------------------------------------------------------------------------*/
/* CAN ID Type                                                               */
/*---------------------------------------------------------------------------*/

typedef enum { CAN_STANDARD = 0U, CAN_EXTENDED, CAN_MIXED } CanIdType;

/*---------------------------------------------------------------------------*/
/* Object Type (TX/RX)                                                       */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_OBJECT_TYPE_RECEIVE = 0U,
  CAN_OBJECT_TYPE_TRANSMIT
} Can_ObjectType;

/*---------------------------------------------------------------------------*/
/* Payload Length Type (Classic vs FD)                                       */
/*---------------------------------------------------------------------------*/

typedef enum {
  CAN_PL_8 = 0U, /* Classic CAN: 8 bytes max */
  CAN_PL_12,     /* FD: 12 bytes */
  CAN_PL_16,     /* FD: 16 bytes */
  CAN_PL_20,     /* FD: 20 bytes */
  CAN_PL_24,     /* FD: 24 bytes */
  CAN_PL_32,     /* FD: 32 bytes */
  CAN_PL_48,     /* FD: 48 bytes */
  CAN_PL_64      /* FD: 64 bytes */
} Can_ObjectPLType;

/*---------------------------------------------------------------------------*/
/* PDU Type (Classic CAN / CAN FD)                                           */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint32 id;                                  /* CAN ID */
  uint8 length;                               /* Data length */
  P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu; /* Data pointer */
  uint32 swPduHandle;                         /* Upper layer PDU handle */
} Can_PduType;

/*---------------------------------------------------------------------------*/
/* CAN XL PDU Type                                                           */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint32 priorityId;                          /* Priority ID (11-bit) */
  uint8 vcid;                                 /* Virtual CAN ID */
  uint8 sdt;                                  /* SDU Type */
  uint16 length;                              /* Data length (up to 2048) */
  P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu; /* Data pointer */
  uint32 swPduHandle;                         /* Upper layer PDU handle */
  boolean sec;                                /* Simple Extended Content */
} CanXL_PduType;

/*---------------------------------------------------------------------------*/
/* Hardware Type (for RX indication)                                         */
/*---------------------------------------------------------------------------*/

typedef struct {
  Can_HwHandleType Hoh; /* Hardware Object Handle */
  uint8 ControllerId;   /* Controller ID */
  uint32 CanId;         /* Received CAN ID */
} Can_HwType;

/*---------------------------------------------------------------------------*/
/* Bit Timing Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 BaudRateConfigId; /* Baud rate config ID */
  uint8 PropSeg;           /* Propagation segment */
  uint8 PhaseSeg1;         /* Phase segment 1 */
  uint8 PhaseSeg2;         /* Phase segment 2 */
  uint8 SyncJumpWidth;     /* Sync jump width */
  uint16 Prescaler;        /* Baud rate prescaler */
} Can_ControllerBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* FD Bit Timing Configuration                                               */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 CanControllerFdBaudRate;       /* FD Baud rate */
  uint8 CanControllerPropSeg;           /* Propagation segment */
  uint8 CanControllerSeg1;              /* Phase segment 1 */
  uint8 CanControllerSeg2;              /* Phase segment 2 */
  uint8 CanControllerSyncJumpWidth;     /* Sync jump width */
  boolean CanControllerTxBitRateSwitch; /* BRS enable */
} Can_ControllerFdBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* XL Bit Timing Configuration                                               */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 CanControllerXlPropSeg;       /* Propagation segment */
  uint8 CanControllerXlSeg1;          /* Phase segment 1 */
  uint8 CanControllerXlSeg2;          /* Phase segment 2 */
  uint8 CanControllerXlSyncJumpWidth; /* Sync jump width */
  uint16 CanControllerXlTdcOffset;    /* Transmitter Delay Compensation offset */
} Can_ControllerXlBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* PWME Configuration (XL pulse-width modulation)                            */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint16 Pwmo; /* PWME offset */
  uint16 Pwms; /* PWME sample */
  uint16 Pwml; /* PWME level */
} Can_ControllerPwmeCfgType;

/*---------------------------------------------------------------------------*/
/* Controller Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 ControllerId;    /* Controller index */
  uint32 BaseAddress;    /* XCAN instance register base address */
  uint32 ClockFrequency; /* CAN clock frequency */
  P2CONST(Can_ControllerBaudrateCfgType, AUTOMATIC, CAN_CONST) BaudrateCfg;
  P2CONST(Can_ControllerFdBaudrateCfgType, AUTOMATIC, CAN_CONST)
  CanControllerFdBaudrateConfig;
  P2CONST(Can_ControllerXlBaudrateCfgType, AUTOMATIC, CAN_CONST)
  CanControllerXlBaudrateConfig;
  P2CONST(Can_ControllerPwmeCfgType, AUTOMATIC, CAN_CONST) CanControllerPwmeConfig;
  uint8 BaudrateCfgCount;   /* Number of baud rate configs */
  uint8 DefaultBaudrateIdx; /* Default baud rate index */
  boolean XlEnable;         /* XL support enable */
  /* FD is enabled if CanControllerFdBaudrateConfig != NULL */

  /* XCAN-specific hardware config */
  uint32 LmemBaseAddress;    /* Local Memory base address */
  uint32 LmemSizeWords;      /* Local Memory size in 32-bit words */
  uint8 RetransMax;          /* Max retransmissions (0=fire-and-forget, 7=unlimited) */
  boolean RxContinuousMode;  /* RX FIFO continuous DC mode */
  uint32 IrcFuncEnaMask;     /* IRC functional interrupt enable mask */
  uint32 IrcErrEnaMask;      /* IRC error interrupt enable mask */
  uint32 IrcSafetyEnaMask;   /* IRC safety interrupt enable mask */
  uint32 LmemFqBase;         /* TX FIFO descriptor base offset in LMEM */
  uint32 LmemPqBase;         /* TX PQ descriptor base offset in LMEM */
  uint32 LmemRxFilterBase;   /* RX filter element base offset in LMEM */
  uint8 TxPqNumSlots;        /* Number of TX Priority Queue slots (0 = disabled) */
  uint32 TxPqDescArrayAddr;  /* System memory address for TX PQ descriptors */
} Can_ControllerType;

/*---------------------------------------------------------------------------*/
/* Hardware Object Configuration                                             */
/*---------------------------------------------------------------------------*/

typedef struct {
  P2CONST(Can_ControllerType, AUTOMATIC, CAN_CONST) CanControllerRef;
  uint16 CanObjectId;                      /* Hardware object ID */
  Can_ObjectType CanObjectType;            /* TX or RX */
  CanIdType CanIdType;                     /* Standard/Extended/Mixed */
  uint8 HwFifoId;                          /* FIFO queue ID (0-7) */
  uint16 HwObjectCount;                    /* Number of HW objects */
  boolean PollingMode;                     /* Polling or interrupt */
  Can_ObjectPLType CanObjectPayloadLength; /* Payload length (FD if >8) */
  uint8 CanFdPaddingValue;                 /* FD frame padding value */

  /* XCAN FIFO memory configuration (provided by integrator) */
  uint32 DescArrayAddr;  /* System memory address for descriptor array */
  uint32 DcStartAddr;    /* System memory address for data containers */
  uint32 DcSizeWord;     /* Data container size per message (in 32-bit words) */
  uint32 FifoSize;       /* Number of descriptors in this FIFO */
} Can_HardwareObjectType;

/*---------------------------------------------------------------------------*/
/* Main Configuration Type                                                   */
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8 ControllerCfgNum; /* Number of controllers */
  P2CONST(Can_ControllerType, AUTOMATIC, CAN_CONST) CanController;
  uint16 HwObjCfgNum;     /* Number of HW objects */
  uint16 HwObjTxstartIdx; /* First TX HW object index */
  P2CONST(Can_HardwareObjectType, AUTOMATIC, CAN_CONST) CanHardwareObject;
} Can_ConfigType;

/*---------------------------------------------------------------------------*/
/* Controller Status Type (HAL internal state)                               */
/*---------------------------------------------------------------------------*/

typedef struct {
  Can_ControllerStateType CtrlState; /* Current controller state */
  uint8 RefCounter;                  /* Reference counter */
  uint32 HthObjBusy;                 /* HTH busy flags (bitmask) */
} CanCtrlStatus;

/*---------------------------------------------------------------------------*/
/* HAL Configuration Count                                                   */
/*---------------------------------------------------------------------------*/

#define CAN_CTRL_CONFIG_CNT CAN_CONTROLLER_CNT

/*---------------------------------------------------------------------------*/
/* HAL State Access Macros                                                   */
/*---------------------------------------------------------------------------*/

/* Declared in mhal_can.c */
extern VAR(CanCtrlStatus, CAN_VAR) can_hd[CAN_CTRL_CONFIG_CNT];
extern P2CONST(Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) pCanHalCfg;

/* Get controller state data */
#define GET_CTRL_DATA(cid) (&can_hd[(cid)])

/* Get controller config */
#define GET_CTRL_CFG(cid) (&pCanHalCfg->CanController[(cid)])

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
can_hal_set_config(P2CONST(Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) Config);

/******************************************************************************
 *  Function    : can_hal_init
 *  Description : Initialize specific CAN controller hardware.
 *                Config must be set via can_hal_set_config() first.
 *  Parameters  : cid - Controller ID (0 to CAN_CONTROLLER_CNT-1)
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
 *  Parameters  : cid - Controller ID
 *                Transition - Target state
 *  Return      : E_OK on success, E_NOT_OK on failure
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            VAR(Can_ControllerStateType, AUTOMATIC) Transition);

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
 *                FD mode determined by HTH config:
 *                - CanObjectPayloadLength > CAN_PL_8 = FD frame
 *                - BRS from CanControllerTxBitRateSwitch
 *                - ID can have FD_STD_MASK/FD_EXT_MASK applied
 *  Parameters  : Hth - Hardware Transmit Handle
 *                PduInfo - Message data (id may have FD mask)
 *  Return      : E_OK if queued, E_NOT_OK on error, CAN_BUSY if full
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
              P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo);

/******************************************************************************
 *  Function    : can_hal_read
 *  Description : Read received CAN message (Classic or FD).
 *                FD indication in returned PduInfo.id via FD masks.
 *                Use IS_FD_FRAME(id) to check if FD.
 *  Parameters  : Hth - Hardware Receive Handle
 *                PduInfo - Output for message
 *  Return      : E_OK if message read, E_NOT_OK if empty/error
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hth,
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
 *                arb_baudrate - Arbitration baud rate value
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE)
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

/******************************************************************************
 *  Function    : canxl_hal_write
 *  Description : Transmit CAN XL message (up to 2048 bytes).
 *  Parameters  : Hth - Hardware Transmit Handle
 *                PduInfo - XL message data
 *  Return      : E_OK if queued, E_NOT_OK on error
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                P2CONST(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo);

/******************************************************************************
 *  Function    : canxl_hal_get_controller_mode
 *  Description : Get CAN XL controller mode.
 *  Parameters  : cid - Controller ID
 *                CtrlModePtr - Output for mode
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_get_controller_mode(VAR(uint8, AUTOMATIC) cid,
                              P2VAR(Can_ControllerStateType, AUTOMATIC,
                                    CANXL_APPL_CONST) CtrlModePtr);

/******************************************************************************
 *  Function    : canxl_hal_read
 *  Description : Read received CAN XL message.
 *  Parameters  : Hrh - Hardware Receive Handle
 *                PduInfo - Output for XL message
 *  Return      : E_OK if message read, E_NOT_OK if empty/error
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE)
canxl_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
               P2VAR(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo);

/******************************************************************************
 *  INTERNAL / HELPER FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 *  Function    : can_hal_get_hth_fd_info
 *  Description : Get FD configuration for HTH.
 *  Parameters  : Hth - Hardware Transmit Handle
 *                IsFd - Output: TRUE if FD configured
 *                Brs - Output: TRUE if BRS enabled
 *                PaddingValue - Output: FD padding byte
 *  Return      : E_OK if HTH valid
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hth_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hth,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsFd,
                        P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) Brs,
                        P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) PaddingValue);

/******************************************************************************
 *  Function    : can_hal_get_hrh_fd_info
 *  Description : Get FD configuration for HRH.
 *  Parameters  : Hrh - Hardware Receive Handle
 *                MaxPayload - Output: Max payload length
 *  Return      : E_OK if HRH valid
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE)
can_hal_get_hrh_fd_info(VAR(Can_HwHandleType, AUTOMATIC) Hrh,
                        P2VAR(Can_ObjectPLType, AUTOMATIC, CAN_APPL_DATA)
                            MaxPayload);

/******************************************************************************
 *  INTERRUPT HANDLER
 *****************************************************************************/

/******************************************************************************
 *  Function    : can_hal_irq_handler
 *  Description : CAN interrupt handler.
 *  Parameters  : cid - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_irq_handler(VAR(uint8, AUTOMATIC) cid);

#endif /* MHAL_CAN_H */
