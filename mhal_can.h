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

#define MHAL_CAN_SW_MAJOR_VERSION       (1U)
#define MHAL_CAN_SW_MINOR_VERSION       (0U)
#define MHAL_CAN_SW_PATCH_VERSION       (0U)

/******************************************************************************
 *  CAN ID MASKS
 *****************************************************************************/

/* Standard ID mask (11-bit) */
#define STD_ID_MASK                 (0x000007FFUL)
#define STD_ID_DATA_MASK            (0x00000000UL)

/* Extended ID mask (29-bit) */
#define EXT_ID_MASK                 (0x1FFFFFFFUL)
#define EXT_ID_DATA_MASK            (0x80000000UL)

/* ID type indicator mask */
#define ID_MASK(x)                  ((x) << 30UL)

/* FD frame ID masks - OR with CAN ID to indicate FD frame */
#define FD_STD_MASK                 (ID_MASK(1UL) | STD_ID_DATA_MASK)
#define FD_EXT_MASK                 (ID_MASK(3UL) | EXT_ID_DATA_MASK)

/* Check if ID indicates FD frame */
#define IS_FD_FRAME(id)             (((id) & ID_MASK(1UL)) != 0UL)
#define IS_EXT_ID(id)               (((id) & EXT_ID_DATA_MASK) != 0UL)

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

typedef enum
{
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

typedef enum
{
    CAN_CS_UNINIT = 0U,
    CAN_CS_STARTED,
    CAN_CS_STOPPED,
    CAN_CS_SLEEP,
    CAN_CS_UNKNOWN
} Can_ControllerStateType;

/*---------------------------------------------------------------------------*/
/* State Transition Type                                                     */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_T_START = 0U,
    CAN_T_STOP,
    CAN_T_SLEEP,
    CAN_T_WAKEUP
} Can_StateTransitionType;

/*---------------------------------------------------------------------------*/
/* Return Type                                                               */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_OK = 0U,
    CAN_NOT_OK,
    CAN_BUSY
} Can_ReturnType;

/*---------------------------------------------------------------------------*/
/* Error State Type                                                          */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_ERRORSTATE_ACTIVE = 0U,
    CAN_ERRORSTATE_PASSIVE,
    CAN_ERRORSTATE_BUSOFF
} Can_ErrorStateType;

/*---------------------------------------------------------------------------*/
/* CAN ID Type                                                               */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_STANDARD = 0U,
    CAN_EXTENDED,
    CAN_MIXED
} CanIdType;

/*---------------------------------------------------------------------------*/
/* Object Type (TX/RX)                                                       */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_OBJECT_TYPE_RECEIVE = 0U,
    CAN_OBJECT_TYPE_TRANSMIT
} Can_ObjectType;

/*---------------------------------------------------------------------------*/
/* Payload Length Type (Classic vs FD)                                       */
/*---------------------------------------------------------------------------*/

typedef enum
{
    CAN_PL_8 = 0U,      /* Classic CAN: 8 bytes max */
    CAN_PL_12,          /* FD: 12 bytes */
    CAN_PL_16,          /* FD: 16 bytes */
    CAN_PL_20,          /* FD: 20 bytes */
    CAN_PL_24,          /* FD: 24 bytes */
    CAN_PL_32,          /* FD: 32 bytes */
    CAN_PL_48,          /* FD: 48 bytes */
    CAN_PL_64           /* FD: 64 bytes */
} Can_ObjectPLType;

/*---------------------------------------------------------------------------*/
/* PDU Type (Classic CAN / CAN FD)                                           */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32 id;                  /* CAN ID */
    uint8 length;               /* Data length */
    P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu;  /* Data pointer */
    uint32 swPduHandle;         /* Upper layer PDU handle */
} Can_PduType;

/*---------------------------------------------------------------------------*/
/* CAN XL PDU Type                                                           */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32 priorityId;          /* Priority ID (11-bit) */
    uint8 vcid;                 /* Virtual CAN ID */
    uint8 sdt;                  /* SDU Type */
    uint16 length;              /* Data length (up to 2048) */
    P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu;  /* Data pointer */
    uint32 swPduHandle;         /* Upper layer PDU handle */
    boolean sec;                /* Simple Extended Content */
} CanXL_PduType;

/*---------------------------------------------------------------------------*/
/* Hardware Type (for RX indication)                                         */
/*---------------------------------------------------------------------------*/

typedef struct
{
    Can_HwHandleType Hoh;       /* Hardware Object Handle */
    uint8 ControllerId;         /* Controller ID */
    uint32 CanId;               /* Received CAN ID */
} Can_HwType;

/*---------------------------------------------------------------------------*/
/* Bit Timing Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint16 BaudRateConfigId;    /* Baud rate config ID */
    uint8 PropSeg;              /* Propagation segment */
    uint8 PhaseSeg1;            /* Phase segment 1 */
    uint8 PhaseSeg2;            /* Phase segment 2 */
    uint8 SyncJumpWidth;        /* Sync jump width */
    uint16 Prescaler;           /* Baud rate prescaler */
} Can_ControllerBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* FD Bit Timing Configuration                                               */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint16 CanControllerFdBaudRate;     /* FD Baud rate */
    uint8 CanControllerPropSeg;         /* Propagation segment */
    uint8 CanControllerSeg1;            /* Phase segment 1 */
    uint8 CanControllerSeg2;            /* Phase segment 2 */
    uint8 CanControllerSyncJumpWidth;   /* Sync jump width */
    boolean CanControllerTxBitRateSwitch; /* BRS enable */
} Can_ControllerFdBaudrateCfgType;

/*---------------------------------------------------------------------------*/
/* Controller Configuration                                                  */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8 ControllerId;         /* Controller index */
    uint32 BaseAddress;         /* Register base address */
    uint32 ClockFrequency;      /* CAN clock frequency */
    P2CONST(Can_ControllerBaudrateCfgType, AUTOMATIC, CAN_CONST) BaudrateCfg;
    P2CONST(Can_ControllerFdBaudrateCfgType, AUTOMATIC, CAN_CONST) CanControllerFdBaudrateConfig;
    uint8 BaudrateCfgCount;     /* Number of baud rate configs */
    uint8 DefaultBaudrateIdx;   /* Default baud rate index */
    boolean XlEnable;           /* XL support enable */
    /* FD is enabled if CanControllerFdBaudrateConfig != NULL */
} Can_ControllerType;

/*---------------------------------------------------------------------------*/
/* Hardware Object Configuration                                             */
/*---------------------------------------------------------------------------*/

typedef struct
{
    P2CONST(Can_ControllerType, AUTOMATIC, CAN_CONST) CanControllerRef;
    uint16 CanObjectId;             /* Hardware object ID */
    Can_ObjectType CanObjectType;   /* TX or RX */
    CanIdType CanIdType;            /* Standard/Extended/Mixed */
    uint8 HwFifoId;                 /* FIFO queue ID (0-7) */
    uint16 HwObjectCount;           /* Number of HW objects */
    boolean PollingMode;            /* Polling or interrupt */
    Can_ObjectPLType CanObjectPayloadLength;  /* Payload length (FD if >8) */
    uint8 CanFdPaddingValue;        /* FD frame padding value */
} Can_HardwareObjectType;

/*---------------------------------------------------------------------------*/
/* Main Configuration Type                                                   */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8 ControllerCfgNum;     /* Number of controllers */
    P2CONST(Can_ControllerType, AUTOMATIC, CAN_CONST) CanController;
    uint16 HwObjCfgNum;         /* Number of HW objects */
    uint16 HwObjTxstartIdx;     /* First TX HW object index */
    P2CONST(Can_HardwareObjectType, AUTOMATIC, CAN_CONST) CanHardwareObject;
} Can_ConfigType;

/******************************************************************************
 *  HAL FUNCTION PROTOTYPES - CLASSIC CAN
 *****************************************************************************/

/******************************************************************************
 *  Function    : can_hal_init
 *  Description : Initialize CAN controller hardware.
 *  Parameters  : Config - Pointer to configuration
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_init
(
    P2CONST(Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) Config
);

/******************************************************************************
 *  Function    : can_hal_deinit
 *  Description : De-initialize CAN controller.
 *  Parameters  : void
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_deinit(void);

/******************************************************************************
 *  Function    : can_hal_set_controller_mode
 *  Description : Set controller mode (start/stop/sleep/wakeup).
 *  Parameters  : Controller - Controller ID
 *                Transition - Target state
 *  Return      : E_OK on success, E_NOT_OK on failure
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_set_controller_mode
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(Can_ControllerStateType, AUTOMATIC) Transition
);

/******************************************************************************
 *  Function    : can_hal_get_controller_mode
 *  Description : Get current controller mode.
 *  Parameters  : Controller - Controller ID
 *                ControllerModePtr - Output for mode
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_get_controller_mode
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_ControllerStateType, AUTOMATIC, CAN_APPL_DATA) ControllerModePtr
);

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
FUNC(Std_ReturnType, CAN_CODE) can_hal_write
(
    VAR(Can_HwHandleType, AUTOMATIC) Hth,
    P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo
);

/******************************************************************************
 *  Function    : can_hal_read
 *  Description : Read received CAN message (Classic or FD).
 *                FD indication in returned PduInfo.id via FD masks.
 *                Use IS_FD_FRAME(id) to check if FD.
 *  Parameters  : Hth - Hardware Receive Handle
 *                PduInfo - Output for message
 *  Return      : E_OK if message read, E_NOT_OK if empty/error
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_read
(
    VAR(Can_HwHandleType, AUTOMATIC) Hth,
    P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) PduInfo
);

/******************************************************************************
 *  Function    : can_hal_enable_controller_interrupts
 *  Description : Enable interrupts for controller.
 *  Parameters  : Controller - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_enable_controller_interrupts
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_disable_controller_interrupts
 *  Description : Disable interrupts for controller.
 *  Parameters  : Controller - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_disable_controller_interrupts
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_check_wakeup
 *  Description : Check for wakeup event.
 *  Parameters  : Controller - Controller ID
 *  Return      : E_OK if wakeup detected
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_check_wakeup
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_get_controller_error_state
 *  Description : Get controller error state.
 *  Parameters  : Controller - Controller ID
 *                ErrorStatePtr - Output for error state
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_get_controller_error_state
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_ErrorStateType, AUTOMATIC, CAN_APPL_DATA) ErrorStatePtr
);

/******************************************************************************
 *  Function    : can_hal_set_baudrate
 *  Description : Set controller baud rate.
 *  Parameters  : Controller - Controller ID
 *                Baudrate - Baud rate config index
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_set_baudrate
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint16, AUTOMATIC) Baudrate
);

/******************************************************************************
 *  Function    : can_hal_main_function_read
 *  Description : Polling function for RX processing.
 *  Parameters  : void
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_read(void);

/******************************************************************************
 *  Function    : can_hal_main_function_write
 *  Description : Polling function for TX confirmation.
 *  Parameters  : void
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_write(void);

/******************************************************************************
 *  Function    : can_hal_main_function_busoff
 *  Description : Polling function for bus-off handling.
 *  Parameters  : void
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_busoff(void);

/******************************************************************************
 *  Function    : can_hal_main_function_wakeup
 *  Description : Polling function for wakeup handling.
 *  Parameters  : void
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_wakeup(void);

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
FUNC(Std_ReturnType, CANXL_CODE) canxl_hal_write
(
    VAR(Can_HwHandleType, AUTOMATIC) Hth,
    P2CONST(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo
);

/******************************************************************************
 *  Function    : canxl_hal_get_controller_mode
 *  Description : Get CAN XL controller mode.
 *  Parameters  : CtrlIdx - Controller index
 *                CtrlModePtr - Output for mode
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE) canxl_hal_get_controller_mode
(
    VAR(uint8, AUTOMATIC) CtrlIdx,
    P2VAR(Can_ControllerStateType, AUTOMATIC, CANXL_APPL_CONST) CtrlModePtr
);

/******************************************************************************
 *  Function    : canxl_hal_transmit
 *  Description : Trigger XL frame transmission from buffer.
 *  Parameters  : CtrlIdx - Controller index
 *                BufIdx - Buffer index
 *                FrameType - Frame type
 *  Return      : E_OK on success
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE) canxl_hal_transmit
(
    VAR(uint8, AUTOMATIC) CtrlIdx,
    VAR(uint16, AUTOMATIC) BufIdx,
    VAR(uint16, AUTOMATIC) FrameType
);

/******************************************************************************
 *  Function    : canxl_hal_enable_egress_timestamp
 *  Description : Enable TX timestamp for buffer.
 *  Parameters  : CtrlIdx - Controller index
 *                BufIdx - Buffer index
 *  Return      : void
 *****************************************************************************/
FUNC(void, CANXL_CODE) canxl_hal_enable_egress_timestamp
(
    VAR(uint8, AUTOMATIC) CtrlIdx,
    VAR(uint16, AUTOMATIC) BufIdx
);

/******************************************************************************
 *  Function    : canxl_hal_read
 *  Description : Read received CAN XL message.
 *  Parameters  : CtrlIdx - Controller index
 *                PduInfo - Output for XL message
 *  Return      : E_OK if message read
 *****************************************************************************/
FUNC(Std_ReturnType, CANXL_CODE) canxl_hal_read
(
    VAR(uint8, AUTOMATIC) CtrlIdx,
    P2VAR(CanXL_PduType, AUTOMATIC, CANXL_APPL_DATA) PduInfo
);

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
FUNC(Std_ReturnType, CAN_CODE) can_hal_get_hth_fd_info
(
    VAR(Can_HwHandleType, AUTOMATIC) Hth,
    P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsFd,
    P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) Brs,
    P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) PaddingValue
);

/******************************************************************************
 *  Function    : can_hal_get_hrh_fd_info
 *  Description : Get FD configuration for HRH.
 *  Parameters  : Hrh - Hardware Receive Handle
 *                MaxPayload - Output: Max payload length
 *  Return      : E_OK if HRH valid
 *****************************************************************************/
FUNC(Std_ReturnType, CAN_CODE) can_hal_get_hrh_fd_info
(
    VAR(Can_HwHandleType, AUTOMATIC) Hrh,
    P2VAR(Can_ObjectPLType, AUTOMATIC, CAN_APPL_DATA) MaxPayload
);

/******************************************************************************
 *  INTERRUPT HANDLER
 *****************************************************************************/

/******************************************************************************
 *  Function    : can_hal_irq_handler
 *  Description : CAN interrupt handler.
 *  Parameters  : Controller - Controller ID
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_irq_handler
(
    VAR(uint8, AUTOMATIC) Controller
);

#endif /* MHAL_CAN_H */
