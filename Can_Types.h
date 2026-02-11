/******************************************************************************
 *  File: Can_Types.h
 *  Module: CAN MHAL Type Definitions
 *
 *  Type definitions for CAN MHAL layer.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef CAN_TYPES_H
#define CAN_TYPES_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include "Std_Types.h"

/******************************************************************************
 *  LOCAL MACROS
 *****************************************************************************/

/* Maximum data lengths */
#define CAN_HAL_CC_MAX_DLC          (8U)
#define CAN_HAL_FD_MAX_DLC          (64U)
#define CAN_HAL_XL_MAX_DLC          (2048U)

/* Queue counts */
#define CAN_HAL_TX_FIFO_COUNT       (8U)
#define CAN_HAL_RX_FIFO_COUNT       (8U)
#define CAN_HAL_TX_PQ_SLOT_COUNT    (32U)

/******************************************************************************
 *  GLOBAL DATA TYPES AND STRUCTURES
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
/* Return Type                                                               */
/*---------------------------------------------------------------------------*/

typedef uint8 Can_Hal_ReturnType;

#define CAN_HAL_E_OK            ((Can_Hal_ReturnType)0x00U)
#define CAN_HAL_E_NOT_OK        ((Can_Hal_ReturnType)0x01U)
#define CAN_HAL_E_PARAM         ((Can_Hal_ReturnType)0x02U)
#define CAN_HAL_E_TIMEOUT       ((Can_Hal_ReturnType)0x03U)
#define CAN_HAL_E_BUSY          ((Can_Hal_ReturnType)0x04U)
#define CAN_HAL_E_EMPTY         ((Can_Hal_ReturnType)0x05U)
#define CAN_HAL_E_FULL          ((Can_Hal_ReturnType)0x06U)
#define CAN_HAL_E_TRANSITION    ((Can_Hal_ReturnType)0x07U)
#define CAN_HAL_E_CRC           ((Can_Hal_ReturnType)0x08U)
#define CAN_HAL_E_DMA           ((Can_Hal_ReturnType)0x09U)

/*---------------------------------------------------------------------------*/
/* Hardware Handle Type                                                      */
/*---------------------------------------------------------------------------*/

typedef uint16 Can_Hal_HwHandleType;

/*---------------------------------------------------------------------------*/
/* CAN ID Type                                                               */
/*---------------------------------------------------------------------------*/

typedef uint32 Can_Hal_IdType;

/* ID type flags (OR with ID value) */
#define CAN_HAL_ID_EXTENDED     ((Can_Hal_IdType)0x80000000U)
#define CAN_HAL_ID_FD           ((Can_Hal_IdType)0x40000000U)
#define CAN_HAL_ID_XL           ((Can_Hal_IdType)0x20000000U)
#define CAN_HAL_ID_RTR          ((Can_Hal_IdType)0x10000000U)
#define CAN_HAL_ID_MASK         ((Can_Hal_IdType)0x1FFFFFFFU)

/*---------------------------------------------------------------------------*/
/* Controller Mode Types                                                     */
/*---------------------------------------------------------------------------*/

typedef uint8 Can_Hal_ControllerModeType;

#define CAN_HAL_CS_UNINIT       ((Can_Hal_ControllerModeType)0x00U)
#define CAN_HAL_CS_STOPPED      ((Can_Hal_ControllerModeType)0x01U)
#define CAN_HAL_CS_STARTED      ((Can_Hal_ControllerModeType)0x02U)
#define CAN_HAL_CS_SLEEP        ((Can_Hal_ControllerModeType)0x03U)

/*---------------------------------------------------------------------------*/
/* State Transition Types                                                    */
/*---------------------------------------------------------------------------*/

typedef uint8 Can_Hal_StateTransitionType;

#define CAN_HAL_T_START         ((Can_Hal_StateTransitionType)0x00U)
#define CAN_HAL_T_STOP          ((Can_Hal_StateTransitionType)0x01U)
#define CAN_HAL_T_SLEEP         ((Can_Hal_StateTransitionType)0x02U)
#define CAN_HAL_T_WAKEUP        ((Can_Hal_StateTransitionType)0x03U)

/*---------------------------------------------------------------------------*/
/* Bus State Type                                                            */
/*---------------------------------------------------------------------------*/

typedef uint8 Can_Hal_BusStateType;

#define CAN_HAL_BUS_ACTIVE      ((Can_Hal_BusStateType)0x00U)
#define CAN_HAL_BUS_WARNING     ((Can_Hal_BusStateType)0x01U)
#define CAN_HAL_BUS_PASSIVE     ((Can_Hal_BusStateType)0x02U)
#define CAN_HAL_BUS_OFF         ((Can_Hal_BusStateType)0x03U)

/*---------------------------------------------------------------------------*/
/* Protocol Type                                                             */
/*---------------------------------------------------------------------------*/

typedef uint8 Can_Hal_ProtocolType;

#define CAN_HAL_PROTOCOL_CC     ((Can_Hal_ProtocolType)0x00U)
#define CAN_HAL_PROTOCOL_FD     ((Can_Hal_ProtocolType)0x01U)
#define CAN_HAL_PROTOCOL_XL     ((Can_Hal_ProtocolType)0x02U)

/*---------------------------------------------------------------------------*/
/* PDU Type (CC/FD messages)                                                 */
/*---------------------------------------------------------------------------*/

typedef struct
{
    Can_Hal_IdType      id;         /* CAN ID (with type flags) */
    uint8               length;     /* Data length */
    P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu;  /* Pointer to data */
    uint32              swPduHandle; /* Upper layer handle */
    boolean             fd;         /* TRUE for FD frame */
    boolean             brs;        /* Bit rate switch (FD) */
} Can_Hal_PduType;

/*---------------------------------------------------------------------------*/
/* XL PDU Type (CAN XL messages)                                             */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              priorityId;  /* Priority ID (11-bit) */
    uint8               vcid;        /* Virtual CAN ID */
    uint8               sdt;         /* SDU Type */
    uint16              length;      /* Data length (up to 2048) */
    P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu;  /* Pointer to data */
    uint32              swPduHandle; /* Upper layer handle */
    boolean             sec;         /* Simple Extended Content */
} Can_Hal_XlPduType;

/*---------------------------------------------------------------------------*/
/* Received Message Type                                                     */
/*---------------------------------------------------------------------------*/

typedef struct
{
    Can_Hal_IdType      id;         /* CAN ID */
    uint8               data[CAN_HAL_XL_MAX_DLC]; /* Message data */
    uint16              length;     /* Data length */
    uint64              timestamp;  /* Reception timestamp */
    uint8               fifoId;     /* Source FIFO ID */
    boolean             extended;   /* Extended ID flag */
    boolean             fd;         /* FD frame flag */
    boolean             xl;         /* XL frame flag */
    boolean             brs;        /* Bit rate switch */
    boolean             esi;        /* Error state indicator */
    boolean             rtr;        /* Remote transmission request */
    uint8               vcid;       /* Virtual CAN ID (XL) */
    uint8               sdt;        /* SDU Type (XL) */
    uint8               status;     /* Reception status */
} Can_Hal_MsgType;

/*---------------------------------------------------------------------------*/
/* Bit Timing Type                                                           */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8               brp;        /* Baud rate prescaler */
    uint16              tseg1;      /* Time segment 1 */
    uint8               tseg2;      /* Time segment 2 */
    uint8               sjw;        /* Sync jump width */
    uint8               tdco;       /* Transmitter delay compensation */
} Can_Hal_BitTimingType;

/*---------------------------------------------------------------------------*/
/* Controller State Type (detailed)                                          */
/*---------------------------------------------------------------------------*/

typedef struct
{
    Can_Hal_ControllerModeType mode;    /* Current mode */
    Can_Hal_BusStateType       busState;/* Bus state */
    uint8                      txErrorCount;
    uint8                      rxErrorCount;
    boolean                    busOff;
    boolean                    errorPassive;
    boolean                    errorWarning;
} Can_Hal_ControllerStateType;

/*---------------------------------------------------------------------------*/
/* Error State Type                                                          */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8               txErrorCount;   /* TX error counter */
    uint8               rxErrorCount;   /* RX error counter */
    uint8               lastErrorCode;  /* Last error code */
    Can_Hal_BusStateType busState;      /* Current bus state */
} Can_Hal_ErrorStateType;

/*---------------------------------------------------------------------------*/
/* Statistics Type                                                           */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              txSuccessCount;
    uint32              rxSuccessCount;
    uint32              txErrorFrames;
    uint32              rxErrorFrames;
    uint32              arbLostCount;
    uint32              busOffCount;
    uint32              overrunCount;
    uint8               txErrorCount;
    uint8               rxErrorCount;
    Can_Hal_BusStateType busState;
    boolean             errorWarning;
    boolean             errorPassive;
    boolean             busOff;
    uint8               lastErrorCode;
} Can_Hal_StatsType;

/*---------------------------------------------------------------------------*/
/* Version Info Type                                                         */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint16              vendorID;
    uint16              moduleID;
    uint8               sw_major_version;
    uint8               sw_minor_version;
    uint8               sw_patch_version;
    uint32              hwVersion;      /* Hardware IP version */
} Can_Hal_VersionInfoType;

/*---------------------------------------------------------------------------*/
/* Callback Types                                                            */
/*---------------------------------------------------------------------------*/

typedef P2FUNC(void, CAN_APPL_CODE, Can_Hal_RxCallbackType)
(
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) UserCtx
);

typedef P2FUNC(void, CAN_APPL_CODE, Can_Hal_TxCallbackType)
(
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) UserCtx
);

typedef P2FUNC(void, CAN_APPL_CODE, Can_Hal_TxPqCallbackType)
(
    VAR(uint8, AUTOMATIC) SlotId,
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) UserCtx
);

typedef P2FUNC(void, CAN_APPL_CODE, Can_Hal_ErrorCallbackType)
(
    VAR(uint8, AUTOMATIC) ErrorCode,
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) UserCtx
);

typedef P2FUNC(void, CAN_APPL_CODE, Can_Hal_BusStateCallbackType)
(
    VAR(Can_Hal_BusStateType, AUTOMATIC) NewState,
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) UserCtx
);

/*---------------------------------------------------------------------------*/
/* Callback Configuration Type                                               */
/*---------------------------------------------------------------------------*/

typedef struct
{
    Can_Hal_RxCallbackType      rxCallbacks[CAN_HAL_RX_FIFO_COUNT];
    Can_Hal_TxCallbackType      txCallbacks[CAN_HAL_TX_FIFO_COUNT];
    Can_Hal_TxPqCallbackType    txPqCallback;
    Can_Hal_ErrorCallbackType   errorCallback;
    Can_Hal_BusStateCallbackType busStateCallback;
    P2VAR(void, AUTOMATIC, CAN_APPL_DATA) userCtx;
} Can_Hal_IrqCallbacksType;

#endif /* CAN_TYPES_H */
