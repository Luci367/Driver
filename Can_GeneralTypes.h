/******************************************************************************
 *  File: Can_GeneralTypes.h
 *  Module: CAN General Types (LOCAL STUB for standalone compilation)
 *
 *  This is a LOCAL STUB that mimics the MCAL's Can_GeneralTypes.h.
 *  In the real MCAL build environment this file is NOT used — the actual
 *  platform-provided Can_GeneralTypes.h (which includes Can.h, Com.h,
 *  ComStack_Types.h, Platform_Types.h) takes precedence.
 *
 *  Purpose: allow the XCAN HAL/HCL/HDL code to compile and be syntax-checked
 *  on a host machine without the full MCAL header chain.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef CAN_GENERAL_TYPES_H
#define CAN_GENERAL_TYPES_H

#include "Std_Types.h"

/******************************************************************************
 *  CAN ID Type
 *****************************************************************************/

typedef uint32 Can_IdType;

/******************************************************************************
 *  Hardware Handle Type
 *****************************************************************************/

typedef uint16 Can_HwHandleType;

/******************************************************************************
 *  PduIdType (normally from ComStack_Types.h via Com.h)
 *****************************************************************************/

#ifndef PDUIDTYPE_DEFINED
#define PDUIDTYPE_DEFINED
typedef uint16 PduIdType;
#endif

/******************************************************************************
 *  State Transition Type (MCAL version includes SLEEP/WAKEUP)
 *****************************************************************************/

typedef enum {
  CAN_T_START = 0U,
  CAN_T_STOP,
  CAN_T_SLEEP,
  CAN_T_WAKEUP
} Can_StateTransitionType;

/******************************************************************************
 *  Controller State Type (MCAL version includes SLEEP)
 *****************************************************************************/

typedef enum {
  CAN_CS_UNINIT = 0U,
  CAN_CS_STARTED,
  CAN_CS_STOPPED,
  CAN_CS_SLEEP
} Can_ControllerStateType;

/******************************************************************************
 *  Error State Type
 *****************************************************************************/

typedef enum {
  CAN_ERRORSTATE_ACTIVE = 0U,
  CAN_ERRORSTATE_PASSIVE,
  CAN_ERRORSTATE_BUSOFF
} Can_ErrorStateType;

/******************************************************************************
 *  Return Type
 *****************************************************************************/

typedef enum { CAN_OK = 0U, CAN_NOT_OK, CAN_BUSY } Can_ReturnType;

/******************************************************************************
 *  CAN PDU Type (MCAL field order: id, swPduHandle, length, sdu)
 *****************************************************************************/

typedef struct {
  VAR(Can_IdType, CAN_VAR) id;
  VAR(PduIdType, CAN_VAR) swPduHandle;
  VAR(uint8, CAN_VAR) length;
  P2VAR(uint8, CAN_VAR, CAN_APPL_DATA) sdu;
} Can_PduType;

/******************************************************************************
 *  CAN XL Params (separate struct referenced by CanXL_PduType)
 *****************************************************************************/

typedef struct {
  uint16 PriorityId;
  uint16 Vcid;
  uint8 SduType;
  uint32 AcceptanceField;
  uint8 Sec;
} CanXL_Params;

/******************************************************************************
 *  CAN XL PDU Type (MCAL version — XL-specific fields via XLParams pointer)
 *****************************************************************************/

typedef struct {
  PduIdType swPduHandle;
  uint16 length;
  P2VAR(uint8, AUTOMATIC, CAN_APPL_DATA) sdu;
  P2VAR(CanXL_Params, AUTOMATIC, CAN_APPL_DATA) XLParams;
} CanXL_PduType;

/******************************************************************************
 *  Hardware Type (MCAL field order: CanId, Hoh, ControllerId)
 *****************************************************************************/

typedef struct {
  Can_IdType CanId;
  Can_HwHandleType Hoh;
  uint8 ControllerId;
} Can_HwType;

#endif /* CAN_GENERAL_TYPES_H */
