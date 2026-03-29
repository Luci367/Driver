/******************************************************************************
 *  File: mhal_can_internal.h
 *  Module: CAN MHAL (Hardware Abstraction Layer) — Internal
 *
 *  HAL-private types, state variables, and access macros.
 *  Included ONLY by mhal_can.c — NOT by the MCAL or any external consumer.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef MHAL_CAN_INTERNAL_H
#define MHAL_CAN_INTERNAL_H

#include "mhal_can.h"

/******************************************************************************
 *  Controller Status Type (HAL internal state)
 *****************************************************************************/

typedef struct {
  Can_ControllerStateType CtrlState;
  uint8 RefCounter;
  uint32 HthObjBusy;
} CanCtrlStatus;

/******************************************************************************
 *  HAL Configuration Count
 *****************************************************************************/

#define CAN_CTRL_CONFIG_CNT MHAL_CAN_CONTROLLER_CNT

/******************************************************************************
 *  HAL State Variables (defined in mhal_can.c)
 *****************************************************************************/

extern VAR(CanCtrlStatus, CAN_VAR) can_hd[CAN_CTRL_CONFIG_CNT];
extern P2CONST(Mhal_Can_ConfigType, AUTOMATIC, CAN_APPL_CONST) pCanHalCfg;

/******************************************************************************
 *  HAL State Access Macros
 *****************************************************************************/

#define GET_CTRL_DATA(cid) (&can_hd[(cid)])
#define GET_CTRL_CFG(cid)  (&pCanHalCfg->CanController[(cid)])

#endif /* MHAL_CAN_INTERNAL_H */
