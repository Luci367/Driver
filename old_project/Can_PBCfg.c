/* Copyright Statement:
 * licensors. Except as otherwise provided in the applicable licensing terms with MediaTek Inc.
 * and/or its licensors, any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */

#include <Can.h>
#include <Can_Cfg.h>
#include <Std_Types.h>
#include <stddef.h>

/*==========================================================================
 *                      GLOBAL VARIABLES
 *==========================================================================*/

CONST(Can_ControllerFdBaudrateCfgType, CAN_CONST) CanControllerFdBaudrateConfig = {2000u, 6u, 6u, 6u, 6u, TRUE};

CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_0 = {0u, 0u, 0u, 0u, 0u, &CanControllerFdBaudrateConfig};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_1 = {0u, 0u, 0u, 0u, 0u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_2 = {0u, 0u, 0u, 0u, 0u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_3 = {250u, 5u, 4u, 4u, 4u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_4 = {0u, 0u, 0u, 0u, 0u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_5 = {0u, 0u, 0u, 0u, 0u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_6 = {250u, 5u, 5u, 5u, 4u, NULL_PTR};
CONST(Can_ControllerBaudrateCfgType, CAN_CONST) CanControllerBaudrateConfig_7 = {0u, 0u, 0u, 0u, 0u, NULL_PTR};

CONST(Can_ControllerType, CAN_CONST) Can_PBController[8] = {
    {
        0u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_INTERRUPT,
        &CanControllerBaudrateConfig_0,
        CAN_PROCESSING_INTERRUPT,
        CAN_PROCESSING_INTERRUPT,
        CAN_WAKEUP_INTERRUPT
    },
    {
        1u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_INTERRUPT,
        &CanControllerBaudrateConfig_1,
        CAN_PROCESSING_INTERRUPT,
        CAN_PROCESSING_INTERRUPT,
        CAN_WAKEUP_INTERRUPT
    },
    {
        2u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_POLLING,
        &CanControllerBaudrateConfig_2,
        CAN_PROCESSING_INTERRUPT,
        CAN_PROCESSING_INTERRUPT,
        CAN_WAKEUP_POLLING
    },
    {
        3u,
        STD_ON,
        80000000,
        CAN_BUSOFF_INTERRUPT,
        &CanControllerBaudrateConfig_3,
        CAN_PROCESSING_MIXED,
        CAN_PROCESSING_MIXED,
        CAN_WAKEUP_INTERRUPT
    },
    {
        4u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_POLLING,
        &CanControllerBaudrateConfig_4,
        CAN_PROCESSING_MIXED,
        CAN_PROCESSING_MIXED,
        CAN_WAKEUP_INTERRUPT
    },
    {
        5u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_POLLING,
        &CanControllerBaudrateConfig_5,
        CAN_PROCESSING_INTERRUPT,
        CAN_PROCESSING_INTERRUPT,
        CAN_WAKEUP_INTERRUPT
    },
    {
        6u,
        STD_ON,
        80000000,
        CAN_BUSOFF_INTERRUPT,
        &CanControllerBaudrateConfig_6,
        CAN_PROCESSING_MIXED,
        CAN_PROCESSING_MIXED,
        CAN_WAKEUP_INTERRUPT
    },
    {
        7u,
        STD_OFF,
        80000000,
        CAN_BUSOFF_INTERRUPT,
        &CanControllerBaudrateConfig_7,
        CAN_PROCESSING_INTERRUPT,
        CAN_PROCESSING_INTERRUPT,
        CAN_WAKEUP_INTERRUPT
    }
};

CONST(Can_HwFilterType, CAN_CONST) CanHardwareObject_1_RX_CanHwFilter = {0xFFFFFFFFu, 0x00000000u};

CONST(Can_HardwareObjectType, CAN_CONST) Can_PBHwObj[4] = {
    {
        &Can_PBController[3],
        0u,
        CAN_EXTENDED,
        0u,
        CAN_OBJECT_TYPE_RECEIVE,
        CAN_HANDLE_TYPE_BASIC,
        STD_ON,
        1u,
        1u,
        &CanHardwareObject_1_RX_CanHwFilter,
        STD_ON,
        CAN_OBJECT_PL_12,
        CanMainFunctionRWPeriods_0
    },
    {
        &Can_PBController[6],
        2u,
        CAN_EXTENDED,
        0u,
        CAN_OBJECT_TYPE_RECEIVE,
        CAN_HANDLE_TYPE_BASIC,
        STD_OFF,
        1u,
        0u,
        NULL_PTR,
        STD_OFF,
        CAN_OBJECT_PL_12,
        0
    },
    {
        &Can_PBController[3],
        1u,
        CAN_EXTENDED,
        0u,
        CAN_OBJECT_TYPE_TRANSMIT,
        CAN_HANDLE_TYPE_BASIC,
        STD_OFF,
        1u,
        0u,
        NULL_PTR,
        STD_ON,
        CAN_OBJECT_PL_12,
        0
    },
    {
        &Can_PBController[6],
        3u,
        CAN_EXTENDED,
        0u,
        CAN_OBJECT_TYPE_TRANSMIT,
        CAN_HANDLE_TYPE_BASIC,
        STD_OFF,
        1u,
        0u,
        NULL_PTR,
        STD_ON,
        CAN_OBJECT_PL_12,
        0
    },
};

CONST(Can_ConfigType, CAN_DATA) Can_ConfigData = {
    CAN_CTRL_CONFIG_CNT,
    &Can_PBController[0],
    CAN_HW_OBJECT_CNT,
    CAN_HTH_START_IDX,
    &Can_PBHwObj[0]
};

/****************************
 * END OF FILE: Can_PBCfg.c
 ****************************/
