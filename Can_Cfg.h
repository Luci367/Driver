/******************************************************************************
 *  File: Can_Cfg.h
 *  Module: CAN Configuration Types
 *
 *  Configuration type definitions for CAN MHAL layer.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef CAN_CFG_H
#define CAN_CFG_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include "Std_Types.h"
#include "Can_Types.h"

/******************************************************************************
 *  CONFIGURATION MACROS
 *****************************************************************************/

/* Maximum number of CAN controllers */
#define CAN_HAL_MAX_CONTROLLERS     (7U)

/* Feature enables */
#define CAN_HAL_FD_ENABLE           STD_ON
#define CAN_HAL_XL_ENABLE           STD_ON
#define CAN_HAL_TX_FILTER_ENABLE    STD_ON
#define CAN_HAL_RX_FILTER_ENABLE    STD_ON
#define CAN_HAL_DET_ENABLE          STD_OFF
#define CAN_HAL_WAKEUP_ENABLE       STD_OFF
#define CAN_HAL_LOOPBACK_ENABLE     STD_ON

/* Queue configuration */
#define CAN_HAL_MAX_TX_QUEUE_SIZE   (1024U)
#define CAN_HAL_MAX_RX_QUEUE_SIZE   (1024U)
#define CAN_HAL_TX_FILTER_MAX       (16U)
#define CAN_HAL_RX_FILTER_MAX       (255U)

/* Timeout values */
#define CAN_HAL_INIT_TIMEOUT_US     (10000U)
#define CAN_HAL_POLL_TIMEOUT_COUNT  (100000U)

/******************************************************************************
 *  GLOBAL DATA TYPES AND STRUCTURES
 *****************************************************************************/

/*---------------------------------------------------------------------------*/
/* Queue Configuration Type                                                  */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              startAddr;      /* Descriptor start address */
    uint32              dcStartAddr;    /* Data Container start address */
    uint16              size;           /* Queue size (descriptor count) */
    uint16              dcSize;         /* Data Container size */
    boolean             enabled;        /* Queue enabled */
    boolean             continuous;     /* Continuous mode (RX only) */
} Can_Hal_QueueConfigType;

/*---------------------------------------------------------------------------*/
/* RX Filter Element Type                                                    */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              id;             /* Filter ID value */
    uint32              mask;           /* Filter mask */
    uint8               targetFifo;     /* Target FIFO (0-7) */
    boolean             extended;       /* Extended ID filter */
    boolean             enabled;        /* Filter enabled */
} Can_Hal_RxFilterType;

/*---------------------------------------------------------------------------*/
/* TX Filter Element Type                                                    */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              id;             /* Filter ID value */
    uint32              mask;           /* Filter mask */
    uint8               targetFifo;     /* Target TX FIFO (0-7) */
    boolean             enabled;        /* Filter enabled */
} Can_Hal_TxFilterType;

/*---------------------------------------------------------------------------*/
/* Interrupt Configuration Type                                              */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint32              funcEnableMask;     /* Functional interrupt mask */
    uint32              errEnableMask;      /* Error interrupt mask */
    uint32              safetyEnableMask;   /* Safety interrupt mask */
} Can_Hal_IrqConfigType;

/*---------------------------------------------------------------------------*/
/* Controller Configuration Type                                             */
/*---------------------------------------------------------------------------*/

typedef struct
{
    /* Base addresses */
    uint32              baseAddr;           /* Register base address */
    uint32              lmemBaseAddr;       /* Local memory base address */
    uint32              lmemSize;           /* Local memory size */

    /* Protocol and mode */
    Can_Hal_ProtocolType protocol;          /* CC/FD/XL */
    boolean             loopbackEnable;     /* Loopback mode */
    boolean             listenOnlyEnable;   /* Listen only mode */

    /* Bit timing */
    Can_Hal_BitTimingType nominalTiming;    /* Nominal bit timing */
    Can_Hal_BitTimingType dataTiming;       /* Data phase timing (FD) */
    Can_Hal_BitTimingType xlTiming;         /* XL phase timing */

    /* TX FIFO queues */
    Can_Hal_QueueConfigType txFifoQueues[CAN_HAL_TX_FIFO_COUNT];

    /* RX FIFO queues */
    Can_Hal_QueueConfigType rxFifoQueues[CAN_HAL_RX_FIFO_COUNT];

    /* TX Priority Queue */
    uint32              txPqStartAddr;      /* Priority queue start address */
    uint8               txPqSlotCount;      /* Number of PQ slots */

    /* Filters */
    uint32              rxFilterBaseAddr;   /* RX filter memory base */
    uint32              txDescBaseAddr;     /* TX descriptor memory base */
    uint8               rxFilterCount;      /* Number of RX filters */

    /* Interrupts */
    Can_Hal_IrqConfigType irqConfig;        /* Interrupt configuration */

    /* Safety features */
    boolean             txDescCrcEnable;    /* TX descriptor CRC check */
    boolean             rxDescCrcEnable;    /* RX descriptor CRC check */

    /* Instance identification */
    uint8               instanceNum;        /* Hardware instance number */

} Can_Hal_ControllerConfigType;

/*---------------------------------------------------------------------------*/
/* Main Configuration Type                                                   */
/*---------------------------------------------------------------------------*/

typedef struct
{
    /* Number of configured controllers */
    uint8               controllerCount;

    /* Controller configurations */
    P2CONST(Can_Hal_ControllerConfigType, AUTOMATIC, CAN_APPL_CONST)
                        controllerConfigs;

    /* RX filter configurations (shared or per-controller) */
    P2CONST(Can_Hal_RxFilterType, AUTOMATIC, CAN_APPL_CONST)
                        rxFilters;
    uint8               rxFilterCount;

    /* TX filter configurations */
    P2CONST(Can_Hal_TxFilterType, AUTOMATIC, CAN_APPL_CONST)
                        txFilters;
    uint8               txFilterCount;

} Can_Hal_ConfigType;

/*---------------------------------------------------------------------------*/
/* HTH to Controller Mapping Type                                            */
/*---------------------------------------------------------------------------*/

typedef struct
{
    uint8               controllerId;       /* Controller index */
    uint8               fifoId;             /* FIFO index */
    boolean             isPriorityQueue;    /* TRUE if PQ, FALSE if FIFO */
} Can_Hal_HthMappingType;

/******************************************************************************
 *  EXTERNAL CONFIGURATION DATA
 *****************************************************************************/

/* Configuration data to be provided by user/integrator */
extern CONST(Can_Hal_ConfigType, CAN_CONST) Can_Hal_Config;

#endif /* CAN_CFG_H */
