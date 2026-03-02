#ifndef CAN_CFG_H
#define CAN_CFG_H

#include <Std_Types.h>

/*==========================================================================
 *                      DEFINES AND MACROS
 *==========================================================================*/

#define CAN_DEV_ERROR_DETECT          STD_ON
#define CAN_SECURITY_EVENT_REPORTING  STD_OFF
#define CAN_GLOBAL_TIME_SUPPORT       STD_ON
#define CAN_INDEX                     1u
#define CAN_PDU_RECEIVE_CALLOUT_FUNC  CanLPduReceiveCalloutFunction_CanGeneral11
#define CAN_MAINF_BUSOFF_PERIOD       5u
#define CAN_MAINF_MODE_PERIOD         1u
#define CAN_MAINF_WAKEUP_PERIOD       1u
#define CAN_MULTI_TRANSMISSION        STD_OFF
#define CAN_SETBAUDRATE_API           STD_ON
#define CAN_TIMEOUT_DURATION          41u
#define CAN_VERSION_INF_API           STD_OFF
#define CAN_SUPPORT_TTCAN_REF         TRUE
#define CAN_FD_MODE_ENABLE            STD_ON
#define CAN_INTERRUPT_ENABLE          STD_ON
#define CAN_CTRL_CONFIG_CNT           8u
#define CAN_HW_OBJECT_CNT            4u
#define CAN_FILTER_CNT                1u
#define CAN_HTH_START_IDX             2u
#define RX_Q_SIZE                     3u
#define TX_Q_SIZE                     3u
#define CanMainFunctionRWPeriods_0    0.1f

typedef enum {
    CAN_RATE_125K,
    CAN_RATE_250K,
    CAN_RATE_500K,
    CAN_RATE_1M,
    CAN_RATE_2M,    /*FD only*/
    CAN_RATE_4M,    /*FD only*/
    CAN_RATE_5M,    /*FD only*/
    CAN_RATE_8M,    /*FD only*/
} can_bit_rate;

#endif // CAN_CFG_H
