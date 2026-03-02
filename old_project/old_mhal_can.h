/* Old project mhal_can.h - extracted from screenshots */

/*
 * CanCtrlStatus - Records the status of a controller during run time
 */
typedef struct {
    /* To record the controller state machine */
    VAR(Can_ControllerStateType, CAN_VAR) CtrlState;
    /*
     * [SWS_Can_00202]
     * store for Can_DisableControllerInterrupt nesting level
     */
    VAR(sint8, CAN_VAR) RefCounter;
    VAR(boolean, CAN_VAR) IrqEn;

    /* each bit indicate the specific HTH */
    VAR(uint32, CAN_VAR) HthObjBusy;
    VAR(uint32, CAN_VAR) ArbBaudrate;
    #if (CAN_FD_MODE_ENABLE == STD_ON)
    VAR(uint32, CAN_VAR) DataBaudrate; /* FD flexbitrate */
    #endif
    /* pointer points to static config set */
    P2CONST(Can_ConfigType, CAN_VAR, CAN_APPL_CONST) CanCFGSet;
    P2VAR(struct queue_node, CAN_VAR, CAN_APPL_DATA) RxCurrent;
    P2VAR(struct queue_node, CAN_VAR, CAN_APPL_DATA) TxCurrent;
    VAR(struct queue, CAN_VAR) RxQ;
    VAR(struct queue, CAN_VAR) TxQ;
} CanCtrlStatus;

/*==========================================================================
 *                  GLOBAL VARIABLE DECLARATIONS
 *==========================================================================*/

#define CAN_START_SEC_CONFIG_DATA_UNSPECIFIED
//#include <Can_MemMap.h>

extern VAR(CanCtrlStatus, CAN_VAR) can_hd[];

#define CAN_STOP_SEC_CONFIG_DATA_UNSPECIFIED
//#include <Can_MemMap.h>

/*==========================================================================
 *                  FUNCTION PROTOTYPES
 *==========================================================================*/

#define CAN_START_SEC_CODE
//#include <Can_MemMap.h>

FUNC(void, CAN_CODE) can_hal_init(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE) can_hal_deinit(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE) can_hal_set_baudrate(VAR(uint8, AUTOMATIC) cid,
                                           VAR(uint16, AUTOMATIC) arb_baudrate);

FUNC(Std_ReturnType, CAN_CODE)
can_hal_set_controller_mode(VAR(uint8, AUTOMATIC) cid,
                            VAR(Can_ControllerStateType, AUTOMATIC) Transition);

FUNC(void, CAN_CODE)
can_hal_disable_controller_interrupt(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE)
can_hal_enable_controller_interrupt(VAR(uint8, AUTOMATIC) cid);

FUNC(Can_ErrorStateType, CAN_CODE)
can_hal_get_error_state(VAR(uint8, AUTOMATIC) cid);

FUNC(uint8, CAN_CODE)
can_hal_get_rx_error_count(VAR(uint8, AUTOMATIC) cid);

FUNC(uint8, CAN_CODE)
can_hal_get_tx_error_count(VAR(uint8, AUTOMATIC) cid);

FUNC(Std_ReturnType, CAN_CODE)
can_hal_write(VAR(Can_HwHandleType, AUTOMATIC) Hth,
              P2CONST(Can_PduType, AUTOMATIC, CAN_APPL_DATA) Pdu);

FUNC(Std_ReturnType, CAN_CODE)
can_hal_read(VAR(Can_HwHandleType, AUTOMATIC) Hth,
             P2VAR(Can_PduType, AUTOMATIC, CAN_APPL_DATA) Pdu);

FUNC(void, CAN_CODE) can_hal_mainfunction_write(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE) can_hal_mainfunction_read(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE) can_hal_mainfunction_busoff(VAR(uint8, AUTOMATIC) cid);

FUNC(void, CAN_CODE) can_hal_interruupt_handler(VAR(uint8, AUTOMATIC) cid);
