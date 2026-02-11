/******************************************************************************
 *  File: can_mhal.h
 *  Module: CAN MHAL (Microcontroller Hardware Abstraction Layer)
 *
 *  CAN MHAL layer API prototypes for CAN CC/FD/XL driver.
 *  This layer is called by MCAL and internally calls MHCL layer.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef CAN_MHAL_H
#define CAN_MHAL_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include "Can_Types.h"
#include "Can_Cfg.h"
#include "Std_Types.h"

/******************************************************************************
 *  LOCAL MACROS
 *****************************************************************************/

/* Module identification */
#define CAN_MHAL_MODULE_ID              (80U)
#define CAN_MHAL_VENDOR_ID              (0U)
#define CAN_MHAL_INSTANCE_ID            (0U)

/* Module version */
#define CAN_MHAL_SW_MAJOR_VERSION       (1U)
#define CAN_MHAL_SW_MINOR_VERSION       (0U)
#define CAN_MHAL_SW_PATCH_VERSION       (0U)

/* Service IDs for error reporting */
#define CAN_HAL_SID_INIT                    (0x00U)
#define CAN_HAL_SID_DEINIT                  (0x01U)
#define CAN_HAL_SID_SET_CONTROLLER_MODE     (0x02U)
#define CAN_HAL_SID_GET_CONTROLLER_MODE     (0x03U)
#define CAN_HAL_SID_GET_CONTROLLER_STATE    (0x04U)
#define CAN_HAL_SID_WRITE                   (0x05U)
#define CAN_HAL_SID_WRITE_XL                (0x06U)
#define CAN_HAL_SID_WRITE_FIFO              (0x07U)
#define CAN_HAL_SID_WRITE_PRIORITY          (0x08U)
#define CAN_HAL_SID_ABORT_TX                (0x09U)
#define CAN_HAL_SID_READ                    (0x0AU)
#define CAN_HAL_SID_RX_FIFO_SETUP           (0x0BU)
#define CAN_HAL_SID_RX_FIFO_SETUP_CONT      (0x0CU)
#define CAN_HAL_SID_RX_HAS_MESSAGE          (0x0DU)
#define CAN_HAL_SID_RX_RESTART              (0x0EU)
#define CAN_HAL_SID_ABORT_RX                (0x0FU)
#define CAN_HAL_SID_ENABLE_IRQ              (0x10U)
#define CAN_HAL_SID_DISABLE_IRQ             (0x11U)
#define CAN_HAL_SID_IRQ_HANDLER             (0x12U)
#define CAN_HAL_SID_GET_IRQ_PENDING         (0x13U)
#define CAN_HAL_SID_SET_BAUDRATE            (0x14U)
#define CAN_HAL_SID_CHECK_BAUDRATE          (0x15U)
#define CAN_HAL_SID_GET_ERROR_STATE         (0x16U)
#define CAN_HAL_SID_GET_BUS_STATE           (0x17U)
#define CAN_HAL_SID_GET_STATS               (0x18U)
#define CAN_HAL_SID_CLEAR_STATS             (0x19U)
#define CAN_HAL_SID_SET_LOOPBACK            (0x1AU)
#define CAN_HAL_SID_SET_LISTEN_ONLY         (0x1BU)
#define CAN_HAL_SID_GET_VERSION             (0x1CU)
#define CAN_HAL_SID_SOFTWARE_RESET          (0x1DU)
#define CAN_HAL_SID_CHECK_WAKEUP            (0x1EU)
#define CAN_HAL_SID_GET_CURRENT_TIME        (0x1FU)
#define CAN_HAL_SID_REGISTER_CALLBACKS      (0x20U)
#define CAN_HAL_SID_MAIN_FUNCTION_READ      (0x21U)
#define CAN_HAL_SID_MAIN_FUNCTION_WRITE     (0x22U)
#define CAN_HAL_SID_MAIN_FUNCTION_BUSOFF    (0x23U)
#define CAN_HAL_SID_MAIN_FUNCTION_WAKEUP    (0x24U)

/******************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *****************************************************************************/

#define CAN_START_SEC_CODE
/* #include "Can_MemMap.h" */

/*===========================================================================*/
/*                      INITIALIZATION / DEINITIALIZATION                    */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_init
 *  Description : Initializes the CAN controller hardware.
 *                Performs reset, unlocks config registers, configures
 *                timeouts, queues, bit timing, and enables the MH.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID (0 to CAN_MAX_CONTROLLERS-1)
 *      ConfigPtr   - Pointer to configuration structure
 *
 *  Return      : CAN_HAL_E_OK on success, error code otherwise
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_init
(
    VAR(uint8, AUTOMATIC) Controller,
    P2CONST(Can_Hal_ConfigType, AUTOMATIC, CAN_APPL_CONST) ConfigPtr
);

/******************************************************************************
 *  Function    : can_hal_deinit
 *  Description : De-initializes the CAN controller.
 *                Stops the controller and resets hardware to default.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : CAN_HAL_E_OK on success, error code otherwise
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_deinit
(
    VAR(uint8, AUTOMATIC) Controller
);

/*===========================================================================*/
/*                      CONTROLLER MODE MANAGEMENT                           */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_set_controller_mode
 *  Description : Sets controller mode (STOPPED, STARTED, SLEEP).
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      Transition  - Requested state transition
 *
 *  Return      : CAN_HAL_E_OK if transition accepted
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_set_controller_mode
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(Can_Hal_StateTransitionType, AUTOMATIC) Transition
);

/******************************************************************************
 *  Function    : can_hal_get_controller_mode
 *  Description : Gets current controller operational mode.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      ControllerModePtr - Output pointer for mode
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_controller_mode
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_Hal_ControllerModeType, AUTOMATIC, CAN_APPL_DATA) ControllerModePtr
);

/******************************************************************************
 *  Function    : can_hal_get_controller_state
 *  Description : Gets detailed controller state including error counters.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      StatePtr    - Output pointer for state
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_controller_state
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_Hal_ControllerStateType, AUTOMATIC, CAN_APPL_DATA) StatePtr
);

/*===========================================================================*/
/*                      TRANSMISSION APIS                                    */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_write
 *  Description : Transmits a CAN CC/FD message using hardware transmit handle.
 *
 *  Parameters  :
 *      Hth     - Hardware Transmit Handle (maps to TX FIFO)
 *      PduInfo - Message information (ID, data, length)
 *
 *  Return      : CAN_HAL_E_OK if queued, CAN_HAL_E_BUSY if full
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_write
(
    VAR(Can_Hal_HwHandleType, AUTOMATIC) Hth,
    P2CONST(Can_Hal_PduType, AUTOMATIC, CAN_APPL_CONST) PduInfo
);

/******************************************************************************
 *  Function    : can_hal_write_xl
 *  Description : Transmits a CAN XL message (up to 2048 bytes).
 *                Handles SDT and VCID fields for XL frames.
 *
 *  Parameters  :
 *      Hth       - Hardware Transmit Handle
 *      PduInfoXl - CAN XL message information
 *
 *  Return      : CAN_HAL_E_OK if queued, CAN_HAL_E_BUSY if full
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_write_xl
(
    VAR(Can_Hal_HwHandleType, AUTOMATIC) Hth,
    P2CONST(Can_Hal_XlPduType, AUTOMATIC, CAN_APPL_CONST) PduInfoXl
);

/******************************************************************************
 *  Function    : can_hal_write_fifo
 *  Description : Transmits a CAN message via specific TX FIFO queue.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - TX FIFO queue ID (0-7)
 *      PduInfo     - Message information
 *
 *  Return      : CAN_HAL_E_OK if queued, CAN_HAL_E_BUSY if full
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_write_fifo
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2CONST(Can_Hal_PduType, AUTOMATIC, CAN_APPL_CONST) PduInfo
);

/******************************************************************************
 *  Function    : can_hal_write_priority
 *  Description : Transmits via TX Priority Queue slot.
 *                Messages sent based on CAN ID arbitration priority.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      SlotId      - Priority queue slot ID (0-31)
 *      PduInfo     - Message information
 *
 *  Return      : CAN_HAL_E_OK if written, CAN_HAL_E_BUSY if slot busy
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_write_priority
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) SlotId,
    P2CONST(Can_Hal_PduType, AUTOMATIC, CAN_APPL_CONST) PduInfo
);

/******************************************************************************
 *  Function    : can_hal_abort_tx
 *  Description : Aborts pending transmissions on specified TX FIFO.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - TX FIFO queue ID
 *
 *  Return      : CAN_HAL_E_OK if abort initiated
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_abort_tx
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId
);

/******************************************************************************
 *  Function    : can_hal_abort_tx_priority
 *  Description : Aborts pending transmission in Priority Queue slot.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      SlotId      - Priority queue slot ID
 *
 *  Return      : CAN_HAL_E_OK if abort initiated
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_abort_tx_priority
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) SlotId
);

/******************************************************************************
 *  Function    : can_hal_tx_is_busy
 *  Description : Checks if TX FIFO has pending messages.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - TX FIFO queue ID
 *      IsBusyPtr   - Output for busy status
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_tx_is_busy
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsBusyPtr
);

/*===========================================================================*/
/*                      RECEPTION APIS                                       */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_read
 *  Description : Reads a received CAN message from RX FIFO.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID (0-7)
 *      MsgPtr      - Output for received message
 *      TimeoutUs   - Timeout in microseconds (0 = no wait)
 *
 *  Return      : CAN_HAL_E_OK if read, CAN_HAL_E_EMPTY if no message
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_read
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(Can_Hal_MsgType, AUTOMATIC, CAN_APPL_DATA) MsgPtr,
    VAR(uint32, AUTOMATIC) TimeoutUs
);

/******************************************************************************
 *  Function    : can_hal_rx_fifo_setup
 *  Description : Configures an RX FIFO queue.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      FifoId          - RX FIFO queue ID (0-7)
 *      DescPhysAddr    - Physical address for descriptors
 *      MaxDesc         - Maximum descriptor count
 *      DcSize          - Data Container size
 *      Continuous      - TRUE for continuous mode
 *
 *  Return      : CAN_HAL_E_OK on success, CAN_HAL_E_BUSY if FIFO busy
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_fifo_setup
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    VAR(uint32, AUTOMATIC) DescPhysAddr,
    VAR(uint16, AUTOMATIC) MaxDesc,
    VAR(uint32, AUTOMATIC) DcSize,
    VAR(boolean, AUTOMATIC) Continuous
);

/******************************************************************************
 *  Function    : can_hal_rx_fifo_setup_continuous
 *  Description : Configures RX FIFO in Continuous Mode.
 *                DMA writes data sequentially in circular buffer.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      FifoId          - RX FIFO queue ID
 *      DescPhysAddr    - Physical address for descriptors
 *      MaxDesc         - Maximum descriptor count
 *      DcStartAddr     - Data Container start address
 *      DcSize          - Total buffer size
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_fifo_setup_continuous
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    VAR(uint32, AUTOMATIC) DescPhysAddr,
    VAR(uint16, AUTOMATIC) MaxDesc,
    VAR(uint32, AUTOMATIC) DcStartAddr,
    VAR(uint32, AUTOMATIC) DcSize
);

/******************************************************************************
 *  Function    : can_hal_rx_has_message
 *  Description : Checks if new message is available in RX FIFO.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID
 *      HasMsgPtr   - Output (TRUE if message available)
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_has_message
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) HasMsgPtr
);

/******************************************************************************
 *  Function    : can_hal_rx_restart
 *  Description : Restarts RX FIFO after overflow or error.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID
 *
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_rx_restart
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId
);

/******************************************************************************
 *  Function    : can_hal_abort_rx
 *  Description : Aborts reception on specified RX FIFO.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID
 *
 *  Return      : CAN_HAL_E_OK if abort initiated
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_abort_rx
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId
);

/******************************************************************************
 *  Function    : can_hal_rx_is_busy
 *  Description : Checks if RX FIFO is busy processing.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID
 *      IsBusyPtr   - Output for busy status
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_is_busy
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(boolean, AUTOMATIC, CAN_APPL_DATA) IsBusyPtr
);

/******************************************************************************
 *  Function    : can_hal_rx_get_fill_level
 *  Description : Gets current message count in RX FIFO.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      FifoId          - RX FIFO queue ID
 *      FillLevelPtr    - Output for fill level
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_get_fill_level
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    P2VAR(uint32, AUTOMATIC, CAN_APPL_DATA) FillLevelPtr
);

/******************************************************************************
 *  Function    : can_hal_rx_update_read_ptr
 *  Description : Updates RX FIFO read pointer (for continuous mode).
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FifoId      - RX FIFO queue ID
 *      NewAddr     - New read pointer address
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_rx_update_read_ptr
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint8, AUTOMATIC) FifoId,
    VAR(uint32, AUTOMATIC) NewAddr
);

/*===========================================================================*/
/*                      INTERRUPT MANAGEMENT                                 */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_enable_controller_interrupts
 *  Description : Enables specified interrupt sources.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FuncMask    - Functional interrupt enable mask
 *      ErrMask     - Error interrupt enable mask
 *      SafetyMask  - Safety interrupt enable mask
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_enable_controller_interrupts
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint32, AUTOMATIC) FuncMask,
    VAR(uint32, AUTOMATIC) ErrMask,
    VAR(uint32, AUTOMATIC) SafetyMask
);

/******************************************************************************
 *  Function    : can_hal_disable_controller_interrupts
 *  Description : Disables all interrupts for the controller.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_disable_controller_interrupts
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_irq_handler
 *  Description : Main interrupt handler. Reads status, clears flags,
 *                invokes registered callbacks.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : void
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_irq_handler
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_get_irq_pending
 *  Description : Gets pending interrupt status.
 *
 *  Parameters  :
 *      Controller       - CAN controller ID
 *      FuncPendingPtr   - Output for functional status (may be NULL)
 *      ErrPendingPtr    - Output for error status (may be NULL)
 *      SafetyPendingPtr - Output for safety status (may be NULL)
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_irq_pending
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(uint32, AUTOMATIC, CAN_APPL_DATA) FuncPendingPtr,
    P2VAR(uint32, AUTOMATIC, CAN_APPL_DATA) ErrPendingPtr,
    P2VAR(uint32, AUTOMATIC, CAN_APPL_DATA) SafetyPendingPtr
);

/******************************************************************************
 *  Function    : can_hal_clear_irq
 *  Description : Clears specified interrupt flags.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      FuncMask    - Functional interrupts to clear
 *      ErrMask     - Error interrupts to clear
 *      SafetyMask  - Safety interrupts to clear
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_clear_irq
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint32, AUTOMATIC) FuncMask,
    VAR(uint32, AUTOMATIC) ErrMask,
    VAR(uint32, AUTOMATIC) SafetyMask
);

/******************************************************************************
 *  Function    : can_hal_register_callbacks
 *  Description : Registers callback functions for TX, RX, and error events.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      CallbacksPtr    - Pointer to callback structure
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_register_callbacks
(
    VAR(uint8, AUTOMATIC) Controller,
    P2CONST(Can_Hal_IrqCallbacksType, AUTOMATIC, CAN_APPL_CONST) CallbacksPtr
);

/*===========================================================================*/
/*                      BAUD RATE MANAGEMENT                                 */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_set_baudrate
 *  Description : Sets bit timing for nominal, data (FD), and XL phases.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      NominalPtr  - Nominal phase timing
 *      DataPtr     - Data phase timing (FD), may be NULL
 *      XlPtr       - XL phase timing, may be NULL
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_set_baudrate
(
    VAR(uint8, AUTOMATIC) Controller,
    P2CONST(Can_Hal_BitTimingType, AUTOMATIC, CAN_APPL_CONST) NominalPtr,
    P2CONST(Can_Hal_BitTimingType, AUTOMATIC, CAN_APPL_CONST) DataPtr,
    P2CONST(Can_Hal_BitTimingType, AUTOMATIC, CAN_APPL_CONST) XlPtr
);

/******************************************************************************
 *  Function    : can_hal_check_baudrate
 *  Description : Validates if baud rate is achievable with current clock.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      BaudRate    - Desired baud rate in bps
 *
 *  Return      : CAN_HAL_E_OK if valid, CAN_HAL_E_NOT_OK otherwise
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_check_baudrate
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(uint32, AUTOMATIC) BaudRate
);

/*===========================================================================*/
/*                      ERROR AND STATUS                                     */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_get_controller_error_state
 *  Description : Gets error state including error counters.
 *
 *  Parameters  :
 *      Controller      - CAN controller ID
 *      ErrorStatePtr   - Output for error state
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_controller_error_state
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_Hal_ErrorStateType, AUTOMATIC, CAN_APPL_DATA) ErrorStatePtr
);

/******************************************************************************
 *  Function    : can_hal_get_bus_state
 *  Description : Gets CAN bus state (Active, Warning, Passive, Bus-Off).
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      StatePtr    - Output for bus state
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_bus_state
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_Hal_BusStateType, AUTOMATIC, CAN_APPL_DATA) StatePtr
);

/******************************************************************************
 *  Function    : can_hal_get_stats
 *  Description : Gets TX/RX statistics and error counters.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      StatsPtr    - Output for statistics
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_stats
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(Can_Hal_StatsType, AUTOMATIC, CAN_APPL_DATA) StatsPtr
);

/******************************************************************************
 *  Function    : can_hal_clear_stats
 *  Description : Clears statistics counters.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_clear_stats
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_get_current_time
 *  Description : Gets current timestamp from CAN controller timer.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      TimePtr     - Output for timestamp
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_current_time
(
    VAR(uint8, AUTOMATIC) Controller,
    P2VAR(uint64, AUTOMATIC, CAN_APPL_DATA) TimePtr
);

/*===========================================================================*/
/*                      UTILITY FUNCTIONS                                    */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_set_loopback
 *  Description : Enables/disables internal loopback mode.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      Enable      - TRUE to enable, FALSE to disable
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_set_loopback
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(boolean, AUTOMATIC) Enable
);

/******************************************************************************
 *  Function    : can_hal_set_listen_only
 *  Description : Enables/disables listen-only (bus monitoring) mode.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *      Enable      - TRUE to enable, FALSE to disable
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_set_listen_only
(
    VAR(uint8, AUTOMATIC) Controller,
    VAR(boolean, AUTOMATIC) Enable
);

/******************************************************************************
 *  Function    : can_hal_get_version_info
 *  Description : Gets driver and hardware version information.
 *
 *  Parameters  :
 *      VersionInfoPtr  - Output for version info
 *
 *  Return      : CAN_HAL_E_OK on success
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_get_version_info
(
    P2VAR(Can_Hal_VersionInfoType, AUTOMATIC, CAN_APPL_DATA) VersionInfoPtr
);

/******************************************************************************
 *  Function    : can_hal_software_reset
 *  Description : Performs software reset of CAN controller.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : CAN_HAL_E_OK on success, CAN_HAL_E_TIMEOUT if reset fails
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_software_reset
(
    VAR(uint8, AUTOMATIC) Controller
);

/******************************************************************************
 *  Function    : can_hal_check_wakeup
 *  Description : Checks if wakeup event occurred on CAN bus.
 *
 *  Parameters  :
 *      Controller  - CAN controller ID
 *
 *  Return      : CAN_HAL_E_OK if wakeup detected, CAN_HAL_E_NOT_OK otherwise
 *****************************************************************************/
FUNC(Can_Hal_ReturnType, CAN_CODE) can_hal_check_wakeup
(
    VAR(uint8, AUTOMATIC) Controller
);

/*===========================================================================*/
/*                      MAIN FUNCTIONS (POLLING MODE)                        */
/*===========================================================================*/

/******************************************************************************
 *  Function    : can_hal_main_function_read
 *  Description : Scheduled function for processing RX messages in polling.
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_read(void);

/******************************************************************************
 *  Function    : can_hal_main_function_write
 *  Description : Scheduled function for processing TX confirmations.
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_write(void);

/******************************************************************************
 *  Function    : can_hal_main_function_busoff
 *  Description : Scheduled function for Bus-Off recovery handling.
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_busoff(void);

/******************************************************************************
 *  Function    : can_hal_main_function_wakeup
 *  Description : Scheduled function for wakeup event handling.
 *****************************************************************************/
FUNC(void, CAN_CODE) can_hal_main_function_wakeup(void);

#define CAN_STOP_SEC_CODE
/* #include "Can_MemMap.h" */

#endif /* CAN_MHAL_H */
