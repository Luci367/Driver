/*
 * XCAN Hardware Common-interface Layer (HCL)
 *
 * Intermediate layer implementing one logical operation per function using
 * multiple HDL calls. Manages FIFO queue state (rolling counters, put/get
 * indices, descriptor pointers), TX descriptor building, RX descriptor
 * parsing, multi-step init/stop sequences, and interrupt processing.
 *
 * Reference:
 *   - xcand.c / xcand_mh.c / xcan_prt.c  (xcan_sw_example, Bosch)
 *   - XCAN User Manual v3.90
 *
 * DO NOT use any existing project files (mhal_can.h, can_driver.c, etc.).
 */

#ifndef MHCL_CAN_H
#define MHCL_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mhdl_can.h"

/* =========================================================================
 * Constants
 * ========================================================================= */
#define MHCL_CAN_MAX_TX_FIFO          MHDL_MH_MAX_TX_FIFO_NUMBER   /* 8 */
#define MHCL_CAN_MAX_RX_FIFO          MHDL_MH_MAX_RX_FIFO_NUMBER   /* 8 */
#define MHCL_CAN_MAX_TX_PQ_SLOTS      MHDL_MH_MAX_TX_PQ_SLOTS      /* 32 */

#define MHCL_CAN_MAX_XL_PAYLOAD_WORD  512u
#define MHCL_CAN_MAX_XL_PAYLOAD_BYTE  2048u

#define MHCL_CAN_CC_MAX_DATA_BYTES    8u
#define MHCL_CAN_FD_MAX_DLC           15u

#define MHCL_CAN_TX_DESC_SIZE_WORD    8u
#define MHCL_CAN_TX_DESC_SIZE_BYTE    (MHCL_CAN_TX_DESC_SIZE_WORD * 4u)
#define MHCL_CAN_RX_DESC_SIZE_WORD    4u
#define MHCL_CAN_RX_DESC_SIZE_BYTE    (MHCL_CAN_RX_DESC_SIZE_WORD * 4u)

#define MHCL_CAN_WORD_TO_BYTE(w)      ((w) << 2u)
#define MHCL_CAN_BYTE_TO_WORD(b)      ((b) >> 2u)

/* Rolling counter mask: 5-bit field (bits 8:4 of DESC_ERR_INFO1.RC) */
#define MHCL_CAN_RC_MASK  ((XCAND_MH_CREG_DESC_ERR_INFO1_RC_MASK >> XCAND_MH_CREG_DESC_ERR_INFO1_RC_SHIFT))

/* Retransmission settings */
#define MHCL_CAN_RETRANS_UNLIMITED     7u
#define MHCL_CAN_RETRANS_FIRE_FORGET   0u

/* =========================================================================
 * Enumerations
 * ========================================================================= */

typedef enum {
    MHCL_CAN_FF_CC = 0,
    MHCL_CAN_FF_FD = 1,
    MHCL_CAN_FF_XL = 2
} Mhcl_Can_FrameFormatType;

typedef enum {
    MHCL_CAN_ID_BASE     = 0,
    MHCL_CAN_ID_EXTENDED = 1
} Mhcl_Can_IdType;

typedef enum {
    MHCL_CAN_DIR_TX = 0,
    MHCL_CAN_DIR_RX = 1
} Mhcl_Can_DirectionType;

typedef enum {
    MHCL_CAN_ERR_ACTIVE  = 0,
    MHCL_CAN_ERR_PASSIVE = 1,
    MHCL_CAN_ERR_BUSOFF  = 2
} Mhcl_Can_ErrorStateType;

typedef enum {
    MHCL_CAN_OK  = 0,
    MHCL_CAN_NOT_OK = 1,
    MHCL_CAN_BUSY = 2
} Mhcl_Can_ReturnType;

/* =========================================================================
 * Unified internal message struct
 * Based on xcand_msg_struct from xcan_sw_example
 * ========================================================================= */
typedef struct {
    Mhcl_Can_DirectionType   direction;
    Mhcl_Can_FrameFormatType frame_format;
    Mhcl_Can_IdType          id_type;
    uint32_t                 frame_id;
    uint32_t                 dlc;

    /* CC/FD specific */
    uint32_t  brs   : 1;
    uint32_t  esi   : 1;
    uint32_t  rtr   : 1;

    /* XL specific */
    uint32_t  rrs   : 1;
    uint32_t  sec   : 1;
    uint32_t  vcid  : 8;
    uint32_t  sdt   : 8;
    uint32_t  fir   : 1;

    uint32_t  af;              /* Acceptance Field (XL only) */

    uint32_t  data_word[MHCL_CAN_MAX_XL_PAYLOAD_WORD];

    /* RX-only metadata */
    uint32_t  rx_fifo_number;
    uint32_t  rx_timestamp_lsw;
    uint32_t  rx_timestamp_msw;
    uint32_t  rx_status;
    uint8_t   rx_filter_fidx;
    uint8_t   rx_filter_fm  : 1;
    uint8_t   rx_filter_blk : 1;
    uint8_t   rx_filter_fab : 1;
} Mhcl_Can_MsgType;

/* =========================================================================
 * Per-FIFO / Per-Slot runtime state
 * ========================================================================= */
typedef struct {
    bool     enabled;
    uint32_t fifo_size;
    uint32_t dc_size_word;
    uint32_t desc_array_addr;     /* system memory address of descriptor array */
    uint32_t dc_start_addr;       /* system memory address of data containers */
    uint32_t put_index;
    uint32_t rolling_counter;
} Mhcl_Can_TxFifoState;

typedef struct {
    bool     enabled;
    uint32_t fifo_size;
    uint32_t dc_size_word;
    uint32_t desc_array_addr;
    uint32_t dc_start_addr;
    uint32_t get_index;
    uint32_t rolling_counter;
} Mhcl_Can_RxFifoState;

typedef struct {
    bool     enabled;
    uint32_t dc_size_word;
    uint32_t dc_start_addr;
} Mhcl_Can_TxPqSlotState;

/* =========================================================================
 * Bit timing config struct (single phase)
 * Based on xcan_prt_bt_single_struct
 * ========================================================================= */
typedef struct {
    uint16_t prop_seg;
    uint16_t phase_seg1;
    uint16_t phase_seg2;
    uint16_t sjw;
    uint16_t tdc_offset;
} Mhcl_Can_BitTimingType;

/* PWME config */
typedef struct {
    uint16_t pwmo;
    uint16_t pwms;
    uint16_t pwml;
} Mhcl_Can_PwmeType;

/* PRT mode flags */
typedef struct {
    bool fd_ena;
    bool xl_ena;
    bool tdc_ena;
    bool xl_tc_mode_switching_ena;
    bool error_signaling_disable;
    bool timestamp_sof;
    bool fault_inject_ena;
    bool pxh_disable;
    bool tx_pause;
} Mhcl_Can_PrtModeType;

/* TX FIFO init config */
typedef struct {
    bool     enabled;
    uint32_t fifo_size;
    uint32_t dc_size_word;
    uint32_t desc_array_addr;   /* caller-provided system memory for descriptors */
    uint32_t dc_start_addr;     /* caller-provided system memory for data containers */
} Mhcl_Can_TxFifoCfgType;

/* TX PQ slot init config */
typedef struct {
    bool     enabled;
    uint32_t dc_size_word;
    uint32_t dc_start_addr;
} Mhcl_Can_TxPqSlotCfgType;

/* TX PQ init config */
typedef struct {
    uint8_t  num_slots;
    uint32_t desc_array_addr;
    Mhcl_Can_TxPqSlotCfgType slot[MHCL_CAN_MAX_TX_PQ_SLOTS];
} Mhcl_Can_TxPqCfgType;

/* RX FIFO init config */
typedef struct {
    bool     enabled;
    uint32_t fifo_size;
    uint32_t dc_size_word;
    uint32_t desc_array_addr;
    uint32_t dc_start_addr;
} Mhcl_Can_RxFifoCfgType;

/* RX filter element config */
typedef struct {
    uint32_t compare0_ref_index;
    uint32_t compare0_word_index;
    bool     compare0_reject_on_match;
    uint32_t compare1_ref_index;
    uint32_t compare1_word_index;
    bool     compare1_reject_on_match;
    bool     blacklist;
    bool     interrupt_enable;
    uint8_t  default_rx_fifo;
} Mhcl_Can_RxFilterElementType;

/* RX filter reference pair */
typedef struct {
    uint32_t value;
    uint32_t mask;
} Mhcl_Can_RxFilterRefPairType;

/* RX filter config */
typedef struct {
    uint32_t num_elements;
    bool     accept_non_matching;
    uint32_t non_matching_fifo;
    bool     accept_non_filtered;
    uint8_t  non_filtered_threshold;
    const Mhcl_Can_RxFilterElementType  *elements;
    uint32_t num_ref_pairs;
    const Mhcl_Can_RxFilterRefPairType  *ref_pairs;
} Mhcl_Can_RxFilterCfgType;

/* IRC config */
typedef struct {
    uint32_t func_ena_mask;
    uint32_t err_ena_mask;
    uint32_t safety_ena_mask;
} Mhcl_Can_IrcCfgType;

/* Full controller init-time configuration */
typedef struct {
    uint32_t  xcan_base_addr;    /* XCAN instance base address */
    uint32_t  lmem_base;         /* Local Memory base address */
    uint32_t  lmem_size_words;
    uint8_t   instance_id;       /* CAN node instance number (0-7) */

    /* Bit timing */
    uint16_t                 brp;
    Mhcl_Can_BitTimingType   nominal;
    Mhcl_Can_BitTimingType   data_fd;
    Mhcl_Can_BitTimingType   data_xl;
    Mhcl_Can_PwmeType        pwme;

    /* PRT mode */
    Mhcl_Can_PrtModeType     prt_mode;

    /* MH global config */
    uint8_t   retrans_max;
    bool      rx_continuous_mode;

    /* FIFOs / PQ */
    Mhcl_Can_TxFifoCfgType   tx_fifo[MHCL_CAN_MAX_TX_FIFO];
    Mhcl_Can_TxPqCfgType     tx_pq;
    Mhcl_Can_RxFifoCfgType   rx_fifo[MHCL_CAN_MAX_RX_FIFO];

    /* RX filter */
    Mhcl_Can_RxFilterCfgType rx_filter;

    /* IRC */
    Mhcl_Can_IrcCfgType      irc;

    /* LMEM layout offsets */
    uint32_t  lmem_fq_base;
    uint32_t  lmem_pq_base;
    uint32_t  lmem_rx_filter_base;
} Mhcl_Can_ConfigType;

/* =========================================================================
 * Controller runtime state
 * ========================================================================= */
typedef struct {
    /* Sub-module base addresses (computed from xcan_base_addr) */
    uint32_t mh_base;
    uint32_t prt_base;
    uint32_t irc_base;
    uint32_t lmem_base;

    uint8_t  instance_id;
    bool     initialized;
    bool     rx_continuous_mode;

    Mhcl_Can_TxFifoState    tx_fifo[MHCL_CAN_MAX_TX_FIFO];
    Mhcl_Can_RxFifoState    rx_fifo[MHCL_CAN_MAX_RX_FIFO];

    /* TX Priority Queue */
    uint8_t                  tx_pq_num_slots;
    uint32_t                 tx_pq_desc_array_addr;
    Mhcl_Can_TxPqSlotState  tx_pq_slot[MHCL_CAN_MAX_TX_PQ_SLOTS];

    /* Callbacks (set by HAL layer) */
    void (*cb_tx_confirmation)(uint8_t controller, uint32_t fifo_or_slot, bool is_pq);
    void (*cb_rx_indication)(uint8_t controller, uint32_t fifo);
    void (*cb_busoff)(uint8_t controller);
    void (*cb_error_passive)(uint8_t controller);
    void (*cb_error_active)(uint8_t controller);
} Mhcl_Can_ControllerType;

/* =========================================================================
 * Init / Deinit
 * ========================================================================= */

/* Full init: LMEM init -> MH global cfg -> TX FIFO setup -> TX PQ setup ->
 * RX FIFO setup -> RX filter -> IRC -> MH start -> RX FIFO start ->
 * PRT config -> PRT start.
 * Based on xcand_config_and_start() + xcand_mh_init(). */
Mhcl_Can_ReturnType mhcl_can_init(Mhcl_Can_ControllerType *ctrl,
                                    const Mhcl_Can_ConfigType *config);

/* Full deinit: PRT stop -> abort all queues -> MH stop.
 * Based on xcand_stop(). */
Mhcl_Can_ReturnType mhcl_can_deinit(Mhcl_Can_ControllerType *ctrl);

/* =========================================================================
 * Mode Control
 * ========================================================================= */

/* PRT start + wait for bus integration.
 * Based on xcan_prt_start_module() + xcan_prt_check_if_started_and_integrated(). */
Mhcl_Can_ReturnType mhcl_can_start(Mhcl_Can_ControllerType *ctrl);

/* PRT stop + abort all + MH stop.
 * Based on xcan_prt_stop_module() + xcand_mh_stop(). */
Mhcl_Can_ReturnType mhcl_can_stop(Mhcl_Can_ControllerType *ctrl);

/* Returns true if PRT is started. */
bool mhcl_can_is_started(const Mhcl_Can_ControllerType *ctrl);

/* =========================================================================
 * TX Path
 * ========================================================================= */

/* Build TX descriptor and enqueue to TX FIFO.
 * Returns 1 on success, 0 if FIFO full. */
int mhcl_can_tx_fifo_enqueue(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id,
                              const Mhcl_Can_MsgType *msg);

/* Build TX descriptor and enqueue to TX Priority Queue slot.
 * Returns 1 on success, 0 if slot busy. */
int mhcl_can_tx_pq_enqueue(Mhcl_Can_ControllerType *ctrl, uint32_t slot_id,
                             const Mhcl_Can_MsgType *msg);

/* Check if TX FIFO is full (VALID bit of current put-index descriptor). */
bool mhcl_can_tx_fifo_is_full(const Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id);

/* Check and clear TX FIFO SENT status for a specific FIFO.
 * Returns true if the FIFO had a completed transmission. */
bool mhcl_can_tx_fifo_check_sent(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id);

/* Check if TX PQ slot is busy (register-based). */
bool mhcl_can_tx_pq_is_busy(const Mhcl_Can_ControllerType *ctrl, uint32_t slot_id);

/* Abort + disable a TX FIFO. */
void mhcl_can_tx_fifo_abort(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id);

/* =========================================================================
 * RX Path
 * ========================================================================= */

/* Dequeue one message from RX FIFO.
 * Returns 1 on success, 0 if FIFO empty. */
int mhcl_can_rx_fifo_dequeue(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id,
                              Mhcl_Can_MsgType *msg);

/* Check if RX FIFO is empty (VALID bit of current get-index descriptor). */
bool mhcl_can_rx_fifo_is_empty(const Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id);

/* Abort + disable an RX FIFO. */
void mhcl_can_rx_fifo_abort(Mhcl_Can_ControllerType *ctrl, uint32_t fifo_id);

/* =========================================================================
 * Baudrate
 * ========================================================================= */

/* Configure bit timing: stops PRT if needed, writes NBTP/DBTP/XBTP/PWME/MODE.
 * Based on xcan_prt_set_config(). */
Mhcl_Can_ReturnType mhcl_can_set_baudrate(Mhcl_Can_ControllerType *ctrl,
                                            uint16_t brp,
                                            const Mhcl_Can_BitTimingType *nominal,
                                            const Mhcl_Can_BitTimingType *data_fd,
                                            const Mhcl_Can_BitTimingType *data_xl,
                                            const Mhcl_Can_PwmeType *pwme,
                                            const Mhcl_Can_PrtModeType *mode);

/* =========================================================================
 * Error / Status
 * ========================================================================= */

/* Get current error state (active / passive / busoff). */
Mhcl_Can_ErrorStateType mhcl_can_get_error_state(const Mhcl_Can_ControllerType *ctrl);

/* Read Transmit Error Counter. */
uint32_t mhcl_can_get_tec(const Mhcl_Can_ControllerType *ctrl);

/* Read Receive Error Counter. */
uint32_t mhcl_can_get_rec(const Mhcl_Can_ControllerType *ctrl);

/* =========================================================================
 * Interrupt Management
 * ========================================================================= */

/* Enable interrupts via IRC. */
void mhcl_can_enable_interrupts(Mhcl_Can_ControllerType *ctrl,
                                 uint32_t func_mask, uint32_t err_mask,
                                 uint32_t safety_mask);

/* Disable all IRC interrupts. */
void mhcl_can_disable_interrupts(Mhcl_Can_ControllerType *ctrl);

/* Process functional interrupt line.
 * Based on xcand_process_irq_func(). */
void mhcl_can_process_irq_func(Mhcl_Can_ControllerType *ctrl);

/* Process error interrupt line.
 * Based on xcand_process_irq_err(). */
void mhcl_can_process_irq_err(Mhcl_Can_ControllerType *ctrl);

/* Process safety interrupt line.
 * Based on xcand_process_irq_safety(). */
void mhcl_can_process_irq_safety(Mhcl_Can_ControllerType *ctrl);

/* =========================================================================
 * DLC Conversion
 * ========================================================================= */

/* Convert DLC to data field bytes. Based on xcand_mh_convert_dlc_to_data_field_bytes(). */
uint32_t mhcl_can_dlc_to_bytes(Mhcl_Can_FrameFormatType ff, uint32_t dlc);

/* Convert data field bytes to DLC (reverse mapping). */
uint32_t mhcl_can_bytes_to_dlc(Mhcl_Can_FrameFormatType ff, uint32_t bytes);

#endif /* MHCL_CAN_H */
