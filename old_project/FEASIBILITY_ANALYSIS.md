# XCAN Feasibility Analysis: Old HAL API + MCAL Config Reuse

## Source Material
- **Old project**: `Can_Cfg.h`, `Can_PBCfg.c`, `old_mhal_can.h` (extracted from screenshots)
- **XCAN reference**: `xcan_user_manual_v390.pdf`, `xcan_sw_example/` (Bosch example driver)

---

## 1. OLD HAL API vs XCAN — API-by-API Analysis

### 1.1 APIs That Can Be Reused (Signature Compatible)

| # | Old API | XCAN Support | Notes |
|---|---------|-------------|-------|
| 1 | `can_hal_init(cid)` | YES | XCAN init requires: LMEM init, MH global config (instance number, retrans_max, FIFO mode), TX/RX FIFO queue setup (descriptor alloc, data container alloc, register config), RX filter config, MH start, RX FIFO start, PRT config + start. Much more complex than old CAN init. |
| 2 | `can_hal_deinit(cid)` | YES | XCAN: PRT stop (unlock sequence + STOP bit) -> MH stop (abort all queues + clear START). |
| 3 | `can_hal_set_controller_mode(cid, Transition)` | PARTIAL | START/STOP supported. SLEEP/WAKEUP NOT supported (XCAN has no sleep mode). |
| 4 | `can_hal_enable_controller_interrupt(cid)` | NEEDS CHANGE | XCAN IRC has 3 separate lines: Functional, Error, Safety. Old API takes no mask — needs masks for each line. |
| 5 | `can_hal_disable_controller_interrupt(cid)` | YES | Can disable all 3 IRC lines. |
| 6 | `can_hal_get_error_state(cid)` | YES | XCAN PRT STAT register has: BO (bus-off), EP (error-passive), TEC, REC. Maps directly. |
| 7 | `can_hal_get_rx_error_count(cid)` | YES | PRT STAT.REC register (7-bit). |
| 8 | `can_hal_get_tx_error_count(cid)` | YES | PRT STAT.TEC register (8-bit). |
| 9 | `can_hal_mainfunction_write(cid)` | YES | Poll TX FIFO interrupt status (TX_FQ_INT_STS.Sent bits) for each enabled FIFO. |
| 10 | `can_hal_mainfunction_read(cid)` | YES | Poll RX FIFO empty status, dequeue messages. |
| 11 | `can_hal_mainfunction_busoff(cid)` | YES | Poll PRT STAT.BO bit. On bus-off, PRT auto-recovers after start command triggers 128x11 recessive bit sequence. |
| 12 | `can_hal_interruupt_handler(cid)` | NEEDS CHANGE | XCAN has 3 separate ISR lines (func/err/safety), not 1. Old single-handler model needs splitting or internal dispatch. |

### 1.2 APIs That Need Signature Changes

| # | Old API | Problem | Required Change |
|---|---------|---------|----------------|
| 13 | `can_hal_write(Hth, Pdu)` | XCAN uses FIFO queues (0-7) + Priority Queue (0-31 slots), not abstract HTH. Message enqueue requires: build TX descriptor (header words T0/T1/T2), write data to Data Container, set VALID bit, start FIFO. | Change to `can_hal_write(Controller, FifoId, PduInfo)` or add HTH→FIFO mapping layer. |
| 14 | `can_hal_read(Hth, Pdu)` | XCAN reads from RX FIFO queues by dequeuing: check rolling counter, read RX descriptor, parse header (R0/R1/R2), copy data from Data Container. | Change to `can_hal_read(Controller, FifoId, MsgPtr)` or add HRH→FIFO mapping layer. |
| 15 | `can_hal_set_baudrate(cid, arb_baudrate)` | XCAN needs 3 separate bit timing configs: NBTP (nominal/arb), DBTP (FD data), XBTP (XL data). Each has PropSeg, PhaseSeg1, PhaseSeg2, SJW, TDC offset. Plus PWME config for XL transceiver. Single `arb_baudrate` value is insufficient. | Need full timing struct per phase, or baudrate-to-timing lookup table. |

### 1.3 APIs That Cannot Be Implemented

| # | Old API | Reason |
|---|---------|--------|
| - | `CAN_T_SLEEP` / `CAN_T_WAKEUP` transitions | XCAN IP has no sleep/wakeup hardware mechanism |
| - | `CAN_WAKEUP_INTERRUPT` / `CAN_WAKEUP_POLLING` | No wakeup source in XCAN |

---

## 2. MCAL CONFIG (`Can_Cfg.h`) — Field-by-Field Analysis

### 2.1 Defines That Can Be Reused As-Is

| Define | Value | XCAN Compatibility |
|--------|-------|--------------------|
| `CAN_DEV_ERROR_DETECT` | STD_ON | Software-only, reusable |
| `CAN_GLOBAL_TIME_SUPPORT` | STD_ON | XCAN has 64-bit timestamps per frame |
| `CAN_SETBAUDRATE_API` | STD_ON | Supported (PRT must be STOPPED) |
| `CAN_FD_MODE_ENABLE` | STD_ON | XCAN supports CC/FD/XL |
| `CAN_INTERRUPT_ENABLE` | STD_ON | XCAN IRC supports full interrupt control |
| `CAN_CTRL_CONFIG_CNT` | 8u | XCAN is 1 node per instance; 8 instances need 8 XCAN IPs |
| `CAN_HW_OBJECT_CNT` | 4u | Reusable as logical mapping |
| `CAN_HTH_START_IDX` | 2u | Logical index, reusable with mapping layer |
| `CAN_MAINF_BUSOFF_PERIOD` | 5u | Software period, reusable |
| `CAN_MAINF_MODE_PERIOD` | 1u | Software period, reusable |
| `CAN_TIMEOUT_DURATION` | 41u | Software timeout, reusable |

### 2.2 Defines That Need Changes

| Define | Issue | Required Change |
|--------|-------|----------------|
| `CAN_MAINF_WAKEUP_PERIOD` | XCAN has no wakeup | Remove |
| `RX_Q_SIZE` / `TX_Q_SIZE` (3u) | XCAN FIFO queue sizes are set per-FIFO (1-1024 descriptors). Old project uses software queue; XCAN has HW FIFO queues. | Replace with per-FIFO descriptor count config |
| `CAN_FILTER_CNT` (1u) | XCAN supports up to 255 RX filter elements, each comparing 32-bit words with mask/value pairs. Much more powerful than old filter. | Expand to XCAN filter element config |

### 2.3 New Defines Needed for XCAN

| Define | Purpose |
|--------|---------|
| `CAN_XL_MODE_ENABLE` | Enable CAN XL support |
| `CAN_TDC_ENABLE` | Transceiver Delay Compensation |
| `CAN_RX_FIFO_CONTINUOUS_MODE` | Normal vs Continuous RX mode |
| `CAN_RETRANS_MAX` | Retransmission attempts (0-7, 7=unlimited) |
| `CAN_LMEM_SIZE` | Local Memory size for TX descriptors + RX filters |
| `CAN_TX_PRIORITY_QUEUE_SLOTS` | Number of TX priority queue slots (0-32) |
| `CAN_XL_BIT_RATE_*` | XL data phase bit timing params |

---

## 3. MCAL CONFIG (`Can_PBCfg.c`) — Structure-by-Structure Analysis

### 3.1 `Can_ControllerBaudrateCfgType` — NEEDS MAJOR REWORK

**Old struct** (from screenshots):
```
{BaudRateConfigId, PropSeg, PhaseSeg1, PhaseSeg2, SyncJumpWidth, FdBaudratePtr}
```

**XCAN requires** (from PRT registers):
- NBTP: BRP (5-bit), NTSEG1 (9-bit = PropSeg+PhaseSeg1-1), NTSEG2 (7-bit), NSJW (7-bit)
- DBTP: DTSEG1 (8-bit), DTSEG2 (7-bit), DSJW (7-bit), DTDCO (8-bit TDC offset)
- XBTP: XTSEG1 (8-bit), XTSEG2 (7-bit), XSJW (7-bit), XTDCO (8-bit)
- PWME: PWMS, PWML, PWMO (for XL transceiver switching)

**Key differences:**
1. **Shared BRP**: XCAN uses a single BRP for ALL phases (nominal, FD data, XL data). Old project could have different prescalers.
2. **TDC offset**: XCAN requires per-phase TDC offset. Old project doesn't have this.
3. **TSEG1 = PropSeg + PhaseSeg1**: XCAN combines them. Old project keeps them separate.
4. **XL timing**: Entirely new — old project has no XL bit timing.
5. **PWME**: New for XL transceiver mode switching.

### 3.2 `Can_ControllerType` — NEEDS REWORK

**Old struct fields** (from screenshot):
```
ControllerId, ActivateStatus, ClockFrequency, BusoffProcessing,
BaudrateCfgPtr, TxProcessing, RxProcessing, WakeupProcessing
```

**Changes needed:**

| Old Field | XCAN Change |
|-----------|-------------|
| `ControllerId` | Keep — maps to XCAN instance number (MH_CFG.INSTNUM) |
| `ActivateStatus` | Keep |
| `ClockFrequency` | Keep — XCAN recommends 160MHz for XL |
| `BusoffProcessing` | Keep (INTERRUPT/POLLING) — XCAN IRC has PRT_BUS_OFF event |
| `BaudrateCfgPtr` | Rework — needs NBTP/DBTP/XBTP/PWME structs |
| `TxProcessing` | Keep (INTERRUPT/POLLING/MIXED) — XCAN IRC has per-FIFO TX IRQs |
| `RxProcessing` | Keep (INTERRUPT/POLLING/MIXED) — XCAN IRC has per-FIFO RX IRQs |
| `WakeupProcessing` | **REMOVE** — XCAN has no wakeup |

**New fields needed:**
- `BaseAddress` — register base (MH at offset 0x000, PRT at 0x900, IRC at 0xA00)
- `LmemBaseAddress` — Local Memory base address
- `LmemSize` — Local Memory size in words
- `XlEnable` — CAN XL operation mode flag
- `FdEnable` — CAN FD operation mode flag
- `TdcEnable` — Transceiver Delay Compensation enable
- `RetransMax` — max retransmission attempts (0-7)

### 3.3 `Can_HardwareObjectType` — NEEDS REWORK

**Old struct fields** (from screenshots):
```
CanControllerRef, CanObjectId, CanIdType, HwFifoId,
CanObjectType, HandleType(BASIC/FULL), PollingMode,
HwObjectCount, FilterCount, HwFilterRef, FdPaddingEnable,
CanObjectPayloadLength, MainFunctionPeriod
```

**Changes needed:**

| Old Field | XCAN Change |
|-----------|-------------|
| `CanControllerRef` | Keep |
| `CanObjectId` | Keep — logical handle |
| `CanIdType` (STANDARD/EXTENDED/MIXED) | Keep — XCAN RX filter can match on XTD bit in R0 |
| `HwFifoId` | **Critical** — maps directly to XCAN TX FIFO (0-7) or RX FIFO (0-7) |
| `CanObjectType` (TX/RX) | Keep |
| `HandleType` (BASIC/FULL) | Rethink — XCAN has FIFO Queue (BASIC) and Priority Queue (FULL-like, 32 slots) |
| `PollingMode` | Keep — determines if IRQ or polling for this object |
| `HwObjectCount` | Keep — number of descriptors in the FIFO |
| `HwFilterRef` | Rework — XCAN filter elements compare header words (R0/R1/R2) with value+mask pairs |
| `CanObjectPayloadLength` | Keep — determines Data Container size per descriptor |
| `MainFunctionPeriod` | Keep — polling period |

**New fields needed:**
- `DcSizeWord` — Data Container size in words (payload storage per descriptor)
- `IsContinuousMode` — for RX FIFOs, continuous vs normal mode
- `PriorityQueueSlotId` — for TX objects using Priority Queue (0-31)

### 3.4 `Can_HwFilterType` — NEEDS COMPLETE REWORK

**Old filter**: Simple `{mask, value}` pair for CAN ID filtering.

**XCAN RX filter** (from manual and example):
- Up to 255 filter elements, each stored in Local Memory (L_MEM)
- Each element has TWO comparisons (compare0 and compare1)
- Each comparison selects a header word (R0, R1, or R2) and a reference pair index
- Reference pairs are `{value, mask}` — up to 256 pairs
- Each element has: `accept/reject on match`, `blacklist`, `interrupt enable`, `default FIFO number`
- R0 contains: FDF, XLF, XTD, BaseID, ExtID, VCID, SDT
- R1 contains: RTR, BRS, ESI, DLC
- R2 contains: first data word (CC/FD) or Acceptance Field (XL)

This is fundamentally more powerful than the old `{mask, id}` filter.

---

## 4. `CanCtrlStatus` (Runtime State) — NEEDS REWORK

**Old struct** (from screenshot):
```c
typedef struct {
    CtrlState;       // state machine
    RefCounter;      // interrupt nesting counter
    IrqEn;           // interrupt enable flag
    HthObjBusy;      // bitmask of busy HTHs
    ArbBaudrate;     // current arb baudrate
    DataBaudrate;    // current FD data baudrate (if FD enabled)
    CanCFGSet;       // pointer to config
    RxCurrent;       // software queue current pointer
    TxCurrent;       // software queue current pointer
    RxQ;             // software RX queue
    TxQ;             // software TX queue
} CanCtrlStatus;
```

**XCAN changes:**
1. **Remove `RxQ`/`TxQ`/`RxCurrent`/`TxCurrent`** — XCAN has hardware FIFO queues managed by the Message Handler DMA. No software queue needed.
2. **Replace `HthObjBusy` (bitmask)** — XCAN checks FIFO full/empty via `TX_FQ_STS0` and `RX_FQ_STS0` registers per queue.
3. **Add per-FIFO state tracking** — rolling counters, put/get indices, descriptor pointers (as seen in example driver's `xcand_mh_tx_fifo_struct` and `xcand_mh_rx_fifo_struct`).
4. **Add PRT status** — bus_on, bus_off, error_passive, error_active, TEC, REC.
5. **Add IRC state** — which interrupts are enabled per line.

---

## 5. ARCHITECTURAL DIFFERENCES SUMMARY

| Aspect | Old CAN IP | XCAN IP |
|--------|-----------|---------|
| **Architecture** | Monolithic register set | 3 sub-modules: MH + PRT + IRC |
| **TX mechanism** | Mailbox / single buffer | 8 FIFO queues (up to 1024 desc each) + 1 Priority Queue (32 slots) |
| **RX mechanism** | Mailbox / single buffer | 8 FIFO queues (up to 1024 desc each), Normal or Continuous mode |
| **Memory model** | Internal message RAM | External System Memory (DMA) + Local Memory (L_MEM for descriptors/filters) |
| **Descriptor model** | None | TX/RX descriptors (32/16 bytes) with rolling counters, valid bits, header words |
| **Data containers** | Inline in mailbox | Separate memory area, pointed to by descriptors |
| **Filtering** | Simple mask/ID match | 255 filter elements, 256 reference pairs, 2 comparisons per element, match on any header word |
| **Interrupts** | Single line | 3 lines: Functional, Error, Safety |
| **Start/Stop** | Simple mode register | PRT: unlock sequence + STOP/START; MH: separate start/stop; FIFOs: per-FIFO start/abort |
| **Sleep/Wakeup** | Supported | NOT supported |
| **CAN XL** | Not supported | Up to 2048 bytes, XL bit timing, PWME |
| **Clock** | 80 MHz typical | 160 MHz recommended for XL |
| **Baud rate** | Single prescaler + segments | Shared BRP + per-phase TSEG + TDC offset |

---

## 6. VERDICT & RECOMMENDATIONS

### Can the old HAL API be reused for XCAN?

**NO — not directly.** The architectural gap is too large. The old project assumes a simple mailbox-based CAN controller. XCAN uses a DMA-based FIFO queue architecture with descriptors, data containers, and 3 sub-modules.

### What can be preserved?

1. **API naming convention and AUTOSAR layering** (MCAL -> MHAL -> MHCL pattern)
2. **State machine logic** (UNINIT -> STOPPED -> STARTED transitions)
3. **Polling main functions** (concept of periodic read/write/busoff polling)
4. **Error state retrieval** (active/passive/bus-off maps 1:1)
5. **Error counter APIs** (TEC/REC directly available)
6. **Interrupt nesting control** (RefCounter pattern)

### What must be redesigned?

1. **Init/Deinit** — Must handle: LMEM init, MH global config, per-FIFO queue setup (descriptor + data container allocation), filter config, PRT config, IRC config, multi-step start sequence
2. **Write/Read APIs** — Must work with XCAN FIFO queue model (build descriptors, manage rolling counters, DMA-based data containers)
3. **Baudrate config** — Must support 3 separate timing phases (NBTP/DBTP/XBTP) with shared BRP + TDC offsets
4. **Interrupt handling** — Must split into 3 lines (func/err/safety) with per-event enable masks
5. **Configuration types** — All PB config structs need significant rework for XCAN register model
6. **Runtime state** — Replace software queues with HW FIFO queue state tracking
7. **Wakeup** — Remove entirely
8. **RX filtering** — Completely new model needed
9. **CAN XL support** — New write_xl API, XL PDU type, XL bit timing, VCID/SDT/AF fields
