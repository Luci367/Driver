# CAN MHAL API to Implementation Mapping

This document maps each MHAL layer API to the corresponding low-level driver function.

## Layer Architecture

```
┌─────────────────────────────────────────┐
│              MCAL Layer                 │  ← Calls MHAL
├─────────────────────────────────────────┤
│         MHAL (can_mhal.h)               │  ← Hardware Abstraction
├─────────────────────────────────────────┤
│         MHCL (Platform Independent)     │  ← To be implemented
├─────────────────────────────────────────┤
│         MHDL (can_driver.c/h)           │  ← Low-level driver
└─────────────────────────────────────────┘
```

## API Mapping Table

| MHAL API | Driver Function | Status |
|----------|-----------------|--------|
| **Initialization** | | |
| `can_hal_init` | `can_init()` | ✓ Implemented |
| `can_hal_deinit` | `can_deinit()` | ✓ Implemented |
| **Mode Control** | | |
| `can_hal_set_controller_mode` | `can_start()`, `can_stop()` | ✓ Implemented |
| `can_hal_get_controller_mode` | Read PRT_STAT register | ✓ Implementable |
| `can_hal_get_controller_state` | `can_get_stats()`, `can_get_bus_state()` | ✓ Implemented |
| **Transmission** | | |
| `can_hal_write` | `can_tx_fifo_push_ext()` | ✓ Implemented |
| `can_hal_write_xl` | `can_tx_fifo_push_ext()` with XL flag | ✓ Implemented |
| `can_hal_write_fifo` | `can_tx_fifo_push_ext()` | ✓ Implemented |
| `can_hal_write_priority` | `can_tx_priority_slot()` | ✓ Implemented |
| `can_hal_abort_tx` | `can_tx_abort()` | ✓ Implemented |
| `can_hal_abort_tx_priority` | `can_tx_priority_abort()` | ✓ Implemented |
| `can_hal_tx_is_busy` | `can_tx_fifo_is_busy()` | ✓ Implemented |
| **Reception** | | |
| `can_hal_read` | `can_rx_read()` | ✓ Implemented |
| `can_hal_rx_fifo_setup` | `can_rx_fifo_setup()` | ✓ Implemented |
| `can_hal_rx_fifo_setup_continuous` | `can_rx_fifo_setup_continuous()` | ✓ Implemented |
| `can_hal_rx_has_message` | `can_rx_has_message()` | ✓ Implemented |
| `can_hal_rx_restart` | `can_rx_restart()` | ✓ Implemented |
| `can_hal_abort_rx` | `can_rx_abort()` | ✓ Implemented |
| `can_hal_rx_is_busy` | `can_rx_fifo_is_busy()` | ✓ Implemented |
| `can_hal_rx_get_fill_level` | `can_rx_get_fill_level()` | ✓ Implemented |
| `can_hal_rx_update_read_ptr` | `can_rx_update_read_ptr()` | ✓ Implemented |
| **Interrupts** | | |
| `can_hal_enable_controller_interrupts` | `can_interrupt_enable()` | ✓ Implemented |
| `can_hal_disable_controller_interrupts` | Write 0 to IRC_ENA registers | ✓ Implementable |
| `can_hal_irq_handler` | `can_irq_handler()` | ✓ Implemented |
| `can_hal_get_irq_pending` | `can_get_irq_pending()` | ✓ Implemented |
| `can_hal_clear_irq` | `can_interrupt_clear()` | ✓ Implemented |
| `can_hal_register_callbacks` | `can_register_callbacks()` | ✓ Implemented |
| **Baud Rate** | | |
| `can_hal_set_baudrate` | `can_set_bit_timing()` | ✓ Implemented |
| `can_hal_check_baudrate` | Timing calculation | ✓ Implementable |
| **Error/Status** | | |
| `can_hal_get_controller_error_state` | `can_get_stats()` | ✓ Implemented |
| `can_hal_get_bus_state` | `can_get_bus_state()` | ✓ Implemented |
| `can_hal_get_stats` | `can_get_stats()` | ✓ Implemented |
| `can_hal_clear_stats` | `can_clear_stats()` | ✓ Implemented |
| `can_hal_get_current_time` | Read timestamp registers | ✓ Implementable |
| **Utility** | | |
| `can_hal_set_loopback` | `can_set_loopback()` | ✓ Implemented |
| `can_hal_set_listen_only` | `can_set_listen_only()` | ✓ Implemented |
| `can_hal_get_version_info` | `can_get_version()` | ✓ Implemented |
| `can_hal_software_reset` | `can_software_reset()` | ✓ Implemented |
| `can_hal_check_wakeup` | Read wakeup status | ✓ Implementable |
| **Polling Mode** | | |
| `can_hal_main_function_read` | Poll RX FIFOs | ✓ Implementable |
| `can_hal_main_function_write` | Poll TX status | ✓ Implementable |
| `can_hal_main_function_busoff` | Poll bus state | ✓ Implementable |
| `can_hal_main_function_wakeup` | Poll wakeup | ✓ Implementable |

## Notes

1. **Controller Parameter**: MHAL APIs take a `Controller` parameter. The MHCL layer will map this to `base_addr` used by MHDL functions.

2. **HTH Mapping**: `can_hal_write()` uses HTH (Hardware Transmit Handle). This maps to a specific controller and FIFO via `Can_Hal_HthMappingType`.

3. **Status Returns**: MHAL uses `Can_Hal_ReturnType` enum. MHCL converts from `enum can_error`.

4. **Wakeup**: Hardware wakeup support depends on IP configuration. API provided for compatibility.

5. **Main Functions**: These are for polling mode operation as alternative to interrupt-driven mode.

## Files Created

| File | Description |
|------|-------------|
| `Std_Types.h` | Standard type definitions (AUTOSAR style) |
| `Can_Types.h` | CAN-specific type definitions |
| `Can_Cfg.h` | Configuration types |
| `can_mhal.h` | MHAL API prototypes |
