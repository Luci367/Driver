# CAN XL Driver

A production-ready CAN CC/FD/XL driver based on Bosch X_CAN IP v3.9 specifications.

## Features

- **Multi-Protocol Support**: CAN CC (Classical), CAN FD, and CAN XL
- **8 TX FIFO Queues**: Independent transmit queues with priority support
- **8 RX FIFO Queues**: Independent receive queues with filtering
- **32 TX Priority Queue Slots**: Immediate priority-based transmission
- **Flexible Bit Timing**: Configurable NBTP/DBTP/XBTP registers
- **Interrupt-Driven**: Full IRC support with callback registration
- **Linux Kernel Style**: Clean, maintainable C code

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Host CPU                                │
│                         │                                    │
│                    ┌────┴────┐                               │
│                    │ HOST AXI │                              │
│                    └────┬────┘                               │
└─────────────────────────┼───────────────────────────────────┘
                          │
┌─────────────────────────┼───────────────────────────────────┐
│                    X_CAN IP                                  │
│  ┌──────────┐    ┌──────┴──────┐    ┌────────────┐          │
│  │   IRC    │◄───│     MH      │───►│    PRT     │          │
│  │(Interrupt│    │  (Message   │    │ (Protocol  │          │
│  │ Control) │    │   Handler)  │    │ Controller)│          │
│  └──────────┘    └──────┬──────┘    └─────┬──────┘          │
│                         │                  │                 │
│                    ┌────┴────┐        ┌────┴────┐           │
│                    │  LMEM   │        │ CAN Bus │           │
│                    └─────────┘        └─────────┘           │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

```c
#include "can_driver.h"

struct can_config cfg = {
    .base_addr = 0x40000000,
    .protocol = CAN_PROTOCOL_FD,
    .nominal_timing = { .brp = 1, .tseg1 = 63, .tseg2 = 16, .sjw = 16 },
    .data_timing = { .brp = 1, .tseg1 = 15, .tseg2 = 4, .sjw = 4, .tdco = 16 },
};

/* Initialize TX/RX queues */
cfg.tx_fifo_queues[0].enabled = true;
cfg.tx_fifo_queues[0].start_addr = 0x10000000;
cfg.tx_fifo_queues[0].size = 64;

cfg.rx_fifo_queues[0].enabled = true;
cfg.rx_fifo_queues[0].start_addr = 0x10010000;
cfg.rx_fifo_queues[0].size = 64;
cfg.rx_fifo_queues[0].dc_size = 8;

/* Initialize driver */
if (can_init(&cfg) != CAN_ERROR_NONE) {
    /* Handle error */
}

/* Transmit a message */
uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
can_tx_fifo_push(cfg.base_addr, 0, 0x123, data, 4, false, false, false);

/* Receive a message */
struct can_msg msg;
if (can_rx_read(cfg.base_addr, 0, &msg, 1000) == CAN_ERROR_NONE) {
    /* Process message */
}
```

## API Reference

### Core Functions
- `can_init()` - Initialize CAN controller
- `can_deinit()` - Deinitialize CAN controller
- `can_start()` / `can_stop()` - Start/stop controller

### TX Functions
- `can_tx_fifo_push()` - Push message to TX FIFO
- `can_tx_priority_slot()` - Send via priority queue
- `can_tx_abort()` - Abort TX queue

### RX Functions
- `can_rx_fifo_setup()` - Configure RX FIFO
- `can_rx_read()` - Read received message
- `can_rx_restart()` - Restart RX FIFO

### Interrupt Functions
- `can_interrupt_enable()` - Enable interrupts
- `can_register_callbacks()` - Register IRQ callbacks
- `can_irq_handler()` - Main interrupt handler

## File Structure

```
CAN_Driver/
├── can_driver.h      # Header with types, macros, prototypes
├── can_driver.c      # Driver implementation
└── README.md         # This file
```

## Reference

Based on Bosch X_CAN IP v3.9 User Manual specifications.

## License

SPDX-License-Identifier: GPL-2.0-or-later
