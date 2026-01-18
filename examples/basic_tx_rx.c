/**
 * @file    basic_tx_rx.c
 * @brief   Basic CAN TX/RX Example
 * @details Demonstrates basic transmit and receive operations.
 *          Reference: X_CAN User Manual v3.9, Section 1.4.7
 */

#include "../can_driver.h"
#include <stdio.h>
#include <string.h>

/* Platform-specific memory allocation (replace with your implementation) */
static uint8_t tx_queue_mem[1024] __attribute__((aligned(32)));
static uint8_t rx_queue_mem[2048] __attribute__((aligned(32)));

/* Callback for RX interrupt */
static void rx_callback(uint8_t fifo_id, void *ctx) {
    (void)ctx;
    printf("RX interrupt on FIFO %u\n", fifo_id);
}

/* Callback for TX interrupt */
static void tx_callback(uint8_t fifo_id, void *ctx) {
    (void)ctx;
    printf("TX complete on FIFO %u\n", fifo_id);
}

/* Callback for bus state changes */
static void bus_state_callback(can_bus_state_t state, void *ctx) {
    (void)ctx;
    const char *state_names[] = {"ACTIVE", "WARNING", "PASSIVE", "BUS_OFF"};
    printf("Bus state changed to: %s\n", state_names[state]);
}

int main(void) {
    can_config_t config;
    can_irq_callbacks_t callbacks;
    can_msg_t rx_msg;
    can_stats_t stats;
    can_error_t status;
    
    /* Test data */
    uint8_t tx_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    
    printf("CAN Driver Basic Example\n");
    printf("========================\n\n");
    
    /*------------------------------------------------------------------------*/
    /* Step 1: Initialize configuration structure                             */
    /* Reference: Section 1.4.7.1 Initial MH Start Procedure                  */
    /*------------------------------------------------------------------------*/
    memset(&config, 0, sizeof(config));
    
    /* Base address - replace with your platform's CAN controller address */
    config.base_addr = CAN_BASE;
    config.lmem_base_addr = CAN_BASE + 0x10000;
    
    /* Protocol: CAN FD */
    config.protocol = CAN_PROTOCOL_FD;
    config.mode = CAN_MODE_NORMAL;
    
    /*------------------------------------------------------------------------*/
    /* Step 2: Configure bit timing                                           */
    /* Reference: Section 1.5.4.2.4 NBTP, DBTP registers                      */
    /* Example: 500 kbit/s nominal, 2 Mbit/s data (80 MHz clock)              */
    /*------------------------------------------------------------------------*/
    config.nominal_timing.brp = 0;
    config.nominal_timing.tseg1 = 127;
    config.nominal_timing.tseg2 = 32;
    config.nominal_timing.sjw = 32;
    
    config.data_timing.brp = 0;
    config.data_timing.tseg1 = 31;
    config.data_timing.tseg2 = 8;
    config.data_timing.sjw = 8;
    config.data_timing.tdco = 32;
    
    /*------------------------------------------------------------------------*/
    /* Step 3: Configure TX FIFO Queue 0                                      */
    /* Reference: Section 1.4.7.6 TX FIFO Queue Initial Start                 */
    /*------------------------------------------------------------------------*/
    config.tx_fifo_queues[0].enabled = true;
    config.tx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)tx_queue_mem;
    config.tx_fifo_queues[0].size = 16;  /* 16 descriptors */
    
    /*------------------------------------------------------------------------*/
    /* Step 4: Configure RX FIFO Queue 0                                      */
    /* Reference: Section 1.4.7.3 RX FIFO Queue Initial Start                 */
    /*------------------------------------------------------------------------*/
    config.rx_fifo_queues[0].enabled = true;
    config.rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_queue_mem;
    config.rx_fifo_queues[0].size = 32;  /* 32 descriptors */
    config.rx_fifo_queues[0].dc_size = 2; /* 64 bytes per data container */
    config.rx_fifo_queues[0].continuous = false;
    
    /*------------------------------------------------------------------------*/
    /* Step 5: Enable interrupts                                              */
    /* Reference: Section 1.4.7.1 Step 7                                      */
    /*------------------------------------------------------------------------*/
    config.func_int_enable = 0x000001FF;  /* TX/RX FIFO 0-7 + TX PQ */
    config.err_int_enable = 0x0000000F;   /* Error interrupts */
    
    /*------------------------------------------------------------------------*/
    /* Step 6: Initialize the controller                                      */
    /*------------------------------------------------------------------------*/
    printf("Initializing CAN controller...\n");
    status = can_init(&config);
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: can_init() failed with code %d\n", status);
        return -1;
    }
    printf("CAN controller initialized successfully.\n\n");
    
    /*------------------------------------------------------------------------*/
    /* Step 7: Register interrupt callbacks                                   */
    /*------------------------------------------------------------------------*/
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.rx_callbacks[0] = rx_callback;
    callbacks.tx_callbacks[0] = tx_callback;
    callbacks.bus_state_callback = bus_state_callback;
    
    status = can_register_callbacks(config.base_addr, &callbacks);
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: Failed to register callbacks\n");
    }
    
    /*------------------------------------------------------------------------*/
    /* Step 8: Transmit a message                                             */
    /* Reference: Section 1.4.5.5 TX Descriptor                               */
    /*------------------------------------------------------------------------*/
    printf("Transmitting CAN FD message...\n");
    printf("  ID: 0x123 (Standard)\n");
    printf("  Length: 8 bytes\n");
    printf("  Data: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", tx_data[i]);
    }
    printf("\n");
    
    status = can_tx_fifo_push(config.base_addr, 0, 0x123, tx_data, 8,
                              true,   /* FD frame */
                              false,  /* Not XL */
                              false); /* Not remote */
    
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: can_tx_fifo_push() failed with code %d\n", status);
    } else {
        printf("Message queued for transmission.\n\n");
    }
    
    /*------------------------------------------------------------------------*/
    /* Step 9: Wait for and receive a message                                 */
    /* Reference: Section 1.4.5.7 RX Descriptor                               */
    /*------------------------------------------------------------------------*/
    printf("Waiting for RX message (5 second timeout)...\n");
    
    status = can_rx_read(config.base_addr, 0, &rx_msg, 5000000);
    
    if (status == CAN_ERROR_NONE) {
        printf("Message received!\n");
        printf("  ID: 0x%X (%s)\n", rx_msg.id, 
               rx_msg.extended ? "Extended" : "Standard");
        printf("  Length: %u bytes\n", rx_msg.len);
        printf("  Frame type: %s\n", 
               rx_msg.xl ? "CAN XL" : (rx_msg.fd ? "CAN FD" : "CAN Classic"));
        printf("  Data: ");
        for (uint32_t i = 0; i < rx_msg.len && i < 16; i++) {
            printf("%02X ", rx_msg.data[i]);
        }
        printf("\n");
        printf("  Timestamp: %llu\n", (unsigned long long)rx_msg.timestamp);
    } else if (status == CAN_ERROR_TIMEOUT) {
        printf("Timeout - no message received.\n");
    } else if (status == CAN_ERROR_QUEUE_EMPTY) {
        printf("Queue empty - no message available.\n");
    } else {
        printf("ERROR: can_rx_read() failed with code %d\n", status);
    }
    
    /*------------------------------------------------------------------------*/
    /* Step 10: Get statistics                                                */
    /* Reference: Section 1.5.4.2.2 STAT register                             */
    /*------------------------------------------------------------------------*/
    printf("\nCAN Statistics:\n");
    can_get_stats(config.base_addr, &stats);
    printf("  TX Error Count (TEC): %u\n", stats.tx_error_count);
    printf("  RX Error Count (REC): %u\n", stats.rx_error_count);
    printf("  Bus State: %s\n", 
           stats.bus_off ? "BUS_OFF" :
           stats.error_passive ? "ERROR_PASSIVE" :
           stats.error_warning ? "ERROR_WARNING" : "ERROR_ACTIVE");
    printf("  TX Success: %u\n", stats.tx_success_count);
    printf("  RX Success: %u\n", stats.rx_success_count);
    
    /*------------------------------------------------------------------------*/
    /* Step 11: Cleanup                                                       */
    /* Reference: Section 1.4.7.2 Stopping MH Procedure                       */
    /*------------------------------------------------------------------------*/
    printf("\nDeinitializing CAN controller...\n");
    can_deinit(config.base_addr);
    printf("Done.\n");
    
    return 0;
}
