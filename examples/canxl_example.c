/**
 * @file    canxl_example.c
 * @brief   CAN XL Large Payload Example
 * @details Demonstrates CAN XL operation with payloads up to 2048 bytes.
 *          Reference: X_CAN User Manual v3.9, Section 1.5.1.3 CAN XL
 */

#include "../can_driver.h"
#include <stdio.h>
#include <string.h>

/* Memory buffers for TX/RX queues - must be 32-byte aligned */
static uint8_t tx_queue_mem[8192] __attribute__((aligned(32)));
static uint8_t rx_queue_mem[8192] __attribute__((aligned(32)));

/**
 * @brief Initialize CAN controller for CAN XL operation
 */
static can_error_t init_canxl(can_config_t *config) {
    memset(config, 0, sizeof(*config));
    
    /* Base addresses */
    config->base_addr = CAN_BASE;
    config->lmem_base_addr = CAN_BASE + 0x10000;
    
    /* Select CAN XL protocol */
    config->protocol = CAN_PROTOCOL_XL;
    config->mode = CAN_MODE_NORMAL;
    
    /*------------------------------------------------------------------------*/
    /* Bit Timing Configuration for CAN XL                                    */
    /* Arbitration (Nominal): 500 kbit/s                                      */
    /* Data Phase (FD/XL): 10 Mbit/s                                          */
    /* Assuming 80 MHz CAN clock                                              */
    /*------------------------------------------------------------------------*/
    
    /* Nominal timing: 500 kbit/s */
    config->nominal_timing.brp = 0;     /* Prescaler = 1 */
    config->nominal_timing.tseg1 = 127; /* Time Segment 1 = 128 */
    config->nominal_timing.tseg2 = 32;  /* Time Segment 2 = 32 */
    config->nominal_timing.sjw = 32;    /* SJW = 32 */
    /* Bit time = 160 Tq, Bitrate = 80MHz / 160 = 500 kbit/s */
    
    /* Data phase timing: 10 Mbit/s */
    config->data_timing.brp = 0;
    config->data_timing.tseg1 = 5;
    config->data_timing.tseg2 = 2;
    config->data_timing.sjw = 2;
    config->data_timing.tdco = 4;
    /* Bit time = 8 Tq, Bitrate = 80MHz / 8 = 10 Mbit/s */
    
    /* XL phase timing (same as data for this example) */
    config->xl_timing = config->data_timing;
    
    /*------------------------------------------------------------------------*/
    /* Queue Configuration                                                    */
    /* For CAN XL with 2048 bytes, we need larger data containers             */
    /* DC_SIZE = ceil(2048/32) = 64 (2048 bytes per container)                */
    /*------------------------------------------------------------------------*/
    
    /* TX FIFO Queue 0 */
    config->tx_fifo_queues[0].enabled = true;
    config->tx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)tx_queue_mem;
    config->tx_fifo_queues[0].size = 8;  /* 8 descriptors */
    
    /* RX FIFO Queue 0 - sized for CAN XL */
    config->rx_fifo_queues[0].enabled = true;
    config->rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_queue_mem;
    config->rx_fifo_queues[0].size = 8;
    config->rx_fifo_queues[0].dc_size = 64;  /* 2048 bytes for XL */
    config->rx_fifo_queues[0].continuous = false;
    
    /* Interrupts */
    config->func_int_enable = 0x000001FF;
    
    return can_init(config);
}

/**
 * @brief Example: Send a large CAN XL message
 */
static int example_send_large_message(uint32_t base_addr) {
    can_error_t status;
    uint8_t large_data[2048];
    
    printf("\n--- CAN XL TX Example: 2048 byte message ---\n");
    
    /* Generate test pattern */
    for (int i = 0; i < 2048; i++) {
        large_data[i] = (uint8_t)(i & 0xFF);
    }
    
    printf("Sending CAN XL message:\n");
    printf("  Priority ID: 0x7FF\n");
    printf("  Data length: 2048 bytes\n");
    printf("  Data pattern: Sequential 0x00-0xFF repeating\n");
    
    /* Send as CAN XL frame */
    status = can_tx_fifo_push(base_addr, 0, 0x7FF, large_data, 2048,
                              false,  /* Not FD (XL takes precedence) */
                              true,   /* XL = true */
                              false); /* Not remote */
    
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: TX failed with code %d\n", status);
        return -1;
    }
    
    printf("Message queued successfully!\n");
    return 0;
}

/**
 * @brief Example: Receive a CAN XL message
 */
static int example_receive_xl_message(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    
    printf("\n--- CAN XL RX Example ---\n");
    printf("Waiting for CAN XL message (3 second timeout)...\n");
    
    status = can_rx_read(base_addr, 0, &rx_msg, 3000000);
    
    if (status == CAN_ERROR_TIMEOUT) {
        printf("Timeout - no message received.\n");
        return -1;
    }
    
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: RX failed with code %d\n", status);
        return -1;
    }
    
    printf("\nMessage received:\n");
    printf("  ID: 0x%X\n", rx_msg.id);
    printf("  Length: %u bytes\n", rx_msg.len);
    printf("  Frame type: %s\n", 
           rx_msg.xl ? "CAN XL" : (rx_msg.fd ? "CAN FD" : "Classical CAN"));
    
    if (rx_msg.xl) {
        printf("  VCID: 0x%02X\n", rx_msg.vcid);
        printf("  SDT: 0x%02X\n", rx_msg.sdt);
    }
    
    printf("  Timestamp: %llu\n", (unsigned long long)rx_msg.timestamp);
    
    /* Print first and last 16 bytes of data */
    printf("  Data (first 16 bytes): ");
    for (uint32_t i = 0; i < 16 && i < rx_msg.len; i++) {
        printf("%02X ", rx_msg.data[i]);
    }
    printf("\n");
    
    if (rx_msg.len > 16) {
        printf("  Data (last 16 bytes):  ");
        for (uint32_t i = rx_msg.len - 16; i < rx_msg.len; i++) {
            printf("%02X ", rx_msg.data[i]);
        }
        printf("\n");
    }
    
    return 0;
}

/**
 * @brief Example: Mixed protocol messages
 */
static int example_mixed_protocols(uint32_t base_addr) {
    can_error_t status;
    uint8_t data_cc[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint8_t data_fd[64];
    uint8_t data_xl[256];  /* Smaller XL for demo */
    
    printf("\n--- Mixed Protocol Example ---\n");
    
    /* Classical CAN message */
    printf("Sending Classical CAN (8 bytes)...\n");
    status = can_tx_fifo_push(base_addr, 0, 0x100, data_cc, 8,
                              false, false, false);
    if (status != CAN_ERROR_NONE) {
        printf("  ERROR: CC TX failed\n");
        return -1;
    }
    printf("  OK\n");
    
    /* CAN FD message */
    for (int i = 0; i < 64; i++) data_fd[i] = (uint8_t)(i * 2);
    printf("Sending CAN FD (64 bytes)...\n");
    status = can_tx_fifo_push(base_addr, 0, 0x200, data_fd, 64,
                              true, false, false);
    if (status != CAN_ERROR_NONE) {
        printf("  ERROR: FD TX failed\n");
        return -1;
    }
    printf("  OK\n");
    
    /* CAN XL message */
    for (int i = 0; i < 256; i++) data_xl[i] = (uint8_t)(i * 3);
    printf("Sending CAN XL (256 bytes)...\n");
    status = can_tx_fifo_push(base_addr, 0, 0x300, data_xl, 256,
                              false, true, false);
    if (status != CAN_ERROR_NONE) {
        printf("  ERROR: XL TX failed\n");
        return -1;
    }
    printf("  OK\n");
    
    printf("All protocol variants sent successfully!\n");
    return 0;
}

/**
 * @brief Calculate data throughput
 */
static void print_throughput_info(void) {
    printf("\n--- CAN XL Throughput Information ---\n");
    printf("\nMaximum theoretical throughput at 10 Mbit/s data rate:\n");
    printf("\n");
    printf("| Protocol | Max Data | Overhead | Approx. Efficiency |\n");
    printf("|----------|----------|----------|--------------------|\n");
    printf("| CAN CC   | 8 bytes  | ~47 bits | ~57%%               |\n");
    printf("| CAN FD   | 64 bytes | ~67 bits | ~88%%               |\n");
    printf("| CAN XL   | 2048 B   | ~67 bits | ~99%%               |\n");
    printf("\n");
    printf("CAN XL advantages:\n");
    printf("  - 2048 bytes vs 64 bytes (32x more data per frame)\n");
    printf("  - Virtual CAN network support (VCID)\n");
    printf("  - SDU Type field for application protocols\n");
    printf("  - Higher efficiency for large data transfers\n");
}

int main(void) {
    can_config_t config;
    can_error_t status;
    
    printf("================================================\n");
    printf("CAN XL Driver Example\n");
    printf("================================================\n");
    printf("Demonstrates CAN XL operation with large payloads\n");
    printf("Based on X_CAN IP v3.9 User Manual\n");
    printf("================================================\n");
    
    /* Print throughput information */
    print_throughput_info();
    
    /* Initialize for CAN XL */
    printf("\nInitializing CAN controller for CAN XL...\n");
    status = init_canxl(&config);
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: Initialization failed with code %d\n", status);
        return -1;
    }
    printf("CAN XL mode initialized successfully.\n");
    
    /* Run examples */
    (void)example_send_large_message(config.base_addr);
    (void)example_receive_xl_message(config.base_addr);
    (void)example_mixed_protocols(config.base_addr);
    
    /* Cleanup */
    printf("\nCleaning up...\n");
    can_deinit(config.base_addr);
    printf("Done.\n");
    
    return 0;
}
