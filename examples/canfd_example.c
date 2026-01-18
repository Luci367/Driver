/**
 * @file    canfd_example.c
 * @brief   CAN FD High-Speed Example
 * @details Demonstrates CAN FD operation with up to 64 bytes payload
 *          and flexible data rate switching (BRS).
 *          Reference: X_CAN User Manual v3.9, Section 1.5.1.2 CAN FD
 */

#include "../can_driver.h"
#include <stdio.h>
#include <string.h>

/* Memory buffers */
static uint8_t tx_queue_mem[2048] __attribute__((aligned(32)));
static uint8_t rx_queue_mem[2048] __attribute__((aligned(32)));

/**
 * @brief DLC values for CAN FD and their corresponding byte lengths
 */
static void print_dlc_table(void) {
    printf("\n--- CAN FD DLC to Length Mapping ---\n");
    printf("\n| DLC | Length (bytes) |\n");
    printf("|-----|----------------|\n");
    printf("|  0  |       0        |\n");
    printf("|  1  |       1        |\n");
    printf("|  2  |       2        |\n");
    printf("|  3  |       3        |\n");
    printf("|  4  |       4        |\n");
    printf("|  5  |       5        |\n");
    printf("|  6  |       6        |\n");
    printf("|  7  |       7        |\n");
    printf("|  8  |       8        |\n");
    printf("|  9  |      12        |\n");
    printf("| 10  |      16        |\n");
    printf("| 11  |      20        |\n");
    printf("| 12  |      24        |\n");
    printf("| 13  |      32        |\n");
    printf("| 14  |      48        |\n");
    printf("| 15  |      64        |\n");
    printf("\n");
}

/**
 * @brief Initialize CAN controller for CAN FD operation
 */
static can_error_t init_canfd(can_config_t *config) {
    memset(config, 0, sizeof(*config));
    
    config->base_addr = CAN_BASE;
    config->lmem_base_addr = CAN_BASE + 0x10000;
    config->protocol = CAN_PROTOCOL_FD;
    config->mode = CAN_MODE_NORMAL;
    
    /*------------------------------------------------------------------------*/
    /* Bit Timing Configuration                                               */
    /* Arbitration Phase: 500 kbit/s (CAN CC compatible)                      */
    /* Data Phase: 2 Mbit/s (CAN FD high-speed)                               */
    /* Assuming 80 MHz CAN clock                                              */
    /*------------------------------------------------------------------------*/
    
    /* Nominal timing: 500 kbit/s, 80% sample point */
    config->nominal_timing.brp = 0;     /* Prescaler = 1 */
    config->nominal_timing.tseg1 = 127; /* 128 Tq */
    config->nominal_timing.tseg2 = 32;  /* 32 Tq */
    config->nominal_timing.sjw = 16;    /* Max jump */
    /* Total: 160 Tq -> 80MHz/160 = 500 kbit/s */
    /* Sample point: (1 + 127) / 160 = 80% */
    
    /* Data phase timing: 2 Mbit/s, 80% sample point */
    config->data_timing.brp = 0;
    config->data_timing.tseg1 = 31;     /* 32 Tq */
    config->data_timing.tseg2 = 8;      /* 8 Tq */
    config->data_timing.sjw = 8;
    config->data_timing.tdco = 16;      /* TDC offset */
    /* Total: 40 Tq -> 80MHz/40 = 2 Mbit/s */
    /* Sample point: (1 + 31) / 40 = 80% */
    
    /*------------------------------------------------------------------------*/
    /* Queue Configuration                                                    */
    /* DC_SIZE = 2 -> 64 bytes (sufficient for CAN FD max payload)            */
    /*------------------------------------------------------------------------*/
    
    config->tx_fifo_queues[0].enabled = true;
    config->tx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)tx_queue_mem;
    config->tx_fifo_queues[0].size = 16;
    
    config->rx_fifo_queues[0].enabled = true;
    config->rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_queue_mem;
    config->rx_fifo_queues[0].size = 16;
    config->rx_fifo_queues[0].dc_size = 2;  /* 64 bytes */
    
    config->func_int_enable = 0x000001FF;
    
    return can_init(config);
}

/**
 * @brief Example: Send CAN FD message with Bit Rate Switch
 */
static int example_fd_with_brs(uint32_t base_addr) {
    can_error_t status;
    uint8_t data[64];
    
    printf("\n--- CAN FD with BRS Example ---\n");
    printf("Bit Rate Switch (BRS) enables higher data rate for the data phase.\n");
    printf("Arbitration: 500 kbit/s, Data: 2 Mbit/s\n\n");
    
    /* Fill with test pattern */
    for (int i = 0; i < 64; i++) {
        data[i] = (uint8_t)(i * 2);
    }
    
    printf("Sending 64-byte CAN FD message with BRS...\n");
    printf("  ID: 0x100\n");
    printf("  Length: 64 bytes\n");
    printf("  BRS: Enabled\n");
    
    /* Use extended API for BRS control */
    status = can_tx_fifo_push_ext(base_addr, 0, 0x100,
                                  false,  /* Standard ID */
                                  data, 64,
                                  true,   /* FD */
                                  false,  /* Not XL */
                                  true,   /* BRS enabled */
                                  false); /* Not remote */
    
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: TX failed with code %d\n", status);
        return -1;
    }
    
    printf("Message sent successfully with BRS!\n");
    return 0;
}

/**
 * @brief Example: Send various FD payload sizes
 */
static int example_fd_payloads(uint32_t base_addr) {
    can_error_t status;
    uint8_t data[64];
    
    /* Common FD payload sizes */
    const uint32_t sizes[] = {8, 12, 16, 20, 24, 32, 48, 64};
    const int num_sizes = 8;
    
    printf("\n--- CAN FD Payload Size Examples ---\n");
    printf("Demonstrating all CAN FD DLC values > 8...\n\n");
    
    for (int i = 0; i < num_sizes; i++) {
        uint32_t len = sizes[i];
        
        /* Fill data pattern */
        for (uint32_t j = 0; j < len; j++) {
            data[j] = (uint8_t)(len + j);
        }
        
        printf("Sending %2u bytes (ID=0x%03X)... ", len, 0x200 + i);
        
        status = can_tx_fifo_push(base_addr, 0, 0x200 + i, data, len,
                                  true, false, false);
        
        if (status != CAN_ERROR_NONE) {
            printf("FAILED (code %d)\n", status);
        } else {
            printf("OK\n");
        }
    }
    
    return 0;
}

/**
 * @brief Example: Error State Indicator (ESI)
 */
static void explain_esi(void) {
    printf("\n--- Error State Indicator (ESI) Explanation ---\n");
    printf("\n");
    printf("The ESI bit in CAN FD frames indicates the error state of\n");
    printf("the transmitting node:\n");
    printf("\n");
    printf("  ESI = 0 (Dominant): Transmitter is ERROR ACTIVE\n");
    printf("    - Normal operation, can transmit active error flags\n");
    printf("    - TEC and REC are both < 128\n");
    printf("\n");
    printf("  ESI = 1 (Recessive): Transmitter is ERROR PASSIVE\n");
    printf("    - Degraded operation, can only send passive error flags\n");
    printf("    - TEC or REC >= 128 (but not BUS OFF)\n");
    printf("\n");
    printf("ESI allows receivers to know the health status of the transmitter.\n");
    printf("This is useful for diagnostic and safety applications.\n");
}

/**
 * @brief Example: CAN FD vs Classical CAN comparison
 */
static void compare_fd_vs_cc(void) {
    printf("\n--- CAN FD vs Classical CAN Comparison ---\n");
    printf("\n");
    printf("| Feature           | Classical CAN | CAN FD          |\n");
    printf("|-------------------|---------------|------------------|\n");
    printf("| Max Data Length   | 8 bytes       | 64 bytes        |\n");
    printf("| Max Bit Rate      | 1 Mbit/s      | 8 Mbit/s (data) |\n");
    printf("| Bit Rate Switch   | No            | Yes (BRS)       |\n");
    printf("| Error Indicator   | No            | Yes (ESI)       |\n");
    printf("| CRC Length        | 15 bits       | 17/21 bits      |\n");
    printf("| Stuff Bit Count   | No            | Yes (SBC)       |\n");
    printf("| Backward Compat.  | N/A           | Yes (arb phase) |\n");
    printf("\n");
    printf("CAN FD is fully backward compatible with Classical CAN nodes\n");
    printf("during the arbitration phase. Only nodes that win arbitration\n");
    printf("and switch to the data phase need CAN FD capability.\n");
}

int main(void) {
    can_config_t config;
    can_error_t status;
    
    printf("================================================\n");
    printf("CAN FD Driver Example\n");
    printf("================================================\n");
    printf("Demonstrates CAN FD operation with high data rates\n");
    printf("Based on X_CAN IP v3.9 User Manual\n");
    printf("================================================\n");
    
    /* Print informational tables */
    print_dlc_table();
    compare_fd_vs_cc();
    explain_esi();
    
    /* Initialize for CAN FD */
    printf("\n--- Initialization ---\n");
    printf("Initializing CAN controller for CAN FD...\n");
    
    status = init_canfd(&config);
    if (status != CAN_ERROR_NONE) {
        printf("ERROR: Initialization failed with code %d\n", status);
        return -1;
    }
    printf("CAN FD mode initialized successfully.\n");
    printf("  Nominal rate: 500 kbit/s\n");
    printf("  Data rate: 2 Mbit/s\n");
    
    /* Run examples */
    (void)example_fd_with_brs(config.base_addr);
    (void)example_fd_payloads(config.base_addr);
    
    /* Cleanup */
    printf("\n--- Cleanup ---\n");
    can_deinit(config.base_addr);
    printf("CAN controller deinitialized.\n");
    
    printf("\n================================================\n");
    printf("Example complete.\n");
    printf("================================================\n");
    
    return 0;
}
