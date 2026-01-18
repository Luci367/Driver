/**
 * @file    test_init.c
 * @brief   CAN Driver Initialization Tests
 * @details Unit tests for can_init() and related functions.
 *          Reference: X_CAN User Manual v3.9, Section 1.4.7.1
 */

#include "../can_driver.h"
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------*/
/* Test Infrastructure                                                        */
/*----------------------------------------------------------------------------*/

static uint32_t test_count = 0;
static uint32_t pass_count = 0;
static uint32_t fail_count = 0;

#define RUN_TEST(test_func) do { \
    test_count++; \
    printf("Running %s... ", #test_func); \
    if (test_func() == 0) { \
        printf("PASS\n"); \
        pass_count++; \
    } else { \
        printf("FAIL\n"); \
        fail_count++; \
    } \
} while(0)

/* Mock memory buffers */
static uint8_t tx_mem[1024] __attribute__((aligned(32)));
static uint8_t rx_mem[2048] __attribute__((aligned(32)));

/*----------------------------------------------------------------------------*/
/* Test Cases                                                                 */
/*----------------------------------------------------------------------------*/

/**
 * @brief Test that can_init rejects NULL config
 */
static int test_init_null_config(void) {
    can_error_t status = can_init(NULL);
    return (status == CAN_ERROR_INVALID_PARAM) ? 0 : -1;
}

/**
 * @brief Test can_init with valid minimal configuration
 */
static int test_init_minimal_config(void) {
    can_config_t config;
    can_error_t status;
    
    memset(&config, 0, sizeof(config));
    config.base_addr = CAN_BASE;
    config.lmem_base_addr = CAN_BASE + 0x10000;
    config.protocol = CAN_PROTOCOL_CC;
    
    /* Minimal bit timing */
    config.nominal_timing.brp = 0;
    config.nominal_timing.tseg1 = 15;
    config.nominal_timing.tseg2 = 4;
    config.nominal_timing.sjw = 4;
    
    /* Enable loopback for testing without hardware */
    config.loopback_enable = true;
    
    status = can_init(&config);
    
    if (status == CAN_ERROR_NONE) {
        can_deinit(config.base_addr);
        return 0;
    }
    
    return -1;
}

/**
 * @brief Test can_init with all queues enabled
 */
static int test_init_all_queues(void) {
    can_config_t config;
    can_error_t status;
    
    memset(&config, 0, sizeof(config));
    config.base_addr = CAN_BASE;
    config.lmem_base_addr = CAN_BASE + 0x10000;
    config.protocol = CAN_PROTOCOL_FD;
    config.loopback_enable = true;
    
    config.nominal_timing.brp = 0;
    config.nominal_timing.tseg1 = 15;
    config.nominal_timing.tseg2 = 4;
    config.nominal_timing.sjw = 4;
    config.data_timing = config.nominal_timing;
    
    /* Enable all TX FIFO queues */
    for (int i = 0; i < CAN_TX_FIFO_QUEUE_COUNT; i++) {
        config.tx_fifo_queues[i].enabled = true;
        config.tx_fifo_queues[i].start_addr = (uint32_t)(uintptr_t)tx_mem + (i * 128);
        config.tx_fifo_queues[i].size = 4;
    }
    
    /* Enable all RX FIFO queues */
    for (int i = 0; i < CAN_RX_FIFO_QUEUE_COUNT; i++) {
        config.rx_fifo_queues[i].enabled = true;
        config.rx_fifo_queues[i].start_addr = (uint32_t)(uintptr_t)rx_mem + (i * 256);
        config.rx_fifo_queues[i].size = 8;
        config.rx_fifo_queues[i].dc_size = 2;
    }
    
    status = can_init(&config);
    
    if (status == CAN_ERROR_NONE) {
        can_deinit(config.base_addr);
        return 0;
    }
    
    return -1;
}

/**
 * @brief Test that can_deinit properly stops the controller
 */
static int test_deinit(void) {
    can_config_t config;
    can_error_t status;
    
    memset(&config, 0, sizeof(config));
    config.base_addr = CAN_BASE;
    config.lmem_base_addr = CAN_BASE + 0x10000;
    config.protocol = CAN_PROTOCOL_CC;
    config.loopback_enable = true;
    
    config.nominal_timing.brp = 0;
    config.nominal_timing.tseg1 = 15;
    config.nominal_timing.tseg2 = 4;
    config.nominal_timing.sjw = 4;
    
    status = can_init(&config);
    if (status != CAN_ERROR_NONE) {
        return -1;
    }
    
    status = can_deinit(config.base_addr);
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test software reset function
 */
static int test_software_reset(void) {
    can_error_t status;
    
    /* Reset should work even when not initialized */
    status = can_software_reset(CAN_BASE);
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test protocol mode configurations
 */
static int test_protocol_modes(void) {
    can_config_t config;
    can_error_t status;
    can_protocol_t protocols[] = {CAN_PROTOCOL_CC, CAN_PROTOCOL_FD, CAN_PROTOCOL_XL};
    
    for (int p = 0; p < 3; p++) {
        memset(&config, 0, sizeof(config));
        config.base_addr = CAN_BASE;
        config.lmem_base_addr = CAN_BASE + 0x10000;
        config.protocol = protocols[p];
        config.loopback_enable = true;
        
        config.nominal_timing.brp = 0;
        config.nominal_timing.tseg1 = 15;
        config.nominal_timing.tseg2 = 4;
        config.nominal_timing.sjw = 4;
        
        if (protocols[p] == CAN_PROTOCOL_FD || protocols[p] == CAN_PROTOCOL_XL) {
            config.data_timing = config.nominal_timing;
        }
        
        if (protocols[p] == CAN_PROTOCOL_XL) {
            config.xl_timing = config.nominal_timing;
        }
        
        status = can_init(&config);
        if (status != CAN_ERROR_NONE) {
            return -1;
        }
        
        can_deinit(config.base_addr);
    }
    
    return 0;
}

/*----------------------------------------------------------------------------*/
/* Main Test Runner                                                           */
/*----------------------------------------------------------------------------*/

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("CAN Driver Initialization Tests\n");
    printf("========================================\n\n");
    
    RUN_TEST(test_init_null_config);
    RUN_TEST(test_init_minimal_config);
    RUN_TEST(test_init_all_queues);
    RUN_TEST(test_deinit);
    RUN_TEST(test_software_reset);
    RUN_TEST(test_protocol_modes);
    
    printf("\n========================================\n");
    printf("Results: %u/%u tests passed\n", pass_count, test_count);
    printf("========================================\n\n");
    
    return (fail_count == 0) ? 0 : 1;
}
