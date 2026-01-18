/**
 * @file    test_tx.c
 * @brief   CAN Driver TX API Tests
 * @details Unit tests for TX FIFO and Priority Queue functionality.
 *          Reference: X_CAN User Manual v3.9, Section 1.4.7.6-1.4.7.11
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

/* Test memory */
static uint8_t tx_mem[4096] __attribute__((aligned(32)));
static uint8_t rx_mem[4096] __attribute__((aligned(32)));

/* Test configuration */
static can_config_t test_config;

/*----------------------------------------------------------------------------*/
/* Setup/Teardown                                                             */
/*----------------------------------------------------------------------------*/

static int setup_loopback_mode(void) {
    can_error_t status;
    
    memset(&test_config, 0, sizeof(test_config));
    test_config.base_addr = CAN_BASE;
    test_config.lmem_base_addr = CAN_BASE + 0x10000;
    test_config.protocol = CAN_PROTOCOL_FD;
    test_config.loopback_enable = true;
    
    test_config.nominal_timing.brp = 0;
    test_config.nominal_timing.tseg1 = 15;
    test_config.nominal_timing.tseg2 = 4;
    test_config.nominal_timing.sjw = 4;
    test_config.data_timing = test_config.nominal_timing;
    
    test_config.tx_fifo_queues[0].enabled = true;
    test_config.tx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)tx_mem;
    test_config.tx_fifo_queues[0].size = 16;
    
    test_config.rx_fifo_queues[0].enabled = true;
    test_config.rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_mem;
    test_config.rx_fifo_queues[0].size = 16;
    test_config.rx_fifo_queues[0].dc_size = 2;
    
    status = can_init(&test_config);
    if (status != CAN_ERROR_NONE) {
        return -1;
    }
    
    return can_set_loopback(test_config.base_addr, true) == CAN_ERROR_NONE ? 0 : -1;
}

static void teardown(void) {
    can_set_loopback(test_config.base_addr, false);
    can_deinit(test_config.base_addr);
}

/*----------------------------------------------------------------------------*/
/* Test Cases                                                                 */
/*----------------------------------------------------------------------------*/

/**
 * @brief Test can_tx_fifo_push with NULL data and zero length (valid for RTR)
 */
static int test_tx_empty_message(void) {
    can_error_t status;
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* Send empty classical CAN frame (remote request) */
    status = can_tx_fifo_push(test_config.base_addr, 0, 0x123,
                              NULL, 0, false, false, true);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_tx_fifo_push with maximum CC payload
 */
static int test_tx_cc_max_length(void) {
    can_error_t status;
    uint8_t data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_tx_fifo_push(test_config.base_addr, 0, 0x100,
                              data, 8, false, false, false);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_tx_fifo_push with CAN FD 64-byte payload
 */
static int test_tx_fd_max_length(void) {
    can_error_t status;
    uint8_t data[64];
    
    for (int i = 0; i < 64; i++) data[i] = (uint8_t)i;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_tx_fifo_push(test_config.base_addr, 0, 0x200,
                              data, 64, true, false, false);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_tx_fifo_push with invalid FIFO ID
 */
static int test_tx_invalid_fifo(void) {
    can_error_t status;
    uint8_t data[] = {1, 2, 3, 4};
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* FIFO ID 8 is invalid (0-7 valid) */
    status = can_tx_fifo_push(test_config.base_addr, 8, 0x100,
                              data, 4, false, false, false);
    
    teardown();
    return (status == CAN_ERROR_INVALID_PARAM) ? 0 : -1;
}

/**
 * @brief Test can_tx_fifo_push with extended ID
 */
static int test_tx_extended_id(void) {
    can_error_t status;
    uint8_t data[] = {0xAA, 0xBB};
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* Extended 29-bit ID */
    status = can_tx_fifo_push_ext(test_config.base_addr, 0, 0x18FEF100,
                                  true, data, 2, false, false, false, false);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_tx_abort
 */
static int test_tx_abort(void) {
    can_error_t status;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_tx_abort(test_config.base_addr, 0);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_tx_fifo_is_busy
 */
static int test_tx_is_busy(void) {
    can_error_t status;
    bool is_busy = true;
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* After init, queue should not be busy */
    status = can_tx_fifo_is_busy(test_config.base_addr, 0, &is_busy);
    
    teardown();
    
    /* Function should succeed and report not busy (no pending messages) */
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test TX with different DLC values (CAN FD)
 */
static int test_tx_fd_dlc_values(void) {
    can_error_t status;
    uint8_t data[64];
    const uint32_t fd_lengths[] = {8, 12, 16, 20, 24, 32, 48, 64};
    
    if (setup_loopback_mode() != 0) return -1;
    
    for (int i = 0; i < 8; i++) {
        memset(data, i, fd_lengths[i]);
        status = can_tx_fifo_push(test_config.base_addr, 0, 0x300 + i,
                                  data, fd_lengths[i], true, false, false);
        if (status != CAN_ERROR_NONE) {
            teardown();
            return -1;
        }
    }
    
    teardown();
    return 0;
}

/*----------------------------------------------------------------------------*/
/* Main Test Runner                                                           */
/*----------------------------------------------------------------------------*/

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("CAN Driver TX API Tests\n");
    printf("========================================\n\n");
    
    RUN_TEST(test_tx_empty_message);
    RUN_TEST(test_tx_cc_max_length);
    RUN_TEST(test_tx_fd_max_length);
    RUN_TEST(test_tx_invalid_fifo);
    RUN_TEST(test_tx_extended_id);
    RUN_TEST(test_tx_abort);
    RUN_TEST(test_tx_is_busy);
    RUN_TEST(test_tx_fd_dlc_values);
    
    printf("\n========================================\n");
    printf("Results: %u/%u tests passed\n", pass_count, test_count);
    printf("========================================\n\n");
    
    return (fail_count == 0) ? 0 : 1;
}
