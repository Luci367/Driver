/**
 * @file    test_rx.c
 * @brief   CAN Driver RX API Tests
 * @details Unit tests for RX FIFO functionality.
 *          Reference: X_CAN User Manual v3.9, Section 1.4.7.3-1.4.7.5
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
 * @brief Test can_rx_read with NULL message pointer
 */
static int test_rx_null_msg(void) {
    can_error_t status;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_rx_read(test_config.base_addr, 0, NULL, 0);
    
    teardown();
    return (status == CAN_ERROR_INVALID_PARAM) ? 0 : -1;
}

/**
 * @brief Test can_rx_read with invalid FIFO ID
 */
static int test_rx_invalid_fifo(void) {
    can_error_t status;
    can_msg_t msg;
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* FIFO ID 8 is invalid */
    status = can_rx_read(test_config.base_addr, 8, &msg, 0);
    
    teardown();
    return (status == CAN_ERROR_INVALID_PARAM) ? 0 : -1;
}

/**
 * @brief Test can_rx_read returns QUEUE_EMPTY when no messages
 */
static int test_rx_empty_queue(void) {
    can_error_t status;
    can_msg_t msg;
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* Non-blocking read with no messages */
    status = can_rx_read(test_config.base_addr, 0, &msg, 0);
    
    teardown();
    return (status == CAN_ERROR_QUEUE_EMPTY) ? 0 : -1;
}

/**
 * @brief Test can_rx_read timeout functionality
 */
static int test_rx_timeout(void) {
    can_error_t status;
    can_msg_t msg;
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* Short timeout with no messages */
    status = can_rx_read(test_config.base_addr, 0, &msg, 100);
    
    teardown();
    return (status == CAN_ERROR_TIMEOUT) ? 0 : -1;
}

/**
 * @brief Test can_rx_has_message returns false when empty
 */
static int test_rx_has_message_empty(void) {
    can_error_t status;
    bool has_msg = true;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_rx_has_message(test_config.base_addr, 0, &has_msg);
    
    teardown();
    return (status == CAN_ERROR_NONE && has_msg == false) ? 0 : -1;
}

/**
 * @brief Test can_rx_fifo_is_busy
 */
static int test_rx_is_busy(void) {
    can_error_t status;
    bool is_busy = true;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_rx_fifo_is_busy(test_config.base_addr, 0, &is_busy);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_rx_restart
 */
static int test_rx_restart(void) {
    if (setup_loopback_mode() != 0) return -1;
    
    /* Should not crash or return error */
    can_rx_restart(test_config.base_addr, 0);
    
    teardown();
    return 0;  /* Success if no crash */
}

/**
 * @brief Test can_rx_abort
 */
static int test_rx_abort(void) {
    can_error_t status;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_rx_abort(test_config.base_addr, 0);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test loopback TX/RX functionality
 */
static int test_loopback_tx_rx(void) {
    can_error_t status;
    can_msg_t rx_msg;
    uint8_t tx_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    
    if (setup_loopback_mode() != 0) return -1;
    
    /* Transmit */
    status = can_tx_fifo_push(test_config.base_addr, 0, 0x123,
                              tx_data, 8, false, false, false);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Receive (with timeout for loopback) */
    status = can_rx_read(test_config.base_addr, 0, &rx_msg, 10000);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Verify */
    if (rx_msg.id != 0x123) {
        teardown();
        return -1;
    }
    
    if (rx_msg.len != 8) {
        teardown();
        return -1;
    }
    
    if (memcmp(rx_msg.data, tx_data, 8) != 0) {
        teardown();
        return -1;
    }
    
    teardown();
    return 0;
}

/**
 * @brief Test can_rx_get_fill_level
 */
static int test_rx_fill_level(void) {
    can_error_t status;
    uint32_t fill_level = 999;
    
    if (setup_loopback_mode() != 0) return -1;
    
    status = can_rx_get_fill_level(test_config.base_addr, 0, &fill_level);
    
    teardown();
    
    /* Should succeed with zero fill level (empty queue) */
    return (status == CAN_ERROR_NONE && fill_level == 0) ? 0 : -1;
}

/*----------------------------------------------------------------------------*/
/* Main Test Runner                                                           */
/*----------------------------------------------------------------------------*/

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("CAN Driver RX API Tests\n");
    printf("========================================\n\n");
    
    RUN_TEST(test_rx_null_msg);
    RUN_TEST(test_rx_invalid_fifo);
    RUN_TEST(test_rx_empty_queue);
    RUN_TEST(test_rx_timeout);
    RUN_TEST(test_rx_has_message_empty);
    RUN_TEST(test_rx_is_busy);
    RUN_TEST(test_rx_restart);
    RUN_TEST(test_rx_abort);
    RUN_TEST(test_loopback_tx_rx);
    RUN_TEST(test_rx_fill_level);
    
    printf("\n========================================\n");
    printf("Results: %u/%u tests passed\n", pass_count, test_count);
    printf("========================================\n\n");
    
    return (fail_count == 0) ? 0 : 1;
}
