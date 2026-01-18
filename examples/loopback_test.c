/**
 * @file    loopback_test.c
 * @brief   CAN Loopback Self-Test Example
 * @details Tests TX/RX functionality using internal loopback mode.
 *          No external hardware or bus connection required.
 *          Reference: X_CAN User Manual v3.9, Section 1.5.4.2.3.4 TEST Register
 */

#include "../can_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Test memory buffers */
static uint8_t tx_queue_mem[4096] __attribute__((aligned(32)));
static uint8_t rx_queue_mem[4096] __attribute__((aligned(32)));

/* Test result counters */
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;

/*----------------------------------------------------------------------------*/
/* Test Helper Macros                                                         */
/*----------------------------------------------------------------------------*/

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s\n", msg); \
        tests_failed++; \
        return -1; \
    } \
} while(0)

#define TEST_PASS(msg) do { \
    printf("  PASS: %s\n", msg); \
    tests_passed++; \
} while(0)

/*----------------------------------------------------------------------------*/
/* Test Functions                                                             */
/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize CAN controller in loopback mode
 */
static int setup_loopback(can_config_t *config) {
    can_error_t status;
    
    memset(config, 0, sizeof(*config));
    
    config->base_addr = CAN_BASE;
    config->lmem_base_addr = CAN_BASE + 0x10000;
    config->protocol = CAN_PROTOCOL_FD;
    config->loopback_enable = true;
    
    /* Minimal bit timing for loopback (timing not critical in loopback) */
    config->nominal_timing.brp = 0;
    config->nominal_timing.tseg1 = 15;
    config->nominal_timing.tseg2 = 4;
    config->nominal_timing.sjw = 4;
    
    config->data_timing = config->nominal_timing;
    
    /* Configure TX Queue 0 */
    config->tx_fifo_queues[0].enabled = true;
    config->tx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)tx_queue_mem;
    config->tx_fifo_queues[0].size = 32;
    
    /* Configure RX Queue 0 */
    config->rx_fifo_queues[0].enabled = true;
    config->rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_queue_mem;
    config->rx_fifo_queues[0].size = 32;
    config->rx_fifo_queues[0].dc_size = 2;  /* 64 bytes */
    
    /* Initialize */
    status = can_init(config);
    TEST_ASSERT(status == CAN_ERROR_NONE, "can_init() failed");
    
    /* Enable loopback mode */
    status = can_set_loopback(config->base_addr, true);
    TEST_ASSERT(status == CAN_ERROR_NONE, "can_set_loopback() failed");
    
    return 0;
}

/**
 * @brief Test 1: Basic Classical CAN frame
 */
static int test_classical_can(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    uint8_t tx_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    
    printf("\nTest 1: Classical CAN Frame (8 bytes)\n");
    
    /* Transmit */
    status = can_tx_fifo_push(base_addr, 0, 0x123, tx_data, 8,
                              false, false, false);
    TEST_ASSERT(status == CAN_ERROR_NONE, "TX failed");
    
    /* Receive */
    status = can_rx_read(base_addr, 0, &rx_msg, 10000);
    TEST_ASSERT(status == CAN_ERROR_NONE, "RX failed");
    
    /* Verify */
    TEST_ASSERT(rx_msg.id == 0x123, "ID mismatch");
    TEST_ASSERT(rx_msg.len == 8, "Length mismatch");
    TEST_ASSERT(rx_msg.fd == false, "FD flag should be false");
    TEST_ASSERT(rx_msg.xl == false, "XL flag should be false");
    TEST_ASSERT(memcmp(rx_msg.data, tx_data, 8) == 0, "Data mismatch");
    
    TEST_PASS("Classical CAN frame");
    return 0;
}

/**
 * @brief Test 2: CAN FD frame with 64 bytes
 */
static int test_canfd_64bytes(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    uint8_t tx_data[64];
    
    printf("\nTest 2: CAN FD Frame (64 bytes)\n");
    
    /* Generate test data */
    for (int i = 0; i < 64; i++) {
        tx_data[i] = (uint8_t)i;
    }
    
    /* Transmit FD frame */
    status = can_tx_fifo_push(base_addr, 0, 0x456, tx_data, 64,
                              true,   /* FD */
                              false,  /* Not XL */
                              false); /* Not remote */
    TEST_ASSERT(status == CAN_ERROR_NONE, "TX failed");
    
    /* Receive */
    status = can_rx_read(base_addr, 0, &rx_msg, 10000);
    TEST_ASSERT(status == CAN_ERROR_NONE, "RX failed");
    
    /* Verify */
    TEST_ASSERT(rx_msg.id == 0x456, "ID mismatch");
    TEST_ASSERT(rx_msg.len == 64, "Length mismatch");
    TEST_ASSERT(rx_msg.fd == true, "FD flag should be true");
    TEST_ASSERT(memcmp(rx_msg.data, tx_data, 64) == 0, "Data mismatch");
    
    TEST_PASS("CAN FD 64-byte frame");
    return 0;
}

/**
 * @brief Test 3: Extended ID (29-bit)
 */
static int test_extended_id(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    uint8_t tx_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint32_t ext_id = 0x18FEF100;  /* 29-bit ID */
    
    printf("\nTest 3: Extended ID (29-bit)\n");
    
    /* Transmit with extended ID */
    status = can_tx_fifo_push_ext(base_addr, 0, ext_id, true, tx_data, 4,
                                  false, false, false, false);
    TEST_ASSERT(status == CAN_ERROR_NONE, "TX failed");
    
    /* Receive */
    status = can_rx_read(base_addr, 0, &rx_msg, 10000);
    TEST_ASSERT(status == CAN_ERROR_NONE, "RX failed");
    
    /* Verify */
    TEST_ASSERT(rx_msg.id == ext_id, "Extended ID mismatch");
    TEST_ASSERT(rx_msg.extended == true, "Extended flag should be true");
    TEST_ASSERT(rx_msg.len == 4, "Length mismatch");
    
    TEST_PASS("Extended ID frame");
    return 0;
}

/**
 * @brief Test 4: Multiple messages in sequence
 */
static int test_sequence(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    uint8_t data[8];
    
    printf("\nTest 4: Message Sequence (10 messages)\n");
    
    /* Send 10 messages */
    for (int i = 0; i < 10; i++) {
        memset(data, i, sizeof(data));
        status = can_tx_fifo_push(base_addr, 0, 0x100 + i, data, 8,
                                  false, false, false);
        TEST_ASSERT(status == CAN_ERROR_NONE, "TX failed");
    }
    
    /* Receive 10 messages */
    for (int i = 0; i < 10; i++) {
        status = can_rx_read(base_addr, 0, &rx_msg, 10000);
        TEST_ASSERT(status == CAN_ERROR_NONE, "RX failed");
        TEST_ASSERT(rx_msg.id == (uint32_t)(0x100 + i), "ID mismatch in sequence");
        TEST_ASSERT(rx_msg.data[0] == (uint8_t)i, "Data mismatch in sequence");
    }
    
    TEST_PASS("10-message sequence");
    return 0;
}

/**
 * @brief Test 5: Error counters and statistics
 */
static int test_statistics(uint32_t base_addr) {
    can_stats_t stats;
    can_error_t status;
    
    printf("\nTest 5: Statistics\n");
    
    status = can_get_stats(base_addr, &stats);
    TEST_ASSERT(status == CAN_ERROR_NONE, "can_get_stats() failed");
    
    /* In loopback mode, error counters should be 0 */
    TEST_ASSERT(stats.tx_error_count == 0, "TEC should be 0 in loopback");
    TEST_ASSERT(stats.rx_error_count == 0, "REC should be 0 in loopback");
    TEST_ASSERT(stats.bus_state == CAN_BUS_STATE_ACTIVE, "Should be ACTIVE");
    TEST_ASSERT(stats.bus_off == false, "Should not be in bus-off");
    
    printf("  TX Success Count: %u\n", stats.tx_success_count);
    printf("  RX Success Count: %u\n", stats.rx_success_count);
    
    TEST_PASS("Statistics read");
    return 0;
}

/**
 * @brief Test 6: Abort functionality
 */
static int test_abort(uint32_t base_addr) {
    can_error_t status;
    bool is_busy;
    
    printf("\nTest 6: TX Abort\n");
    
    /* Queue a message */
    uint8_t data[] = {1, 2, 3, 4};
    status = can_tx_fifo_push(base_addr, 0, 0x789, data, 4, false, false, false);
    TEST_ASSERT(status == CAN_ERROR_NONE, "TX failed");
    
    /* Abort the queue */
    status = can_tx_abort(base_addr, 0);
    TEST_ASSERT(status == CAN_ERROR_NONE, "Abort failed");
    
    /* Check queue is no longer busy */
    can_tx_fifo_is_busy(base_addr, 0, &is_busy);
    TEST_ASSERT(is_busy == false, "Queue should not be busy after abort");
    
    TEST_PASS("TX abort");
    return 0;
}

/**
 * @brief Test 7: Non-blocking RX (queue empty)
 */
static int test_nonblocking_rx(uint32_t base_addr) {
    can_msg_t rx_msg;
    can_error_t status;
    
    printf("\nTest 7: Non-blocking RX (empty queue)\n");
    
    /* Try to read with no messages pending */
    status = can_rx_read(base_addr, 0, &rx_msg, 0);
    TEST_ASSERT(status == CAN_ERROR_QUEUE_EMPTY, 
                "Should return QUEUE_EMPTY when no message");
    
    TEST_PASS("Non-blocking RX returns QUEUE_EMPTY");
    return 0;
}

/*----------------------------------------------------------------------------*/
/* Main Test Runner                                                           */
/*----------------------------------------------------------------------------*/

int main(void) {
    can_config_t config;
    int result;
    
    printf("========================================\n");
    printf("CAN Driver Loopback Test Suite\n");
    printf("========================================\n");
    printf("This test uses internal loopback mode.\n");
    printf("No external CAN bus connection required.\n");
    printf("========================================\n");
    
    /* Setup loopback mode */
    printf("\nSetting up loopback mode...\n");
    result = setup_loopback(&config);
    if (result != 0) {
        printf("\nFATAL: Failed to initialize loopback mode\n");
        return -1;
    }
    printf("Loopback mode enabled.\n");
    
    /* Run tests */
    test_classical_can(config.base_addr);
    test_canfd_64bytes(config.base_addr);
    test_extended_id(config.base_addr);
    test_sequence(config.base_addr);
    test_statistics(config.base_addr);
    test_abort(config.base_addr);
    test_nonblocking_rx(config.base_addr);
    
    /* Cleanup */
    printf("\n========================================\n");
    printf("Cleaning up...\n");
    can_set_loopback(config.base_addr, false);
    can_deinit(config.base_addr);
    
    /* Results */
    printf("\n========================================\n");
    printf("TEST RESULTS\n");
    printf("========================================\n");
    printf("Passed: %u\n", tests_passed);
    printf("Failed: %u\n", tests_failed);
    printf("========================================\n");
    
    if (tests_failed == 0) {
        printf("\n*** ALL TESTS PASSED ***\n\n");
        return 0;
    } else {
        printf("\n*** SOME TESTS FAILED ***\n\n");
        return -1;
    }
}
