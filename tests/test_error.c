/**
 * @file    test_error.c
 * @brief   CAN Driver Error Handling Tests
 * @details Unit tests for error handling, statistics, and bus state.
 *          Reference: X_CAN User Manual v3.9, Section 1.5.4.2.2 STAT Register
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
static uint8_t tx_mem[2048] __attribute__((aligned(32)));
static uint8_t rx_mem[2048] __attribute__((aligned(32)));

/* Test configuration */
static can_config_t test_config;

/*----------------------------------------------------------------------------*/
/* Setup/Teardown                                                             */
/*----------------------------------------------------------------------------*/

static int setup_controller(void) {
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
    test_config.tx_fifo_queues[0].size = 8;
    
    test_config.rx_fifo_queues[0].enabled = true;
    test_config.rx_fifo_queues[0].start_addr = (uint32_t)(uintptr_t)rx_mem;
    test_config.rx_fifo_queues[0].size = 8;
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
 * @brief Test can_get_stats returns valid structure
 */
static int test_get_stats(void) {
    can_error_t status;
    can_stats_t stats;
    
    if (setup_controller() != 0) return -1;
    
    memset(&stats, 0xFF, sizeof(stats));  /* Fill with known value */
    status = can_get_stats(test_config.base_addr, &stats);
    
    teardown();
    
    if (status != CAN_ERROR_NONE) return -1;
    
    /* In loopback mode, error counters should be 0 */
    if (stats.tx_error_count != 0) return -1;
    if (stats.rx_error_count != 0) return -1;
    if (stats.bus_off != false) return -1;
    
    return 0;
}

/**
 * @brief Test can_get_stats with NULL pointer
 */
static int test_get_stats_null(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    status = can_get_stats(test_config.base_addr, NULL);
    
    teardown();
    return (status == CAN_ERROR_INVALID_PARAM) ? 0 : -1;
}

/**
 * @brief Test can_get_bus_state returns ACTIVE in loopback
 */
static int test_get_bus_state(void) {
    can_error_t status;
    can_bus_state_t state = CAN_BUS_STATE_BUS_OFF;
    
    if (setup_controller() != 0) return -1;
    
    status = can_get_bus_state(test_config.base_addr, &state);
    
    teardown();
    
    if (status != CAN_ERROR_NONE) return -1;
    if (state != CAN_BUS_STATE_ACTIVE) return -1;
    
    return 0;
}

/**
 * @brief Test can_clear_stats
 */
static int test_clear_stats(void) {
    can_error_t status;
    can_stats_t stats;
    
    if (setup_controller() != 0) return -1;
    
    status = can_clear_stats(test_config.base_addr);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Verify stats are cleared */
    status = can_get_stats(test_config.base_addr, &stats);
    
    teardown();
    
    if (status != CAN_ERROR_NONE) return -1;
    if (stats.tx_success_count != 0) return -1;
    if (stats.rx_success_count != 0) return -1;
    
    return 0;
}

/**
 * @brief Test can_get_version
 */
static int test_get_version(void) {
    can_error_t status;
    uint32_t mh_version = 0, prt_version = 0;
    
    if (setup_controller() != 0) return -1;
    
    status = can_get_version(test_config.base_addr, &mh_version, &prt_version);
    
    teardown();
    
    /* Should succeed (version values depend on hardware) */
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_software_reset
 */
static int test_software_reset(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    status = can_software_reset(test_config.base_addr);
    
    /* After reset, need to reinit */
    teardown();
    
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_set_loopback enable/disable
 */
static int test_loopback_toggle(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    /* Disable loopback */
    status = can_set_loopback(test_config.base_addr, false);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Re-enable loopback */
    status = can_set_loopback(test_config.base_addr, true);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test can_set_listen_only
 */
static int test_listen_only(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    /* Enable listen-only mode */
    status = can_set_listen_only(test_config.base_addr, true);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Disable listen-only mode */
    status = can_set_listen_only(test_config.base_addr, false);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test interrupt enable/disable
 */
static int test_interrupt_enable(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    /* Enable all functional interrupts */
    status = can_interrupt_enable(test_config.base_addr, 0xFFFFFFFF, 0, 0);
    if (status != CAN_ERROR_NONE) {
        teardown();
        return -1;
    }
    
    /* Disable all */
    status = can_interrupt_enable(test_config.base_addr, 0, 0, 0);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test interrupt clear
 */
static int test_interrupt_clear(void) {
    can_error_t status;
    
    if (setup_controller() != 0) return -1;
    
    /* Clear all functional interrupts */
    status = can_interrupt_clear(test_config.base_addr, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/**
 * @brief Test interrupt status read
 */
static int test_interrupt_status(void) {
    can_error_t status;
    uint32_t func_status, err_status, safety_status;
    
    if (setup_controller() != 0) return -1;
    
    status = can_interrupt_get_raw_status(test_config.base_addr,
                                          &func_status, &err_status, &safety_status);
    
    teardown();
    return (status == CAN_ERROR_NONE) ? 0 : -1;
}

/*----------------------------------------------------------------------------*/
/* Main Test Runner                                                           */
/*----------------------------------------------------------------------------*/

int main(void) {
    printf("\n");
    printf("========================================\n");
    printf("CAN Driver Error Handling Tests\n");
    printf("========================================\n\n");
    
    RUN_TEST(test_get_stats);
    RUN_TEST(test_get_stats_null);
    RUN_TEST(test_get_bus_state);
    RUN_TEST(test_clear_stats);
    RUN_TEST(test_get_version);
    RUN_TEST(test_software_reset);
    RUN_TEST(test_loopback_toggle);
    RUN_TEST(test_listen_only);
    RUN_TEST(test_interrupt_enable);
    RUN_TEST(test_interrupt_clear);
    RUN_TEST(test_interrupt_status);
    
    printf("\n========================================\n");
    printf("Results: %u/%u tests passed\n", pass_count, test_count);
    printf("========================================\n\n");
    
    return (fail_count == 0) ? 0 : 1;
}
