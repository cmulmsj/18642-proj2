#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "student_mock.h"

// Test cases following state chart transitions
void test_t1(void) {
    initialize_test();
    
    // Setup test state
    TurtleTestState state = {
        .current_state = STATE_FORWARD,
        .current_direction = L_NORTH,
        .current_location = {14, 14},
        .is_bumped = false,
        .is_at_end = false,
        .timeout_counter = 0,
        .directions_checked = 0
    };
    set_test_state(state);
    
    // Get initial move
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify initial forward movement
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
    
    // Verify timeout behavior
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    cleanup_test();
}

// Test Forward to Forward transition
void test_t2(void) {
    initialize_test();
    
    // Setup for continuous forward movement
    TurtleTestState state = {
        .current_state = STATE_FORWARD,
        .current_direction = L_NORTH,
        .current_location = {14, 14},
        .is_bumped = false,
        .is_at_end = false,
        .timeout_counter = 0,
        .directions_checked = 0
    };
    set_test_state(state);
    
    // First move
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    
    // Wait for timeout
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Second forward move
    result = studentTurtleStep(false, false);
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
    
    cleanup_test();
}

// Add more test cases...

int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("Student Turtle Test Suite", init_suite, clean_suite);
    
    CU_add_test(suite, "Test T1: Init to Forward", test_t1);
    CU_add_test(suite, "Test T2: Forward to Forward", test_t2);
    // Add more tests...
    
    CU_basic_run_tests();
    CU_cleanup_registry();
    
    return 0;
}
