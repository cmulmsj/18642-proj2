/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <CUnit/Basic.h>

// Test transition from STARTUP to PLAN_NEXT (T1)
void test_t1() {
    // Initialize state
    robot_state = STARTUP;
    first_run = true;
    mock_set_visit_count(0);
    
    // Execute transition
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify state transition and outputs
    CU_ASSERT_EQUAL(robot_state, PLAN_NEXT);
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_TRUE(result.validAction);
}

// Test transition from PLAN_NEXT to turning state (T2)
void test_t2() {
    // Initialize state
    robot_state = PLAN_NEXT;
    rotations_checked = 0;
    mock_set_visit_count(5);  // Set a relatively high visit count
    mock_set_wall(true);      // Set wall detection
    
    // Execute transition
    turtleMove result = studentTurtleStep(true, false);
    
    // Verify state transition and outputs
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_TRUE(result.validAction);
    CU_ASSERT_EQUAL(rotations_checked, 1);
}

int init() {
    return 0;
}

int cleanup() {
    return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {
    CU_pSuite pSuite = NULL;

    /* initialize the CUnit test registry */
    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();

    /* add a suite to the registry */
    pSuite = CU_add_suite("Suite_1", init, cleanup);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* add the tests to the suite */
    if ((NULL == CU_add_test(pSuite, "test of transition T1", test_t1)) ||
        (NULL == CU_add_test(pSuite, "test of transition T2", test_t2)))
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* Run all tests using the CUnit Basic interface */
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
