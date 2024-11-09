/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <CUnit/Basic.h>
#include <iostream>

// Test startup to PLAN_NEXT transition (T1)
void test_t1() {
    mock_reset_state();
    mock_set_wall(false);
    
    turtleMove result = studentTurtleStep(false, false);
    
    std::cout << "T1 test - Action: " << result.action 
              << ", ValidAction: " << result.validAction << std::endl;
    
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_TRUE(result.validAction);
}

// Test response to wall detection (T2)
void test_t2() {
    mock_reset_state();
    mock_set_wall(true);
    
    // First call to initialize exploration
    turtleMove init_result = studentTurtleStep(false, false);
    
    // Second call with wall detection
    turtleMove result = studentTurtleStep(true, false);
    
    std::cout << "T2 test - Action: " << result.action 
              << ", ValidAction: " << result.validAction
              << ", RobotState: " << robot_state 
              << ", RotationsChecked: " << rotations_checked << std::endl;
    
    // In PLAN_NEXT state with wall detected, should turn RIGHT to explore
    CU_ASSERT_EQUAL(robot_state, PLAN_NEXT);
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_TRUE(result.validAction);
}

// Added coverage test for movement after rotation
void test_t3() {
    mock_reset_state();
    mock_set_wall(false);
    
    // Complete rotation sequence
    for(int i = 0; i < 4; i++) {
        turtleMove result = studentTurtleStep(false, false);
        std::cout << "T3 test rotation " << i << " - Action: " << result.action 
                  << ", RotationsChecked: " << rotations_checked << std::endl;
    }
}

int init() {
    return 0;
}

int cleanup() {
    return 0;
}

int main() {
    CU_pSuite pSuite = NULL;

    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();

    pSuite = CU_add_suite("Suite_1", init, cleanup);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    if ((NULL == CU_add_test(pSuite, "test of transition T1", test_t1)) ||
        (NULL == CU_add_test(pSuite, "test of transition T2", test_t2)) ||
        (NULL == CU_add_test(pSuite, "test of transition T3", test_t3)))
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
