/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <CUnit/Basic.h>
#include <iostream>

// Existing tests remain...

// Data coverage tests
void test_at_goal() {
    mock_reset_state();
    turtleMove result = studentTurtleStep(false, true);
    CU_ASSERT_FALSE(result.validAction);
}

void test_no_wall_forward() {
    mock_reset_state();
    mock_set_wall(false);
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_EQUAL(result.action, FORWARD);
}

void test_wall_rotation() {
    mock_reset_state();
    mock_set_wall(true);
    // Complete rotation sequence with wall
    for(int i = 0; i < 4; i++) {
        turtleMove result = studentTurtleStep(true, false);
        CU_ASSERT(result.action == RIGHT || result.action == LEFT);
    }
}

void test_visit_count_decision() {
    mock_reset_state();
    // Set different visit counts
    coordinate pos = {START_POS, START_POS};
    updateVisitMap(pos);
    updateVisitMap(pos);  // Current position visited twice
    
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_TRUE(result.validAction);
}

// Branch coverage tests
void test_all_directions() {
    mock_reset_state();
    // Test all four facing directions
    for(int dir = 0; dir < 4; dir++) {
        facing_direction = dir;
        turtleMove result = studentTurtleStep(false, false);
        CU_ASSERT_TRUE(result.validAction);
    }
}

void test_grid_boundaries() {
    mock_reset_state();
    // Test behavior at grid edges
    current_pos = {0, 0};  // Top-left corner
    turtleMove result1 = studentTurtleStep(false, false);
    
    current_pos = {GRID_SIZE-1, GRID_SIZE-1};  // Bottom-right corner
    turtleMove result2 = studentTurtleStep(false, false);
    
    CU_ASSERT_TRUE(result1.validAction);
    CU_ASSERT_TRUE(result2.validAction);
}

void test_invalid_states() {
    mock_reset_state();
    robot_state = MOVING;  // Test invalid state handling
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_TRUE(result.validAction);  // Should handle gracefully
}

void test_equal_visit_counts() {
    mock_reset_state();
    // Set equal visit counts in all directions
    for(int i = 0; i < 4; i++) {
        coordinate next_pos = current_pos;
        next_pos.x += (i == 2) - (i == 0);  // Adjust for direction
        next_pos.y += (i == 3) - (i == 1);
        updateVisitMap(next_pos);
    }
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_TRUE(result.validAction);
}

int main() {
    CU_pSuite pSuite = NULL;
    if (CUE_SUCCESS != CU_initialize_registry()) return CU_get_error();
    
    pSuite = CU_add_suite("Suite_1", init, cleanup);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    // Add all tests
    if ((NULL == CU_add_test(pSuite, "test of transition T1", test_t1)) ||
        (NULL == CU_add_test(pSuite, "test of transition T2", test_t2)) ||
        (NULL == CU_add_test(pSuite, "test at goal", test_at_goal)) ||
        (NULL == CU_add_test(pSuite, "test no wall forward", test_no_wall_forward)) ||
        (NULL == CU_add_test(pSuite, "test wall rotation", test_wall_rotation)) ||
        (NULL == CU_add_test(pSuite, "test visit count decision", test_visit_count_decision)) ||
        (NULL == CU_add_test(pSuite, "test all directions", test_all_directions)) ||
        (NULL == CU_add_test(pSuite, "test grid boundaries", test_grid_boundaries)) ||
        (NULL == CU_add_test(pSuite, "test invalid states", test_invalid_states)) ||
        (NULL == CU_add_test(pSuite, "test equal visit counts", test_equal_visit_counts)))
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
