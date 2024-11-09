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
    
    // Initial setup - get into PLAN_NEXT state
    turtleMove init = studentTurtleStep(false, false);
    std::cout << "Initial state - Facing: " << facing_direction 
              << ", State: " << robot_state << std::endl;
    
    // Now test the rotation sequence with wall detected
    for(int i = 0; i < 3; i++) {
        turtleMove result = studentTurtleStep(true, false);
        std::cout << "Rotation " << i << " - Facing: " << facing_direction 
                  << ", Action: " << result.action 
                  << ", RotationsChecked: " << rotations_checked 
                  << ", Best Dir: " << best_direction 
                  << ", Wall: " << (mock_get_wall() ? "true" : "false") << std::endl;
        
        // Should be turning RIGHT to scan all directions
        CU_ASSERT_EQUAL(result.action, RIGHT);
        CU_ASSERT_TRUE(result.validAction);
    }

    // After scanning all directions, should make a movement decision
    turtleMove final_result = studentTurtleStep(true, false);
    std::cout << "Final decision - Facing: " << facing_direction 
              << ", Action: " << final_result.action 
              << ", RotationsChecked: " << rotations_checked 
              << ", Best Dir: " << best_direction << std::endl;
    
    // The final action should be valid and should turn to or move in best direction
    CU_ASSERT_TRUE(final_result.validAction);
    CU_ASSERT(final_result.action == FORWARD || 
              final_result.action == LEFT || 
              final_result.action == RIGHT);
}

void test_wall_handling() {
    mock_reset_state();
    mock_set_wall(true);
    
    // Get through initial state setup
    turtleMove init_result = studentTurtleStep(false, false);
    std::cout << "Wall handling initial - State: " << robot_state 
              << ", Facing: " << facing_direction 
              << ", Action: " << init_result.action << std::endl;
    
    // Now test wall response
    turtleMove result = studentTurtleStep(true, false);
    std::cout << "Wall handling response - State: " << robot_state 
              << ", Facing: " << facing_direction 
              << ", Action: " << result.action 
              << ", RotationsChecked: " << rotations_checked
              << ", BestDir: " << best_direction << std::endl;
    
    // When facing a wall, should take valid action
    CU_ASSERT_TRUE(result.validAction);
    
    // Action should be one of: RIGHT (to explore new direction), 
    // LEFT (to turn to better direction), or FORWARD (if better option available)
    CU_ASSERT(result.action == RIGHT || 
              result.action == LEFT || 
              result.action == FORWARD);
}

// Add comprehensive state transition test
void test_state_transitions() {
    mock_reset_state();
    
    // Test STARTUP to PLAN_NEXT
    CU_ASSERT_EQUAL(robot_state, STARTUP);
    turtleMove result1 = studentTurtleStep(false, false);
    CU_ASSERT_EQUAL(robot_state, PLAN_NEXT);
    
    // Test PLAN_NEXT rotation sequence
    for(int i = 0; i < 3; i++) {
        turtleMove result = studentTurtleStep(false, false);
        std::cout << "Rotation " << i << " - State: " << robot_state 
                  << ", Facing: " << facing_direction 
                  << ", Action: " << result.action 
                  << ", Rotations: " << rotations_checked << std::endl;
        CU_ASSERT_EQUAL(robot_state, PLAN_NEXT);
    }
    
    // Test decision after rotations
    turtleMove final = studentTurtleStep(false, false);
    std::cout << "Final decision - State: " << robot_state 
              << ", Action: " << final.action << std::endl;
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
        (NULL == CU_add_test(pSuite, "test equal visit counts", test_equal_visit_counts)) ||
        (NULL == CU_add_test(pSuite, "test wall handling", test_wall_handling)) ||
        (NULL == CU_add_test(pSuite, "test state transitions", test_state_transitions)))
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
