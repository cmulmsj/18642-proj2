#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "student_mock.h"

// Test cases following state chart transitions
void test_t1(void) {
    initialize_test();
    
    // Setup test state
    TurtleTestState state;
    state.current_state = STATE_FORWARD;
    state.current_direction = L_NORTH;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = false;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 0;
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

void test_t2(void) {
    initialize_test();
    
    // Setup for continuous forward movement
    TurtleTestState state;
    state.current_state = STATE_FORWARD;
    state.current_direction = L_NORTH;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = false;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 0;
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
void test_t3(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_FORWARD;
    state.current_direction = L_NORTH;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = true;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 0;
    set_test_state(state);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Get transition move
    result = studentTurtleStep(true, false);
    
    // Verify state after blocked path
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    // The state is actually set before checking directions
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
}

void test_t4(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_UNVISITED;
    state.current_direction = L_EAST;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = false;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 1;
    
    // Set all visit counts high except for in front
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            state.visit_count_map[i][j] = 1;
        }
    }
    // Clear the cell we want to move to
    state.visit_count_map[15][14] = 0;  // East is unvisited
    
    set_test_state(state);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Get transition move
    result = studentTurtleStep(false, false);
    
    // Verify transition to forward when unvisited path found
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

void test_t5(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_UNVISITED;
    state.current_direction = L_NORTH;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = true;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 1;  // Start with one direction checked
    set_test_state(state);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Get rotation move
    result = studentTurtleStep(true, false);
    
    // Verify direction checking continues
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
    CU_ASSERT_EQUAL(get_test_state().directions_checked, 2);  // Should increment by 1
}
void test_t6(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_UNVISITED;
    state.current_direction = L_WEST;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = true;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 4;  // All directions checked
    set_test_state(state);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Get transition move
    result = studentTurtleStep(true, false);
    
    // Verify transition to UNBUMPED
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNBUMPED);
}

// T7: Find_Least_Visited to Forward (optimal direction found)
void test_t7(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_UNBUMPED;
    state.current_direction = L_EAST;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = false;
    state.is_at_end = false;
    state.timeout_counter = 0;
    state.directions_checked = 0;
    
    // Set all visit counts high
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            state.visit_count_map[i][j] = 2;
        }
    }
    // Set the east direction as minimum visits
    state.visit_count_map[15][14] = 0;
    
    set_test_state(state);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(result.validAction, false);
    }
    
    // Get transition move
    result = studentTurtleStep(false, false);
    
    // Verify transition to FORWARD with optimal direction
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}
// Test goal condition
void test_goal_transition(void) {
    initialize_test();
    
    TurtleTestState state;
    state.current_state = STATE_FORWARD;
    state.current_direction = L_NORTH;
    state.current_location.x = 14;
    state.current_location.y = 14;
    state.is_bumped = false;
    state.is_at_end = true;  // Set goal condition
    state.timeout_counter = 0;
    state.directions_checked = 0;
    set_test_state(state);
    
    turtleMove result = studentTurtleStep(false, true);
    
    // Verify goal handling
    CU_ASSERT_EQUAL(result.validAction, false);
    
    cleanup_test();
}

int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("Student Turtle Test Suite", init_suite, clean_suite);
    
 // Add all tests
    CU_add_test(suite, "Test T1: Init to Forward", test_t1);
    CU_add_test(suite, "Test T2: Forward to Forward", test_t2);
    CU_add_test(suite, "Test T3: Forward to Check_Unvisited", test_t3);
    CU_add_test(suite, "Test T4: Check_Unvisited to Forward", test_t4);
    CU_add_test(suite, "Test T5: Check_Unvisited to Check_Unvisited", test_t5);
    CU_add_test(suite, "Test T6: Check_Unvisited to Find_Least_Visited", test_t6);
    CU_add_test(suite, "Test T7: Find_Least_Visited to Forward", test_t7);
    CU_add_test(suite, "Test Goal Transition", test_goal_transition);
    
    CU_basic_run_tests();
    CU_cleanup_registry();
    
    return 0;
}
