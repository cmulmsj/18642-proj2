#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "student_mock.h"

// External functions from student_turtle.cpp that we need for testing
extern FSM_STATES getCurrentState();
extern void setCurrentState(FSM_STATES state);
extern LOCAL_DIRECTION getCurrentDirection();
extern void setCurrentDirection(LOCAL_DIRECTION dir);
extern coordinate getCurrentLocation();
extern void setCurrentLocation(coordinate loc);
extern void resetVisitMap();
extern turtleMove studentTurtleStep(bool bumped, bool at_end);

// Test suite initialization
int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

// T1: Initial transition to FORWARD state and behavior
void test_t1(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_FORWARD);

    // Make move
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify first move
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
    
    // Verify timeout behavior
    for(int i = 0; i < 5; i++) {
        turtleMove timeout_result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(timeout_result.validAction, false);
    }
}

// T2: FORWARD to FORWARD (unblocked path)
void test_t2(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_FORWARD);

    // Make first move
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify first move
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
    
    // Wait for timeout
    for(int i = 0; i < 5; i++) {
        turtleMove timeout_result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(timeout_result.validAction, false);
    }
    
    // Make second move
    result = studentTurtleStep(false, false);
    
    // Verify continued forward movement
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

// T3: FORWARD to CHECK_UNVISITED (blocked path)
void test_t3(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_FORWARD);

    // Make move
    turtleMove result = studentTurtleStep(true, false);
    
    // Verify transition
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
    
    // Verify timeout behavior
    for(int i = 0; i < 5; i++) {
        turtleMove timeout_result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(timeout_result.validAction, false);
    }
}

// T4: CHECK_UNVISITED to FORWARD (found unvisited path)
void test_t4(void) {
    // Setup
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_EAST);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_UNVISITED);

    // Make move
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify transition
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
    
    // Verify timeout behavior
    for(int i = 0; i < 5; i++) {
        turtleMove timeout_result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(timeout_result.validAction, false);
    }
}

// T5: CHECK_UNVISITED to CHECK_UNVISITED (continue searching)
void test_t5(void) {
    // Setup
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_UNVISITED);

    // Make move
    turtleMove result = studentTurtleStep(true, false);
    
    // Verify continued searching
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
    
    // Verify timeout behavior
    for(int i = 0; i < 5; i++) {
        turtleMove timeout_result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(timeout_result.validAction, false);
    }
}

// T6: CHECK_UNVISITED to FIND_LEAST_VISITED (all directions checked)
void test_t6(void) {
    // Setup
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // Get initial state data
    FSM_STATES initial_state = getCurrentState();
    CU_ASSERT_EQUAL(initial_state, STATE_UNVISITED);

    // Check all four directions
    for(int dir = 0; dir < 4; dir++) {
        // Make move for this direction
        turtleMove result = studentTurtleStep(true, false);
        
        // Verify rotation
        CU_ASSERT_EQUAL(result.action, RIGHT);
        CU_ASSERT_EQUAL(result.validAction, true);
        
        // Wait for timeout
        for(int i = 0; i < 5; i++) {
            turtleMove timeout_result = studentTurtleStep(true, false);
            CU_ASSERT_EQUAL(timeout_result.validAction, false);
        }
    }
    
    // Verify final transition
    turtleMove final_result = studentTurtleStep(true, false);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNBUMPED);
}

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("Student Turtle Test Suite", init_suite, clean_suite);
    
    // Add tests
    CU_add_test(suite, "Test T1: Init to Forward", test_t1);
    CU_add_test(suite, "Test T2: Forward to Forward", test_t2);
    CU_add_test(suite, "Test T3: Forward to Check_Unvisited", test_t3);
    CU_add_test(suite, "Test T4: Check_Unvisited to Forward", test_t4);
    CU_add_test(suite, "Test T5: Check_Unvisited to Check_Unvisited", test_t5);
    CU_add_test(suite, "Test T6: Check_Unvisited to Find_Least_Visited", test_t6);
    
    // Run tests
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    
    return 0;
}
