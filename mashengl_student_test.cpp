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

// Test T1: Init to Forward transition
void test_t1(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // Execute
    turtleMove result = studentTurtleStep(false, false);
    
    // Verify
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

// T2: Forward to Forward (unblocked path)
void test_t2(void) {
    // Test for continuous forward movement
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    turtleMove result = studentTurtleStep(false, false);
    
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

// T3: Forward to Check_Unvisited
void test_t3(void) {
    // Test transition when path is blocked
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    turtleMove result = studentTurtleStep(true, false);
    
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
}

// T4: Check_Unvisited to Forward
void test_t4(void) {
    // Test finding an unvisited path
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_EAST);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    turtleMove result = studentTurtleStep(false, false);
    
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

// T5: Check_Unvisited to Check_Unvisited
void test_t5(void) {
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    turtleMove result = studentTurtleStep(true, false);
    
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
}

// T6: Check_Unvisited to Find_Least_Visited
void test_t6(void) {
    // Setup: Check all directions (need to call 4 times)
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // Make 4 moves to check all directions
    for(int i = 0; i < 3; i++) {
        turtleMove result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(result.action, RIGHT);
    }
    
    // Final move should transition to UNBUMPED
    turtleMove result = studentTurtleStep(true, false);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNBUMPED);
}

int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

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
    CU_basic_run_tests();
    CU_cleanup_registry();
    
    return 0;
}
