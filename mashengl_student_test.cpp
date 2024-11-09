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

void test_t1(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
  // First wait for timeout to expire (5 cycles)
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
        CU_ASSERT_EQUAL(result.validAction, false);  // During timeout, validAction should be false
    }
    
    // Now get the actual move
    result = studentTurtleStep(false, false);
    
    // Verify the actual move
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

void test_t2(void) {
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 6; i++) {
        result = studentTurtleStep(false, false);
    }
    
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

void test_t3(void) {
    // Setup
    setCurrentState(STATE_FORWARD);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // First wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(true, false);
    }
    
    // Get the actual transition move
    result = studentTurtleStep(true, false);
    
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
}

void test_t4(void) {
    // Setup
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_EAST);
    resetVisitMap();
    setMockBump(false);
    setMockAtEnd(false);
    
    // First wait for timeout
    turtleMove result;
    for(int i = 0; i < 5; i++) {
        result = studentTurtleStep(false, false);
    }
    
    // Get the actual transition
    result = studentTurtleStep(false, false);
    
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_EQUAL(result.validAction, true);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_FORWARD);
}

void test_t5(void) {
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    // Wait for timeout
    turtleMove result;
    for(int i = 0; i < 6; i++) {
        result = studentTurtleStep(true, false);
    }
    
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_EQUAL(getCurrentState(), STATE_UNVISITED);
}

void test_t6(void) {
    setCurrentState(STATE_UNVISITED);
    coordinate loc = {14, 14};
    setCurrentLocation(loc);
    setCurrentDirection(L_NORTH);
    resetVisitMap();
    setMockBump(true);
    setMockAtEnd(false);
    
    turtleMove result;
    
    // Need to check all 4 directions
    for(int dir = 0; dir < 4; dir++) {
        // Wait for timeout
        for(int i = 0; i < 5; i++) {
            result = studentTurtleStep(true, false);
            CU_ASSERT_EQUAL(result.validAction, false);
        }
        
        // Get direction check move
        result = studentTurtleStep(true, false);
        CU_ASSERT_EQUAL(result.action, RIGHT);
        CU_ASSERT_EQUAL(result.validAction, true);
    }
    
    // After 4 directions checked, should transition to UNBUMPED
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
