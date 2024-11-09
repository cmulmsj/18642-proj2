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

int init_suite(void) { return 0; }
int clean_suite(void) { return 0; }

int main() {
    CU_initialize_registry();
    CU_pSuite suite = CU_add_suite("Student Turtle Test Suite", init_suite, clean_suite);
    
    // Add tests
    CU_add_test(suite, "Test T1: Init to Forward", test_t1);
    
    // Run tests
    CU_basic_run_tests();
    CU_cleanup_registry();
    
    return 0;
}
