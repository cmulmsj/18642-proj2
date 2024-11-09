/*
 * Student Name: Mashengjun Li
 * Andrew ID: mashengl
 */

#include "student_mock.h"
#include <CUnit/Basic.h>

void test_t1() {
    mock_reset_state();
    mock_set_wall(false);
    
    turtleMove result = studentTurtleStep(false, false);
    CU_ASSERT_EQUAL(result.action, FORWARD);
    CU_ASSERT_TRUE(result.validAction);
}

void test_t2() {
    mock_reset_state();
    mock_set_wall(true);
    
    turtleMove result = studentTurtleStep(true, false);
    CU_ASSERT_EQUAL(result.action, RIGHT);
    CU_ASSERT_TRUE(result.validAction);
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
        (NULL == CU_add_test(pSuite, "test of transition T2", test_t2)))
    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();
    return CU_get_error();
}
