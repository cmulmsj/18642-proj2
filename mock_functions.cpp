#ifdef testing

#include "student_mock.h"

static TurtleTestState test_state;
static bool error_occurred = false;

void initialize_test(void) {
    test_state.current_state = STATE_FORWARD;
    test_state.current_direction = L_NORTH;
    test_state.current_location.x = 14;
    test_state.current_location.y = 14;
    test_state.is_bumped = false;
    test_state.is_at_end = false;
    test_state.timeout_counter = 0;
    test_state.directions_checked = 0;
    
    // Initialize visit map
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            test_state.visit_count_map[i][j] = 0;
        }
    }
    error_occurred = false;
}

void set_test_state(TurtleTestState state) {
    test_state.current_state = state.current_state;
    test_state.current_direction = state.current_direction;
    test_state.current_location = state.current_location;
    test_state.is_bumped = state.is_bumped;
    test_state.is_at_end = state.is_at_end;
    test_state.timeout_counter = state.timeout_counter;
    test_state.directions_checked = state.directions_checked;
    
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            test_state.visit_count_map[i][j] = state.visit_count_map[i][j];
        }
    }
}

void cleanup_test(void) {
    initialize_test();
}

TurtleTestState get_test_state(void) {
    return test_state;
}

FSM_STATES getCurrentState(void) {
    return test_state.current_state;
}

void setCurrentState(FSM_STATES state) {
    test_state.current_state = state;
}

LOCAL_DIRECTION getCurrentDirection(void) {
    return test_state.current_direction;
}

void setCurrentDirection(LOCAL_DIRECTION dir) {
    test_state.current_direction = dir;
}

coordinate getCurrentLocation(void) {
    return test_state.current_location;
}

void setCurrentLocation(coordinate loc) {
    test_state.current_location = loc;
}

void setMockBump(bool bump) {
    test_state.is_bumped = bump;
}

void setMockAtEnd(bool at_end) {
    test_state.is_at_end = at_end;
}

void resetVisitMap(void) {
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            test_state.visit_count_map[i][j] = 0;
        }
    }
}

uint8_t getMockVisitCount(void) {
    coordinate loc = getCurrentLocation();
    return test_state.visit_count_map[loc.x][loc.y];
}

bool getError(void) {
    return error_occurred;
}
#endif
