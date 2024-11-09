#include "student_mock.h"

static TurtleTestState test_state;
static bool error_occurred = false;

void initialize_test(void) {
    test_state = {
        .current_state = STATE_FORWARD,
        .current_direction = L_NORTH,
        .current_location = {14, 14},
        .is_bumped = false,
        .is_at_end = false,
        .timeout_counter = 0,
        .directions_checked = 0
    };
    
    // Initialize visit map
    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 30; j++) {
            test_state.visit_count_map[i][j] = 0;
        }
    }
    error_occurred = false;
}

void cleanup_test(void) {
    initialize_test();
}

void set_test_state(TurtleTestState state) {
    test_state = state;
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
