#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <stdint.h>
#include <stdio.h>

#ifdef testing

// ROS mock definitions
#define ROS_INFO(format, ...) printf(format "\n", ##__VA_ARGS__)
#define ROS_ERROR(format, ...) printf("ERROR: " format "\n", ##__VA_ARGS__)

// Enums and types
enum FSM_STATES { 
    STATE_FORWARD = 0,           
    STATE_UNVISITED = 1,   
    STATE_UNBUMPED = 2  
};

enum LOCAL_DIRECTION { 
    L_WEST = 0, 
    L_NORTH = 1, 
    L_EAST = 2, 
    L_SOUTH = 3 
};

enum turtleAction {FORWARD, LEFT, RIGHT};

typedef struct {
    uint8_t x;
    uint8_t y;
} coordinate;

typedef struct {
    turtleAction action;
    bool validAction;
    uint8_t visitCount;
} turtleMove;

// Test state structure
typedef struct {
    FSM_STATES current_state;
    LOCAL_DIRECTION current_direction;
    coordinate current_location;
    bool is_bumped;
    bool is_at_end;
    uint8_t timeout_counter;
    uint8_t directions_checked;
    uint8_t visit_count_map[30][30];
} TurtleTestState;

// External variables
extern uint8_t visit_count_map[30][30];

// Test control functions
void initialize_test(void);
void cleanup_test(void);
void set_test_state(TurtleTestState state);
TurtleTestState get_test_state(void);

// Mock state access
FSM_STATES getCurrentState(void);
void setCurrentState(FSM_STATES state);
LOCAL_DIRECTION getCurrentDirection(void);
void setCurrentDirection(LOCAL_DIRECTION dir);
coordinate getCurrentLocation(void);
void setCurrentLocation(coordinate loc);
void resetVisitMap(void);

// Mock control functions
void setMockBump(bool bump);
void setMockAtEnd(bool at_end);
uint8_t getMockVisitCount(void);
bool getError(void);

// Turtle functions
turtleMove studentTurtleStep(bool bumped, bool at_end);

#endif // testing

#endif // STUDENT_MOCK_H
