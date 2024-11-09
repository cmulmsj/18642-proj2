#ifndef STUDENT_MOCK_H
#define STUDENT_MOCK_H

#include <stdint.h>
#include <stdio.h>

// Redefine ROS macros for testing
#define ROS_INFO(format, ...) printf(format "\n", ##__VA_ARGS__)
#define ROS_ERROR(format, ...) printf("ERROR: " format "\n", ##__VA_ARGS__)

// Define all enums and types here
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

// External variables that need to be accessible for testing
extern FSM_STATES current_state;
extern LOCAL_DIRECTION current_local_direction;
extern coordinate current_location;
extern uint8_t visit_count_map[30][30];

// Mock functions declarations
bool bumped(int x1, int y1, int x2, int y2);
bool atend(int x, int y);
void displayVisits(int visits);

// Test control functions
void setMockBump(bool will_bump);
void setMockAtEnd(bool at_end);
int getMockVisits();

// Test access functions
FSM_STATES getCurrentState();
void setCurrentState(FSM_STATES state);
LOCAL_DIRECTION getCurrentDirection();
void setCurrentDirection(LOCAL_DIRECTION dir);
coordinate getCurrentLocation();
void setCurrentLocation(coordinate loc);
void resetVisitMap();

#endif
