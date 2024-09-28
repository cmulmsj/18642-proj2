#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

// Constants
const int32_t MAZE_SIZE = 23;
const int32_t START_POS = MAZE_SIZE / 2;

// Enum for directions
enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

// States
const int32_t STATE_MOVING = 2;
const int32_t STATE_TURNED = 1;
const int32_t STATE_BUMPED = 0;

// Global variables
static int32_t currentState = STATE_MOVING;
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
static int32_t currentX = 0;
static int32_t currentY = 0;
static int32_t currentOrientation = LEFT;

// Function to update the turtle's orientation
void updateOrientation(bool isBumped) {
    int32_t oldOrientation = currentOrientation;
    if (currentState == STATE_MOVING) {
        currentOrientation = (currentOrientation + 1) % 4;
        currentState = STATE_TURNED;
    }
    else if (isBumped) {
        currentOrientation = (currentOrientation + 2) % 4;
        currentState = STATE_BUMPED;
    }
    else {
        currentState = STATE_MOVING;
    }
    ROS_INFO("Orientation updated from %d to %d, new state: %d", oldOrientation, currentOrientation, currentState);
}

// Function to update the turtle's position
void updatePosition() {
    int32_t oldX = currentX;
    int32_t oldY = currentY;
    
    switch (currentOrientation) {
        case LEFT:  currentX--; break;
        case UP:    currentY--; break;
        case RIGHT: currentX++; break;
        case DOWN:  currentY++; break;
    }
    
    ROS_INFO("Relative position updated from (%d, %d) to (%d, %d)", oldX, oldY, currentX, currentY);
}

// Main function to determine the turtle's next move
turtleMove studentTurtleStep(bool bumped) {
    ROS_INFO("studentTurtleStep called with bumped: %s", bumped ? "true" : "false");
    
    updateOrientation(bumped);
    
    if (currentState == STATE_MOVING) {
        updatePosition();
        ROS_INFO("Turtle step: Moving, orientation: %d", currentOrientation);
    } else if (currentState == STATE_TURNED) {
        ROS_INFO("Turtle step: Turned, orientation: %d", currentOrientation);
    } else {
        ROS_INFO("Turtle step: Bumped, orientation: %d", currentOrientation);
    }
    
    return MOVE;
}

// Function to get and update the visit count for a given position
int getVisitCount(int x, int y) {
    int relX = x - START_POS + currentX;
    int relY = y - START_POS + currentY;
    if (relX >= 0 && relX < MAZE_SIZE && relY >= 0 && relY < MAZE_SIZE) {
        visitMap[relY][relX]++;
        ROS_INFO("Visit count for (%d, %d) updated to %d", x, y, visitMap[relY][relX]);
        return visitMap[relY][relX];
    }
    ROS_WARN("Attempted to access visit count outside of maze bounds: (%d, %d)", x, y);
    return 0;
}

// Initialize function (if needed)
void initializeTurtle() {
    currentState = STATE_MOVING;
    currentX = 0;
    currentY = 0;
    currentOrientation = LEFT;
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            visitMap[i][j] = 0;
        }
    }
    ROS_INFO("Turtle initialized at (0, 0), orientation: LEFT");
}
