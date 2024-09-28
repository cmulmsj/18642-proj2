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
}

// Function to update the turtle's position
void updatePosition() {
    switch (currentOrientation) {
        case LEFT:  currentX--; break;
        case UP:    currentY--; break;
        case RIGHT: currentX++; break;
        case DOWN:  currentY++; break;
    }
}

// Main function to determine the turtle's next move
turtleMove studentTurtleStep(bool bumped) {
    updateOrientation(bumped);
    
    if (currentState == STATE_MOVING) {
        updatePosition();
    }
    
    return MOVE;
}

// Function to get and update the visit count for a given position
int getVisitCount(int x, int y) {
    int relX = x - START_POS + currentX;
    int relY = y - START_POS + currentY;
    if (relX >= 0 && relX < MAZE_SIZE && relY >= 0 && relY < MAZE_SIZE) {
        return ++visitMap[relY][relX];
    }
    return 0;
}
