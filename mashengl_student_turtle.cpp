/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: [Your Last Update Date]
 *
 * This file contains the turtle's movement logic using a modified right-hand rule.
 * The turtle operates based on its local perception without knowledge of
 * absolute positions or orientations.
 */

#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

// Constants
const int32_t MAZE_SIZE = 100; // Adjusted size to accommodate the turtle's local map
const int32_t START_POS = MAZE_SIZE / 2;

// Enum for directions (relative to turtle's starting orientation)
enum Direction {
    UP = 0,    // Facing up
    RIGHT = 1, // Facing right
    DOWN = 2,  // Facing down
    LEFT = 3   // Facing left
};

// Global variables
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
static int32_t currentX = START_POS;
static int32_t currentY = START_POS;
static Direction orientation = LEFT; // Start facing left

// Function to get visit count
int32_t getVisitCount(int32_t x, int32_t y) {
    return visitMap[y][x];
}

// Function to set visit count
void setVisitCount(int32_t x, int32_t y, int32_t count) {
    visitMap[y][x] = count;
}

// Function to get the current number of visits for the display
int getCurrentVisitCount() {
    return getVisitCount(currentX, currentY);
}

// The turtle's movement logic implementing a modified right-hand rule
turtleMove studentTurtleStep(bool bumped) {
    // Prefix the enumerators in the State enum to avoid conflicts
    static enum State {
        STATE_INITIAL_TURN_RIGHT,
        STATE_CHECK_RIGHT,
        STATE_MOVE_FORWARD,
        STATE_TURN_LEFT,
        STATE_TURN_RIGHT,
        STATE_CHECK_FORWARD
    } state = STATE_INITIAL_TURN_RIGHT;

    switch (state) {
        case STATE_INITIAL_TURN_RIGHT:
            // Turn right to start following the right-hand rule
            state = STATE_CHECK_RIGHT;
            return TURN_RIGHT;

        case STATE_CHECK_RIGHT:
            if (bumped) {
                // Wall to the right, turn back to original orientation
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                state = STATE_CHECK_FORWARD;
                return TURN_LEFT;
            } else {
                // No wall to the right, proceed forward
                state = STATE_MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case STATE_CHECK_FORWARD:
            if (bumped) {
                // Wall ahead, turn left to find a new path
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                state = STATE_TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall ahead, move forward
                state = STATE_MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case STATE_MOVE_FORWARD:
            // Move forward in the current orientation
            switch (orientation) {
                case UP:    currentY -= 1; break;
                case RIGHT: currentX += 1; break;
                case DOWN:  currentY += 1; break;
                case LEFT:  currentX -= 1; break;
            }
            // Increment visit count
            {
                int visits = getVisitCount(currentX, currentY) + 1;
                setVisitCount(currentX, currentY, visits);
            }
            // After moving forward, try to turn right again
            state = STATE_TURN_RIGHT;
            return MOVE_FORWARD;

        case STATE_TURN_LEFT:
            // After turning left, check forward
            state = STATE_CHECK_FORWARD;
            return TURN_LEFT;

        case STATE_TURN_RIGHT:
            // Turn right to check for a wall on the right
            orientation = static_cast<Direction>((orientation + 1) % 4); // Turn right
            state = STATE_CHECK_RIGHT;
            return TURN_RIGHT;

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = STATE_INITIAL_TURN_RIGHT;
            return STOP;
    }
}
