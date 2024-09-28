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
    static enum State {
        INITIAL_TURN_RIGHT,
        CHECK_RIGHT,
        MOVE_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        CHECK_FORWARD
    } state = INITIAL_TURN_RIGHT;

    switch (state) {
        case INITIAL_TURN_RIGHT:
            // Turn right to start following the right-hand rule
            state = CHECK_RIGHT;
            return TURN_RIGHT;

        case CHECK_RIGHT:
            if (bumped) {
                // Wall to the right, turn back to original orientation
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                state = CHECK_FORWARD;
                return TURN_LEFT;
            } else {
                // No wall to the right, proceed forward
                state = MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case CHECK_FORWARD:
            if (bumped) {
                // Wall ahead, turn left to find a new path
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                state = TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall ahead, move forward
                state = MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case MOVE_FORWARD:
            // Update position
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
            state = TURN_RIGHT;
            return MOVE_FORWARD;

        case TURN_LEFT:
            // After turning left, check forward
            state = CHECK_FORWARD;
            return TURN_LEFT;

        case TURN_RIGHT:
            // Turn right to check for a wall on the right
            orientation = static_cast<Direction>((orientation + 1) % 4); // Turn right
            state = CHECK_RIGHT;
            return TURN_RIGHT;

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = INITIAL_TURN_RIGHT;
            return STOP;
    }
}
