/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file contains the turtle's movement logic using the right-hand rule.
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
    LEFT = 0,  // Facing left
    UP = 1,    // Facing up
    RIGHT = 2, // Facing right
    DOWN = 3   // Facing down
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

// The turtle's movement logic implementing the right-hand rule
turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    if (firstCall) {
        // At the starting position, mark as visited
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    // The right-hand rule logic requires checking walls to the right and ahead
    // Since we can only make one call to bumped per tick, we need to manage this carefully

    // State variables
    enum TurtleState { STATE_CHECK_RIGHT, STATE_CHECK_FORWARD, STATE_MOVE_FORWARD, STATE_TURN_LEFT, STATE_TURN_RIGHT };
    static TurtleState state = STATE_CHECK_RIGHT;
    static bool rightIsClear = false;
    static bool forwardIsClear = false;

    switch (state) {
        case STATE_CHECK_RIGHT:
            // In this tick, we check if there's a wall to the right
            // Since we can only check the wall ahead, we temporarily turn right to face the right wall and check for a wall.
            orientation = static_cast<Direction>((orientation + 1) % 4); // Temporarily turn right
            state = STATE_CHECK_FORWARD;
            return TURN_RIGHT; // Turn right to face the right wall (for checking)

        case STATE_CHECK_FORWARD:
            // Now we check if there's a wall ahead (which is to the right of the original orientation)
            rightIsClear = !bumped;
            // Turn back to the original orientation
            orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left to original orientation
            state = STATE_MOVE_FORWARD;
            return TURN_LEFT; // Turn left to face original direction

        case STATE_MOVE_FORWARD:
            if (rightIsClear) {
                // If there's no wall to the right, turn right and move forward
                orientation = static_cast<Direction>((orientation + 1) % 4); // Turn right
                state = STATE_TURN_RIGHT;
                return TURN_RIGHT; // Turn right to face the new direction
            } else {
                // Check if we can move forward
                forwardIsClear = !bumped;
                if (forwardIsClear) {
                    // Move forward
                    // Update position
                    switch (orientation) {
                        case LEFT:  currentX -= 1; break;
                        case UP:    currentY -= 1; break;
                        case RIGHT: currentX += 1; break;
                        case DOWN:  currentY += 1; break;
                    }
                    // Increment visit count
                    {
                        int visits = getVisitCount(currentX, currentY) + 1;
                        setVisitCount(currentX, currentY, visits);
                    }
                    state = STATE_CHECK_RIGHT;
                    return MOVE_FORWARD;
                } else {
                    // Cannot move forward, turn left
                    orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                    state = STATE_TURN_LEFT;
                    return TURN_LEFT;
                }
            }

        case STATE_TURN_RIGHT:
            {
                // After turning right, move forward
                // Update position
                switch (orientation) {
                    case LEFT:  currentX -= 1; break;
                    case UP:    currentY -= 1; break;
                    case RIGHT: currentX += 1; break;
                    case DOWN:  currentY += 1; break;
                }
                // Increment visit count
                int visits = getVisitCount(currentX, currentY) + 1;
                setVisitCount(currentX, currentY, visits);
                state = STATE_CHECK_RIGHT;
                return MOVE_FORWARD;
            }

        case STATE_TURN_LEFT:
            // After turning left, we'll check the right wall again in the next tick
            state = STATE_CHECK_RIGHT;
            return TURN_LEFT;

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = STATE_CHECK_RIGHT;
            return STOP;
    }
}
