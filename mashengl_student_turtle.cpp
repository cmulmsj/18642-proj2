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

turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    static turtleMove lastMove = TURN_RIGHT;
    if (firstCall) {
        firstCall = false;
        // Start by turning right to check for a wall
        lastMove = TURN_RIGHT;
        return TURN_RIGHT;
    }

    // Use the bumped status to decide the next action
    switch (lastMove) {
        case TURN_RIGHT:
            if (bumped) {
                // Wall to the right, turn left to face original direction
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                lastMove = TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall to the right, proceed to move forward
                lastMove = MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case TURN_LEFT:
            // After turning left, check if we can move forward
            if (bumped) {
                // Wall ahead, turn left again
                orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
                lastMove = TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall ahead, move forward
                lastMove = MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case MOVE_FORWARD:
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
            // After moving forward, start over
            lastMove = TURN_RIGHT;
            return MOVE_FORWARD;

        default:
            ROS_ERROR("Invalid lastMove in studentTurtleStep");
            lastMove = TURN_RIGHT;
            return STOP;
    }
}

