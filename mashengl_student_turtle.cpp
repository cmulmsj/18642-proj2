/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file contains the turtle's movement logic using the left-hand rule.
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

// The turtle's movement logic implementing the left-hand rule
turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    if (firstCall) {
        // At the starting position, mark as visited
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    static bool justMovedForward = false; // To track if we need to turn left after moving forward

    if (bumped) {
        // Turn right if we bumped into a wall
        orientation = static_cast<Direction>((orientation + 1) % 4);
        justMovedForward = false;
        return TURN_RIGHT;
    } else {
        if (justMovedForward) {
            // After moving forward, turn left to keep the wall on the left
            orientation = static_cast<Direction>((orientation + 3) % 4); // Equivalent to -1 mod 4
            justMovedForward = false;
            return TURN_LEFT;
        } else {
            // Move forward
            switch (orientation) {
                case UP:    currentY -= 1; break;
                case RIGHT: currentX += 1; break;
                case DOWN:  currentY += 1; break;
                case LEFT:  currentX -= 1; break;
                default:
                    ROS_ERROR("Invalid orientation in studentTurtleStep");
                    break;
            }
            // Increment visit count
            int visits = getVisitCount(currentX, currentY) + 1;
            setVisitCount(currentX, currentY, visits);
            justMovedForward = true;
            return MOVE_FORWARD;
        }
    }
}
