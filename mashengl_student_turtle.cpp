/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

// Constants
static const int32_t MAZE_SIZE = 23;
static const int32_t START_POS = MAZE_SIZE / 2;

// Global variables
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0}; // Initialize visit map
static int32_t currentX = START_POS; // Turtle's current X position in local map
static int32_t currentY = START_POS; // Turtle's current Y position in local map

// Turtle's orientation (relative)
enum Direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };
static Direction orientation = LEFT; // Turtle starts facing LEFT

// Function prototypes
int32_t getVisitCount(int32_t x, int32_t y);
void setVisitCount(int32_t x, int32_t y, int32_t count);

/**
 * Retrieves the visit count for a specific cell in the local map.
 */
int32_t getVisitCount(int32_t x, int32_t y) {
    return visitMap[y][x];
}

/**
 * Updates the visit count for a specific cell and calls displayVisits().
 */
void setVisitCount(int32_t x, int32_t y, int32_t count) {
    visitMap[y][x] = count;
    displayVisits(count);
}

/**
 * Determines the turtle's next move based on whether it has bumped into a wall.
 */
turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    if (firstCall) {
        // Initialize the visit count at the starting position
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    // Implementing the right-hand wall-following algorithm approximation
    static int state = 0; // 0: Try to turn right, 1: Move forward, 2: Turn left

    turtleMove nextMove;

    if (state == 0) {
        // Try to turn right
        orientation = static_cast<Direction>((orientation + 1) % 4); // Turn right
        nextMove = TURN_RIGHT;
        state = 1;
    } else if (state == 1) {
        // After turning right, attempt to move forward
        if (bumped) {
            // Can't move forward, need to turn left
            orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
            nextMove = TURN_LEFT;
            state = 2;
        } else {
            // Move forward
            // Update the turtle's internal position
            switch (orientation) {
                case LEFT:  currentX--; break;
                case DOWN:  currentY++; break;
                case RIGHT: currentX++; break;
                case UP:    currentY--; break;
            }

            // Update visit count
            int32_t visits = getVisitCount(currentX, currentY) + 1;
            setVisitCount(currentX, currentY, visits);

            nextMove = FORWARD;
            state = 0;
        }
    } else if (state == 2) {
        // After turning left, try to move forward
        if (bumped) {
            // Can't move forward, need to turn left again
            orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
            nextMove = TURN_LEFT;
            // Stay in state 2 to keep turning left until we can move forward
        } else {
            // Move forward
            // Update the turtle's internal position
            switch (orientation) {
                case LEFT:  currentX--; break;
                case DOWN:  currentY++; break;
                case RIGHT: currentX++; break;
                case UP:    currentY--; break;
            }

            // Update visit count
            int32_t visits = getVisitCount(currentX, currentY) + 1;
            setVisitCount(currentX, currentY, visits);

            nextMove = FORWARD;
            state = 0;
        }
    }

    return nextMove;
}
