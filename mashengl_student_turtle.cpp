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
static Direction orientation = LEFT; 

// Function prototypes
int32_t getVisitCount(int32_t x, int32_t y);
void setVisitCount(int32_t x, int32_t y, int32_t count);

// Variable to store the last move made
static turtleMove lastMove = FORWARD;

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
    static int orientation = 0; // 0: RIGHT, 1: DOWN, 2: LEFT, 3: UP
    static int state = 0; // 0: Check right, 1: Move forward, 2: Check left, 3: Turn around
    static bool firstMove = true;

    ROS_INFO("Turtle Input - Bumped: %d, Orientation: %d, Position: (%d, %d), State: %d, First Move: %d",
             bumped, orientation, currentX, currentY, state, firstMove);

    turtleMove nextMove;

    if (firstMove) {
        // On the first move, we want to move forward to check our initial orientation
        nextMove = FORWARD;
        firstMove = false;
    } else {
        switch (state) {
            case 0: // Check right
                nextMove = TURN_RIGHT;
                orientation = (orientation + 1) % 4;
                state = 1; // Next, try to move forward
                break;
            case 1: // Try to move forward
                if (!bumped) {
                    nextMove = FORWARD;
                    state = 0; // If successful, check right again
                } else {
                    nextMove = TURN_LEFT;
                    orientation = (orientation + 3) % 4;
                    state = 2; // If bumped, check left
                }
                break;
            case 2: // Check left
                if (!bumped) {
                    nextMove = FORWARD;
                    state = 0; // If path is clear, move forward
                } else {
                    nextMove = TURN_LEFT;
                    orientation = (orientation + 3) % 4;
                    state = 3; // If still bumped, prepare to turn around
                }
                break;
            case 3: // Turn around
                nextMove = TURN_LEFT;
                orientation = (orientation + 3) % 4;
                state = 1; // After turning around, try to move forward
                break;
        }
    }

    ROS_INFO("Turtle Decision - Next Move: %d, New Orientation: %d, New Position: (%d, %d), New State: %d",
             nextMove, orientation, currentX, currentY, state);

    return nextMove;
}
