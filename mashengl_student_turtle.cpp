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
    static int state = 0; // 0: Move forward, 1: Turn right, 2: Turn left
    static bool firstMove = true;

    ROS_INFO("Turtle Input - Bumped: %d, State: %d, First Move: %d, Orientation: %d",
             bumped, state, firstMove, orientation);

    turtleMove nextMove;

    if (firstMove) {
        nextMove = FORWARD;
        firstMove = false;
        state = 1; // Next, we'll try to turn right
    } else {
        switch (state) {
            case 0: // Moving forward
                if (bumped) {
                    nextMove = TURN_RIGHT;
                    state = 1;
                } else {
                    nextMove = FORWARD;
                    state = 1; // Always try to turn right after moving forward
                }
                break;
            case 1: // Turning right
                nextMove = TURN_RIGHT;
                state = 0; // After turning right, try to move forward
                break;
            case 2: // Turning left
                nextMove = TURN_LEFT;
                state = 0; // After turning left, try to move forward
                break;
        }
    }

    // Update orientation based on turn
    if (nextMove == TURN_LEFT) {
        orientation = static_cast<Direction>((orientation + 3) % 4);
    } else if (nextMove == TURN_RIGHT) {
        orientation = static_cast<Direction>((orientation + 1) % 4);
    }

    // Update local position based on move (only if actually moving forward and not bumped)
    if (nextMove == FORWARD && !bumped) {
        switch (orientation) {
            case LEFT: currentX--; break;
            case RIGHT: currentX++; break;
            case UP: currentY--; break;
            case DOWN: currentY++; break;
        }
    }

    // Update visit count
    int visits = getVisitCount(currentX, currentY) + 1;
    setVisitCount(currentX, currentY, visits);

    ROS_INFO("Turtle Decision - Next Move: %d, New State: %d, Local Pos: (%d, %d), New Orientation: %d",
             nextMove, state, currentX, currentY, orientation);

    return nextMove;
}
