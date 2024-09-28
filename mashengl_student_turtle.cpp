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
    static turtleMove lastMove = FORWARD;
    static int turnCount = 0;

    ROS_INFO("Turtle Input - Bumped: %d, Last Move: %d, Orientation: %d, Position: (%d, %d)",
             bumped, lastMove, orientation, currentX, currentY);

    turtleMove nextMove;

    // If we're not bumped, try to move forward
    if (!bumped) {
        nextMove = FORWARD;
        turnCount = 0;
    } else {
        // If we're bumped, turn right
        nextMove = TURN_RIGHT;
        orientation = (orientation + 1) % 4;
        turnCount++;

        // If we've turned right 4 times (made a full rotation), move left once
        if (turnCount >= 4) {
            nextMove = TURN_LEFT;
            orientation = (orientation + 3) % 4;
            turnCount = 0;
        }
    }

    // Update position if moving forward
    if (nextMove == FORWARD) {
        switch (orientation) {
            case 0: currentX++; break;
            case 1: currentY++; break;
            case 2: currentX--; break;
            case 3: currentY--; break;
        }
    }

    // Update visit count
    int visits = getVisitCount(currentX, currentY) + 1;
    setVisitCount(currentX, currentY, visits);

    ROS_INFO("Turtle Decision - Next Move: %d, New Orientation: %d, New Position: (%d, %d), Turn Count: %d",
             nextMove, orientation, currentX, currentY, turnCount);

    lastMove = nextMove;
    return nextMove;
}

