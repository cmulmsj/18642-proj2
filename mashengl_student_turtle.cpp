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

// Turtle's orientation
enum Direction { UP = 0, RIGHT = 1, DOWN = 2, LEFT = 3 };
static Direction orientation = LEFT;

// Possible moves the turtle can make
enum turtleMove { FORWARD, TURN_LEFT, TURN_RIGHT, STOP };
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

    // Implement the right-hand wall-following logic
    // The turtle tries to keep a wall on its right

    // Variables to hold the next move and orientation
    turtleMove nextMove;

    if (bumped) {
        // If there's a wall ahead, turn left
        orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
        nextMove = TURN_LEFT;
    } else {
        // If no wall ahead, try to turn right to follow the wall
        // Update the turtle's position based on its orientation
        switch (orientation) {
            case UP:    currentY--; break;
            case RIGHT: currentX++; break;
            case DOWN:  currentY++; break;
            case LEFT:  currentX--; break;
        }

        // Update the visit count for the new cell
        int32_t visits = getVisitCount(currentX, currentY) + 1;
        setVisitCount(currentX, currentY, visits);

        nextMove = FORWARD;
    }

    return nextMove;
}
