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

// mashengl_student_turtle.cpp

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
enum Direction { UP = 0, RIGHT = 1, DOWN = 2, LEFT = 3 };
static Direction orientation = UP; // Assuming the turtle starts facing UP

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
    static int state = 0; // 0: Try to turn right, 1: Move forward, 2: Turn left
    static int turnCount = 0; // To prevent infinite turning

    if (firstCall) {
        // Initialize the visit count at the starting position
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    if (bumped) {
        // Can't move forward, need to turn left
        orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
        turnCount++;
        if (turnCount > 4) {
            // We've turned all directions and are stuck
            return STOP;
        }
        return TURN_LEFT;
    } else {
        // Reset turn count since we're able to move
        turnCount = 0;

        // Move forward
        // Update the turtle's position based on its orientation
        switch (orientation) {
            case UP:    currentY--; break;
            case RIGHT: currentX++; break;
            case DOWN:  currentY++; break;
            case LEFT:  currentX--; break;
        }

        // Update visit count
        int32_t visits = getVisitCount(currentX, currentY) + 1;
        setVisitCount(currentX, currentY, visits);

        return FORWARD;
    }
}
