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
const int32_t MAZE_SIZE = 50;  // Increased size to handle negative coordinates
const int32_t START_POS = MAZE_SIZE / 2;

// Enum for directions (matching maze orientations)
enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

// Global variables
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
static int32_t currentX = START_POS;
static int32_t currentY = START_POS;
static Direction orientation = UP;  // Start facing UP

// Function to get visit count
int32_t getVisitCount(int32_t x, int32_t y) {
    return visitMap[y][x];
}

// Function to set visit count
void setVisitCount(int32_t x, int32_t y, int32_t count) {
    visitMap[y][x] = count;
}

// The turtle's movement logic
turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    if (firstCall) {
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    // Left-hand rule logic
    if (bumped) {
        // If bumped, turn right
        orientation = static_cast<Direction>((orientation + 1) % 4);
        return TURN_RIGHT;
    } else {
        // Move forward
        // Update position
        switch (orientation) {
            case LEFT: currentX -= 1; break;
            case UP:   currentY -= 1; break;
            case RIGHT: currentX += 1; break;
            case DOWN: currentY += 1; break;
        }
        // Increment visit count
        int32_t visits = getVisitCount(currentX, currentY) + 1;
        setVisitCount(currentX, currentY, visits);

        // Then turn left to keep the wall on the left
        orientation = static_cast<Direction>((orientation + 3) % 4);  // Equivalent to -1 mod 4
        return TURN_LEFT;
    }
}

// Function to get the current number of visits for the display
int getCurrentVisitCount() {
    return getVisitCount(currentX, currentY);
}

