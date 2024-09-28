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
const int32_t MAZE_SIZE = 50;  // Adjusted size to accommodate the turtle's local map
const int32_t START_POS_X = 0;  // Start at internal position (0, 0)
const int32_t START_POS_Y = 0;

// Enum for directions (matching maze orientations)
enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

// Global variables
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
static int32_t currentX = START_POS_X;
static int32_t currentY = START_POS_Y;
static Direction orientation = UP;  // Start facing UP

// Function to get visit count
int32_t getVisitCount(int32_t x, int32_t y) {
    return visitMap[y + MAZE_SIZE / 2][x + MAZE_SIZE / 2];
}

// Function to set visit count
void setVisitCount(int32_t x, int32_t y, int32_t count) {
    visitMap[y + MAZE_SIZE / 2][x + MAZE_SIZE / 2] = count;
}

// The turtle's movement logic
turtleMove studentTurtleStep(bool bumped) {
    static bool firstCall = true;
    if (firstCall) {
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    static int turnCount = 0;       // Count how many times we've turned in place
    static bool justMovedForward = false;  // Flag to track if we just moved forward

    ROS_INFO("studentTurtleStep called with bumped = %d", bumped);

    if (bumped) {
        // Turn right
        orientation = static_cast<Direction>((orientation + 1) % 4);
        turnCount++;
        if (turnCount >= 4) {
            // We've turned all the way around and can't move forward
            turnCount = 0;
            ROS_WARN("Surrounded by walls, stopping.");
            return STOP;  // Or another appropriate action
        }
        ROS_INFO("Bumped into wall, turning right. New orientation: %d", orientation);
        return TURN_RIGHT;
    } else {
        // Reset turn count when we can move forward
        turnCount = 0;

        if (justMovedForward) {
            // After moving forward, turn left to keep the wall on the left
            orientation = static_cast<Direction>((orientation + 3) % 4);  // Equivalent to -1 mod 4
            ROS_INFO("Turning left to keep wall on the left. New orientation: %d", orientation);
            justMovedForward = false;  // Reset the flag
            return TURN_LEFT;
        } else {
            // Move forward
            switch (orientation) {
                case LEFT: currentX -= 1; break;
                case UP:   currentY -= 1; break;
                case RIGHT: currentX += 1; break;
                case DOWN: currentY += 1; break;
                default:
                    ROS_ERROR("Invalid orientation: %d", orientation);
                    break;
            }
            int32_t visits = getVisitCount(currentX, currentY) + 1;
            setVisitCount(currentX, currentY, visits);

            ROS_INFO("Moved forward to position (%d, %d). Visits: %d", currentX, currentY, visits);

            justMovedForward = true;  // Set the flag to turn left on next tick
            return MOVE_FORWARD;
        }
    }
}

// Function to get the current number of visits for the display
int getCurrentVisitCount() {
    return getVisitCount(currentX, currentY);
}
