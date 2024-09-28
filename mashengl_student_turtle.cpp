/*
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: [Your Last Update Date]
 *
 * This file contains the turtle's movement logic using the right-hand rule.
 * The turtle operates based on its local perception without knowledge of
 * absolute positions or orientations.
 */

#include "student.h"
#include "turtle_state.h"
#include <ros/ros.h>
#include <stdint.h>
#include <QPointF>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>

// Define global variables
int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
int32_t currentX = START_POS;
int32_t currentY = START_POS;
Direction orientation = LEFT; // Start facing left

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

// The turtle's movement logic implementing the right-hand rule
turtleMove studentTurtleStep(bool bumped) {
    static enum TurtleState {
        STATE_START,
        STATE_CHECK_RIGHT,
        STATE_MOVE_FORWARD,
        STATE_CHECK_FORWARD,
        STATE_TURN_LEFT
    } state = STATE_START;

    static bool justTurned = false;

    if (justTurned) {
        // After a turn, we need to check for a wall ahead
        if (bumped) {
            // There's a wall ahead, need to turn left
            state = STATE_TURN_LEFT;
            justTurned = false;
            return TURN_LEFT;
        } else {
            // No wall ahead, move forward
            state = STATE_CHECK_RIGHT;
            justTurned = false;
            return MOVE_FORWARD;
        }
    }

    switch (state) {
        case STATE_START:
        case STATE_CHECK_RIGHT:
            // Try to turn right
            orientation = static_cast<Direction>((orientation + 1) % 4); // Turn right
            state = STATE_MOVE_FORWARD;
            justTurned = true;
            return TURN_RIGHT;

        case STATE_MOVE_FORWARD:
            // Move forward in the current orientation
            switch (orientation) {
                case UP:    currentY -= 1; break;
                case RIGHT: currentX += 1; break;
                case DOWN:  currentY += 1; break;
                case LEFT:  currentX -= 1; break;
            }
            // Increment visit count
            setVisitCount(currentX, currentY, getVisitCount(currentX, currentY) + 1);
            // After moving forward, check right again
            state = STATE_CHECK_RIGHT;
            return MOVE_FORWARD;

        case STATE_TURN_LEFT:
            // Turn left to find a new path
            orientation = static_cast<Direction>((orientation + 3) % 4); // Turn left
            state = STATE_CHECK_FORWARD;
            justTurned = true;
            return TURN_LEFT;

        case STATE_CHECK_FORWARD:
            // After turning left, check if we can move forward
            if (bumped) {
                // Wall ahead, turn left again
                state = STATE_TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall ahead, move forward
                state = STATE_MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = STATE_START;
            return STOP;
    }
}
