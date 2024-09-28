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
#include "mashengl_turtle_state.h"
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

// In mashengl_student_turtle.cpp

turtleMove studentTurtleStep(bool bumped) {
    static enum TurtleState {
        STATE_INIT,
        STATE_CHECK_RIGHT,
        STATE_MOVE_FORWARD,
        STATE_CHECK_FORWARD,
        STATE_TURN_LEFT
    } state = STATE_INIT;

    switch (state) {
        case STATE_INIT:
            // Start by attempting to turn right
            state = STATE_CHECK_RIGHT;
            return TURN_RIGHT;

        case STATE_CHECK_RIGHT:
            if (bumped) {
                // Wall to the right, turn left to original orientation
                state = STATE_TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall to the right, move forward
                state = STATE_MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case STATE_TURN_LEFT:
            // After turning left, check forward
            state = STATE_CHECK_FORWARD;
            return TURN_LEFT;

        case STATE_CHECK_FORWARD:
            if (bumped) {
                // Wall ahead, turn left again
                state = STATE_TURN_LEFT;
                return TURN_LEFT;
            } else {
                // No wall ahead, move forward
                state = STATE_MOVE_FORWARD;
                return MOVE_FORWARD;
            }

        case STATE_MOVE_FORWARD:
            // Move forward in the current orientation
            // Update internal position
            switch (orientation) {
                case UP:    currentY -= 1; break;
                case RIGHT: currentX += 1; break;
                case DOWN:  currentY += 1; break;
                case LEFT:  currentX -= 1; break;
            }
            // Increment visit count
            setVisitCount(currentX, currentY, getVisitCount(currentX, currentY) + 1);
            // After moving forward, attempt to turn right again
            state = STATE_CHECK_RIGHT;
            return MOVE_FORWARD;

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = STATE_INIT;
            return STOP;
    }
}

