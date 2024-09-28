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

// Define global variables
int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
int32_t currentX = START_POS;
int32_t currentY = START_POS;
Direction orientation = LEFT; // Start facing left

turtleMove studentTurtleStep(bool bumped)
{
    static enum TurtleState {
        STATE_INIT,
        STATE_CHECK_RIGHT,
        STATE_MOVE_FORWARD,
        STATE_CHECK_FORWARD,
        STATE_TURN_LEFT
    } state = STATE_INIT;

    ROS_INFO("Current state: %d, Bumped: %s", state, bumped ? "true" : "false");

    turtleMove move;
    switch (state) {
        case STATE_INIT:
            state = STATE_CHECK_RIGHT;
            move = TURN_RIGHT;
            break;

        case STATE_CHECK_RIGHT:
            if (bumped) {
                state = STATE_TURN_LEFT;
                move = TURN_LEFT;
            } else {
                state = STATE_MOVE_FORWARD;
                move = MOVE_FORWARD;
            }
            break;

        case STATE_TURN_LEFT:
            state = STATE_CHECK_FORWARD;
            move = TURN_LEFT;
            break;

        case STATE_CHECK_FORWARD:
            if (bumped) {
                state = STATE_TURN_LEFT;
                move = TURN_LEFT;
            } else {
                state = STATE_MOVE_FORWARD;
                move = MOVE_FORWARD;
            }
            break;

        case STATE_MOVE_FORWARD:
            if (!bumped) {
                switch (orientation) {
                    case UP:    currentY -= 1; break;
                    case RIGHT: currentX += 1; break;
                    case DOWN:  currentY += 1; break;
                    case LEFT:  currentX -= 1; break;
                }
                setVisitCount(currentX, currentY, getVisitCount(currentX, currentY) + 1);
            }
            state = STATE_CHECK_RIGHT;
            move = MOVE_FORWARD;
            break;

        default:
            ROS_ERROR("Invalid state in studentTurtleStep");
            state = STATE_INIT;
            move = STOP;
            break;
    }

    ROS_INFO("Next move: %d, New state: %d, New orientation: %d", 
             static_cast<int>(move), state, static_cast<int>(orientation));
    return move;
}

int32_t getVisitCount(int32_t x, int32_t y)
{
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visitMap[y][x];
    }
    ROS_WARN("Attempted to get visit count for invalid position (%d, %d)", x, y);
    return 0;
}

void setVisitCount(int32_t x, int32_t y, int32_t count)
{
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visitMap[y][x] = count;
        ROS_INFO("Updated visit count at (%d, %d) to %d", x, y, count);
    } else {
        ROS_WARN("Attempted to set visit count for invalid position (%d, %d)", x, y);
    }
}

int getCurrentVisitCount()
{
    return getVisitCount(currentX, currentY);
}

// This function is no longer needed, but kept for compatibility
bool studentMoveTurtle(QPointF& pos_, int& nw_or)
{
    ROS_WARN("studentMoveTurtle called, but it's not being used in the current implementation");
    return true;
}
