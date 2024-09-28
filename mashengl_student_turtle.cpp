#include "student.h"
#include "mashengl_turtle_state.h"
#include <ros/ros.h>
#include <stdint.h>

// Define global variables
int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
int32_t currentX = START_POS;
int32_t currentY = START_POS;
Direction orientation = LEFT; // Start facing left

turtleMove studentTurtleStep(bool bumped) {
    static enum TurtleState {
        CHECK_RIGHT,
        MOVE_FORWARD,
        TURN_LEFT
    } state = CHECK_RIGHT;

    ROS_INFO("Current state: %d, Orientation: %d, Bumped: %s", 
             state, static_cast<int>(orientation), bumped ? "true" : "false");

    turtleMove move;

    switch (state) {
        case CHECK_RIGHT:
            // Always try to turn right first
            move = TURN_RIGHT;
            state = MOVE_FORWARD;
            break;

        case MOVE_FORWARD:
            if (!bumped) {
                // No wall in front, move forward
                move = MOVE_FORWARD;
                // Update position in the turtle's local coordinate system
                switch (orientation) {
                    case UP:    currentY--; break;
                    case RIGHT: currentX++; break;
                    case DOWN:  currentY++; break;
                    case LEFT:  currentX--; break;
                }
                setVisitCount(currentX, currentY, getVisitCount(currentX, currentY) + 1);
                state = CHECK_RIGHT; // After moving, check right again
            } else {
                // Wall in front, turn left
                move = TURN_LEFT;
                state = TURN_LEFT;
            }
            break;

        case TURN_LEFT:
            // We've turned left, now try to move forward
            move = MOVE_FORWARD;
            state = MOVE_FORWARD;
            break;
    }

    // Update orientation based on the move
    if (move == TURN_RIGHT) {
        orientation = static_cast<Direction>((static_cast<int>(orientation) + 1) % 4);
    } else if (move == TURN_LEFT) {
        orientation = static_cast<Direction>((static_cast<int>(orientation) + 3) % 4);
    }

    ROS_INFO("Next move: %d, New state: %d, New orientation: %d", 
             static_cast<int>(move), state, static_cast<int>(orientation));
    return move;
}

int32_t getVisitCount(int32_t x, int32_t y) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visitMap[y][x];
    }
    ROS_WARN("Attempted to get visit count for invalid position (%d, %d)", x, y);
    return 0;
}

void setVisitCount(int32_t x, int32_t y, int32_t count) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visitMap[y][x] = count;
        ROS_INFO("Updated visit count at (%d, %d) to %d", x, y, count);
    } else {
        ROS_WARN("Attempted to set visit count for invalid position (%d, %d)", x, y);
    }
}

int getCurrentVisitCount() {
    return getVisitCount(currentX, currentY);
}

// This function is no longer needed, but kept for compatibility
bool studentMoveTurtle(QPointF& pos_, int& nw_or) {
    ROS_WARN("studentMoveTurtle called, but it's not being used in the current implementation");
    return true;
}
