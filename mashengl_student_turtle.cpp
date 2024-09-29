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

// Initialize the visit map to keep track of visited cells
static int8_t visit_map[MAZE_SIZE][MAZE_SIZE] = {0};

// Renamed to addVisit
void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        visit_map[x][y]++;
        ROS_INFO("Visited (%d, %d): %d times", x, y, visit_map[x][y]);
    } else {
        ROS_WARN("Attempted to visit out-of-bounds position (%d, %d)", x, y);
    }
}

// Renamed to getVisit
uint8_t getVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
        return visit_map[x][y];
    }
    ROS_WARN("Attempted to get visit count for out-of-bounds position (%d, %d)", x, y);
    return 0;
}

TurtleMove studentTurtleStep(bool bumped, bool goal, NavigationMode* cur_state) {
    TurtleMove nextMove;

    if (goal) {
        *cur_state = COMPLETE;
        nextMove = HALT;
        ROS_INFO("Goal reached. Halting.");
        return nextMove;
    }

    switch (*cur_state) {
        case INITIAL:
        case PROCEED:
            // In INITIAL and PROCEED states, attempt to rotate clockwise to find a path
            if (!bumped) {
                *cur_state = PROCEED;
                nextMove = ADVANCE;
                ROS_INFO("State PROCEED: Advancing forward.");
            } else {
                *cur_state = ADJUST;
                nextMove = ROTATE_CW;
                ROS_INFO("State ADJUST: Rotating clockwise.");
            }
            break;

        case ADJUST:
            // In ADJUST state, decide whether to rotate counter-clockwise or advance
            if (bumped) {
                // If bumped after rotation, rotate counter-clockwise to try another direction
                nextMove = ROTATE_CCW;
                ROS_INFO("State ADJUST: Bumped after rotation. Rotating counter-clockwise.");
            } else {
                // If no bump after rotation, proceed to advance
                *cur_state = PROCEED;
                nextMove = ADVANCE;
                ROS_INFO("State ADJUST: No bump. Advancing forward.");
            }
            break;

        case COMPLETE:
            // In COMPLETE state, halt the turtle
            nextMove = HALT;
            ROS_INFO("State COMPLETE: Halting.");
            break;

        default:
            // Handle unexpected states
            ROS_ERROR("Invalid Navigation Mode encountered.");
            nextMove = HALT;
            break;
    }

    return nextMove;
}

