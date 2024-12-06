/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 12/02/2024
 */

#include "student.h"

static bool first_move = true;
static int cycles = 0;
static bool waiting_after_move = false;
static bool check_atend = false;
static bool need_wall_check = true;
static bool wall_detected = false;
static bool reached_goal = false; 
bool state_changed = false;

bool checkObstacle(QPointF pos, int direction) {
    int x = static_cast<int>(std::floor(pos.x()));
    int y = static_cast<int>(std::floor(pos.y()));
    int x1 = x, y1 = y;
    int x2 = x, y2 = y;

    switch (direction) {
        case 0: // WEST
            y2 = y + 1;
            break;
        case 1: // NORTH
            x2 = x + 1;
            break;
        case 2: // EAST
            x1 = x + 1;
            x2 = x + 1;
            y2 = y + 1;
            break;
        case 3: // SOUTH
            x2 = x + 1;
            y1 = y + 1;
            y2 = y + 1;
            break;
        default:
            ROS_ERROR("Invalid direction");
            return true;
    }
    return bumped(x1, y1, x2, y2);
}


bool moveTurtle(QPointF& pos, int& orientation) {
    // Handle the first move
    if (first_move) {
        first_move = false;
        cycles = TIMEOUT;
        return true; 
    }

    // If we've reached the goal, stop all movement
    if (reached_goal) {
        return false;
    }

    // Handle timeouts
    if (cycles > 0) {
        cycles--;
        return false;
    }

    // Check walls after position/orientation has settled
    if (need_wall_check) {
        wall_detected = checkObstacle(pos, orientation);
        need_wall_check = false;
        cycles = TIMEOUT;
        return false;
    }

    // Handle post-movement goal check
    if (check_atend) {
        int current_x = static_cast<int>(std::floor(pos.x()));
        int current_y = static_cast<int>(std::floor(pos.y()));
        bool at_goal = atend(current_x, current_y);
        check_atend = false;
        cycles = TIMEOUT;
        if (at_goal) {
            reached_goal = true;  // Set the goal state
            return false;
        }
        need_wall_check = true;  // Schedule next wall check
        return true;
    }

    // If waiting after a move/turn, continue waiting
    if (waiting_after_move) {
        waiting_after_move = false;
        cycles = TIMEOUT;
        need_wall_check = true;  // Schedule wall check after wait
        return false;
    }

    // Get next move based on last wall check
    turtleMove next_move = studentTurtleStep(wall_detected, false);

    if (!next_move.validAction) {
        cycles = TIMEOUT;
        need_wall_check = true;
        return false;
    }

    // Execute the chosen action
    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected) {
                switch (orientation) {
                    case 0: // WEST
                        pos.setX(pos.x() - 1.0);
                        break;
                    case 1: // NORTH
                        pos.setY(pos.y() - 1.0);
                        break;
                    case 2: // EAST
                        pos.setX(pos.x() + 1.0);
                        break;
                    case 3: // SOUTH
                        pos.setY(pos.y() + 1.0);
                        break;
                    default:
                        ROS_ERROR("Invalid orientation");
                        return false;
                }
                displayVisits(next_move.visitCount);
                state_changed = true;
                waiting_after_move = true;  // Wait after movement
                check_atend = true;  // Schedule goal check after waiting
            } else {
                need_wall_check = true;  // Recheck walls if we couldn't move
            }
            break;

        case RIGHT:
            orientation = (orientation + 1) % 4;
            state_changed = true;
            waiting_after_move = true;
            break;

        case LEFT:
            orientation = (orientation + 3) % 4;
            state_changed = true;
            waiting_after_move = true;
            break;

        default:
            ROS_ERROR("Invalid action");
            cycles = TIMEOUT;
            need_wall_check = true;
            return false;
    }

    cycles = TIMEOUT;
    return state_changed;
}



