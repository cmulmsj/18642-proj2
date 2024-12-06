/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 12/02/2024
 */

#include "student.h"

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
    static bool first_move = true;
    static int cycles = 0;
    bool state_changed = false;

    if (first_move) {
        // Don't modify the starting position - let ROS set it
        // Just initialize other state
        first_move = false;
        cycles = TIMEOUT;
        return true; 
    }

    // Rest of function stays the same
    if (cycles > 0) {
        cycles--;
        return false;
    }

    int current_x = static_cast<int>(std::floor(pos.x()));
    int current_y = static_cast<int>(std::floor(pos.y()));
    bool reached_goal = atend(current_x, current_y);
    
    if (reached_goal) {
        return false;
    }

    bool wall_detected = checkObstacle(pos, orientation);
    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);

    if (!next_move.validAction) {
        return false;
    }

    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected && !reached_goal) {
                switch (orientation) {
                    case 0: pos.setX(pos.x() - 1.0); break; // WEST
                    case 1: pos.setY(pos.y() - 1.0); break; // NORTH
                    case 2: pos.setX(pos.x() + 1.0); break; // EAST
                    case 3: pos.setY(pos.y() + 1.0); break; // SOUTH
                    default:
                        ROS_ERROR("Invalid orientation");
                        return false;
                }
                displayVisits(next_move.visitCount);
                state_changed = true;
            }
            break;
            
        case RIGHT:
            orientation = (orientation + 1) % 4;
            state_changed = true;
            break;
            
        case LEFT:
            orientation = (orientation + 3) % 4;
            state_changed = true;
            break;
            
        default:
            ROS_ERROR("Invalid action");
            return false;
    }
    
    cycles = TIMEOUT;
    return state_changed;
}


