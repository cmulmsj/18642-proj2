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
    
    if (first_move) {
        first_move = false;
        cycles = TIMEOUT;
        return true;
    }

    if (cycles > 0) {
        cycles--;
        return false;
    }

    // Check goal first - this should happen before any moves
    bool reached_goal = atend(static_cast<int>(std::floor(pos.x())), 
                            static_cast<int>(std::floor(pos.y())));
    if (reached_goal) {
        return false;
    }

    // Single wall check
    bool wall_detected = checkObstacle(pos, orientation);
    
    // Get next move
    turtleMove next_move = studentTurtleStep(wall_detected, reached_goal);
    if (!next_move.validAction) {
        return false;
    }

    // Execute exactly one action per tick
    bool did_move = false;
    switch (next_move.action) {
        case FORWARD:
            if (!wall_detected && !reached_goal) {
                switch (orientation) {
                    case 0: pos.setX(pos.x() - 1.0); break;
                    case 1: pos.setY(pos.y() - 1.0); break;
                    case 2: pos.setX(pos.x() + 1.0); break;
                    case 3: pos.setY(pos.y() + 1.0); break;
                    default:
                        ROS_ERROR("Invalid orientation");
                        return false;
                }
                // Update visits exactly once after move
                displayVisits(next_move.visitCount);
                did_move = true;
            }
            break;
            
        case RIGHT:
            orientation = (orientation + 1) % 4;
            did_move = true;
            break;
            
        case LEFT:
            orientation = (orientation + 3) % 4;
            did_move = true;
            break;
            
        default:
            ROS_ERROR("Invalid action");
            return false;
    }
    
    cycles = TIMEOUT;
    return did_move;
}


