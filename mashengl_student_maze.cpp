/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"

bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int count_down = 0;
    static NavigationMode current_mode = INITIAL;

    if (count_down == 0) {
        if (current_mode == INITIAL) {
            addVisit(pos_);
            displayVisits(getVisit(pos_));
        }

        bool has_wall = detectObstacle(pos_, static_cast<TurtleDirection>(nw_or));
        bool at_goal = atend(pos_.x(), pos_.y());

        TurtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_mode);
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("Orientation=%d  Mode=%d  NextMove=%d", nw_or, current_mode, nextMove);

        if (nextMove == ADVANCE) {
            pos_ = translatePos(pos_, static_cast<TurtleDirection>(nw_or));
            addVisit(pos_);
            displayVisits(getVisit(pos_));
        }

        if (current_mode == COMPLETE) {
            return false;
        }

        count_down = MOVE_DELAY;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, TurtleDirection orientation) {
    switch (orientation) {
        case WEST:  pos_.setX(pos_.x() - 1); break;
        case SOUTH: pos_.setY(pos_.y() - 1); break; // Assuming y decreases when moving south
        case EAST:  pos_.setX(pos_.x() + 1); break;
        case NORTH: pos_.setY(pos_.y() + 1); break; // Assuming y increases when moving north
        default:    ROS_ERROR("Invalid Orientation"); break;
    }
    return pos_;
}

int translateOrnt(int orientation, TurtleMove nextMove) {
    switch (nextMove) {
        case ROTATE_CW:  return (orientation + 1) % DIRECTION_COUNT;
        case ROTATE_CCW: return (orientation - 1 + DIRECTION_COUNT) % DIRECTION_COUNT;
        default:         return orientation; // No orientation change for ADVANCE or HALT
    }
}

// Modified detectObstacle logic to ensure it's unique
bool detectObstacle(QPointF pos_, TurtleDirection orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    // Adjusting logic to modify detection based on orientation
    switch (orient) {
        case WEST:  x2 = x1 - 1; break;
        case SOUTH: y2 = y1 - 1; break;
        case EAST:  x2 = x1 + 1; break;
        case NORTH: y2 = y1 + 1; break;
        default:    ROS_ERROR("Invalid Orientation"); return false;
    }

    return bumped(x1, y1, x2, y2);
}

