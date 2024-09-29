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

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    static int count_down = 0;
    static State current_state = INIT;

    if (count_down == 0) {
        if (current_state == INIT) {
            record_visited(pos_);
            displayVisits(get_visited(pos_));
        }

        bool has_wall = check_bumped(pos_, static_cast<Orientation>(nw_or));
        bool at_goal = atend(pos_.x(), pos_.y());

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("Orientation=%d  State=%d  NextMove=%d", nw_or, current_state, nextMove);

        if (nextMove == MOVE) {
            pos_ = translatePos(pos_, static_cast<Orientation>(nw_or));
            record_visited(pos_);
            displayVisits(get_visited(pos_));
        }

        if (current_state == GOAL) {
            return false;
        }

        count_down = TIMEOUT;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, Orientation orientation) {
    switch (orientation) {
        case LEFT:  pos_.setX(pos_.x() - 1); break;
        case DOWN:  pos_.setY(pos_.y() - 1); break;
        case RIGHT: pos_.setX(pos_.x() + 1); break;
        case UP:    pos_.setY(pos_.y() + 1); break;
        default:    ROS_ERROR("Invalid Orientation"); break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case TURNRIGHT: return (orientation + 1) % NUM_ORIENTATIONS;
        case TURNLEFT:  return (orientation - 1 + NUM_ORIENTATIONS) % NUM_ORIENTATIONS;
        case MOVE:
        case STOP:      return orientation;
        default:        ROS_ERROR("Invalid Move"); return orientation;
    }
}

bool check_bumped(QPointF pos_, Orientation orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (orient) {
        case LEFT:  y2++; break;
        case DOWN:  x2++; break;
        case RIGHT: x2++; y2++; x1++; break;
        case UP:    x2++; y2++; y1++; break;
        default:    ROS_ERROR("Invalid Orientation"); return false;
    }

    return bumped(x1, y1, x2, y2);
}
