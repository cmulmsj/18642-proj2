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
    static State current_state = EXPLORE;

    if (count_down == 0) {
        record_visited(pos_);
        displayVisits(get_visited(pos_));

        bool has_wall = check_bumped(pos_, nw_or);
        bool at_goal = atend(pos_.x(), pos_.y());

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        
        ROS_INFO("Before Move - Pos: (%.0f, %.0f), Orientation: %d, Move: %d, State: %d", 
                 pos_.x(), pos_.y(), nw_or, nextMove, current_state);

        if (nextMove == FORWARD && !has_wall) {
            pos_ = translatePos(pos_, nw_or);
        } else if (nextMove == TURN_RIGHT) {
            nw_or = translateOrnt(nw_or, TURN_RIGHT);
        } else if (nextMove == TURN_LEFT) {
            nw_or = translateOrnt(nw_or, TURN_LEFT);
        }

        ROS_INFO("After Move - Pos: (%.0f, %.0f), New Orientation: %d", 
                 pos_.x(), pos_.y(), nw_or);

        if (current_state == FINISH) {
            return false;
        }

        count_down = TIMEOUT;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, int orientation) {
    switch (static_cast<Orientation>(orientation)) {
        case LEFT:  pos_.setX(pos_.x() - 1); break;
        case DOWN:  pos_.setY(pos_.y() + 1); break;
        case RIGHT: pos_.setX(pos_.x() + 1); break;
        case UP:    pos_.setY(pos_.y() - 1); break;
        default:    ROS_ERROR("Invalid Orientation"); break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case TURN_RIGHT: return (orientation + 1) % NUM_ORIENTATIONS;
        case TURN_LEFT:  return (orientation - 1 + NUM_ORIENTATIONS) % NUM_ORIENTATIONS;
        case FORWARD:
        case STOP:       return orientation;
        default:         ROS_ERROR("Invalid Move"); return orientation;
    }
}

bool check_bumped(QPointF pos_, int orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (static_cast<Orientation>(orient)) {
        case LEFT:  x2 = x1 - 1; break;
        case DOWN:  y2 = y1 + 1; break;
        case RIGHT: x2 = x1 + 1; break;
        case UP:    y2 = y1 - 1; break;
        default:    ROS_ERROR("Invalid Orientation"); return false;
    }

    return bumped(x1, y1, x2, y2);
}


