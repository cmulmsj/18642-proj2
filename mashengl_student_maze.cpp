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
            addVisit(pos_);
            displayVisits(getVisit(pos_));
            ROS_INFO("moveTurtle: Initial position recorded and visits displayed.");
        }

        bool has_wall = check_bumped(pos_, static_cast<Orientation>(nw_or));
        bool at_goal = atend(pos_.x(), pos_.y());

        ROS_INFO("moveTurtle: Checking obstacles - Has Wall: %d, At Goal: %d", has_wall, at_goal);

        turtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_state);
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("moveTurtle: Orientation updated to %d after move %d", nw_or, nextMove);

        if (nextMove == MOVE) {
            if (!has_wall) {
                pos_ = translatePos(pos_, static_cast<Orientation>(nw_or));
                addVisit(pos_);
                displayVisits(getVisit(pos_));
                ROS_INFO("moveTurtle: Moved to new position (%.0f, %.0f)", pos_.x(), pos_.y());
            } else {
                ROS_WARN("moveTurtle: Attempted to move forward but a wall was detected. Position unchanged.");
            }
        }

        if (current_state == GOAL) {
            ROS_INFO("moveTurtle: Turtle has reached the goal. Halting.");
            return false;
        }

        count_down = TIMEOUT;
        ROS_INFO("moveTurtle: Move completed. Countdown reset to %d.", count_down);
        return true;
    }

    count_down--;
    ROS_DEBUG("moveTurtle: Countdown decreased to %d.", count_down);
    return false;
}

QPointF translatePos(QPointF pos_, Orientation orientation) {
    switch (orientation) {
        case LEFT:
            pos_.setX(pos_.x() - 1);
            ROS_INFO("translatePos: Moving LEFT to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case DOWN:
            pos_.setY(pos_.y() - 1);
            ROS_INFO("translatePos: Moving DOWN to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case RIGHT:
            pos_.setX(pos_.x() + 1);
            ROS_INFO("translatePos: Moving RIGHT to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        case UP:
            pos_.setY(pos_.y() + 1);
            ROS_INFO("translatePos: Moving UP to (%.0f, %.0f)", pos_.x(), pos_.y());
            break;
        default:
            ROS_ERROR("translatePos: Invalid Orientation %d", orientation);
            break;
    }
    return pos_;
}


int translateOrnt(int orientation, TurtleMove nextMove) {
    switch (nextMove) {
        case ROTATE_CW:
            return (orientation + 1) % NUM_ORIENTATIONS;
        case ROTATE_CCW:
            return (orientation - 1 + NUM_ORIENTATIONS) % NUM_ORIENTATIONS;
        case ADVANCE:
        case HALT:
            return orientation; // No orientation change for ADVANCE or HALT
        default:
            ROS_ERROR("Invalid Move in translateOrnt");
            return orientation;
    }
}

bool detectObstacle(QPointF pos_, Orientation orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (orient) {
        case LEFT:
            y2 = y1 + 1;
            break;
        case DOWN:
            x2 = x1 + 1;
            break;
        case RIGHT:
            x2 = x1 + 1;
            y2 = y1 + 1;
            x1 = x1 + 1;
            break;
        case UP:
            x2 = x1 + 1;
            y2 = y1 + 1;
            y1 = y1 + 1;
            break;
        default:
            ROS_ERROR("detectObstacle: Invalid Orientation %d", orient);
            return false;
    }

    // Log the coordinates being checked for a wall
    ROS_INFO("detectObstacle: Checking wall between (%d, %d) and (%d, %d)", x1, y1, x2, y2);

    bool wall_detected = bumped(x1, y1, x2, y2);

    ROS_INFO("detectObstacle: Wall detected: %d", wall_detected);
    return wall_detected;
}


