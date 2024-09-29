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
    static NavigationMode current_mode = INITIAL;

    if (count_down == 0) {
        if (current_mode == INITIAL) {
            addVisit(pos_);
            displayVisits(getVisit(pos_));
        }

        bool at_goal = atend(pos_.x(), pos_.y());

        // Detect obstacle based on current orientation before deciding the next move
        bool has_wall = detectObstacle(pos_, static_cast<TurtleDirection>(nw_or));

        // Decide the next move based on the current state and obstacle detection
        TurtleMove nextMove = studentTurtleStep(has_wall, at_goal, &current_mode);

        // Update orientation based on the decided move
        nw_or = translateOrnt(nw_or, nextMove);

        ROS_INFO("Orientation=%d  Mode=%d  NextMove=%d", nw_or, current_mode, nextMove);

        // Move the turtle forward if the next move is ADVANCE and there's no wall
        if (nextMove == ADVANCE) {
            if (!has_wall) {
                pos_ = translatePos(pos_, static_cast<TurtleDirection>(nw_or));
                addVisit(pos_);
                displayVisits(getVisit(pos_));
                ROS_INFO("Moved to (%.0f, %.0f)", pos_.x(), pos_.y());
            } else {
                ROS_WARN("Attempted to move forward but detected a wall.");
            }
        }

        // Check if the turtle has reached the goal
        if (current_mode == COMPLETE) {
            ROS_INFO("Turtle has reached the goal.");
            return false;
        }

        count_down = TIMEOUT;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, TurtleDirection orientation) {
    switch (orientation) {
        case WEST:
            pos_.setX(pos_.x() - 1);
            break;
        case SOUTH:
            pos_.setY(pos_.y() - 1); // Assuming y decreases when moving south
            break;
        case EAST:
            pos_.setX(pos_.x() + 1);
            break;
        case NORTH:
            pos_.setY(pos_.y() + 1); // Assuming y increases when moving north
            break;
        default:
            ROS_ERROR("Invalid Orientation in translatePos");
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

// Modified detectObstacle function to ensure accurate wall detection
bool detectObstacle(QPointF pos_, TurtleDirection orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (orient) {
        case WEST:
            x2 = x1 - 1;
            y2 = y1;
            break;
        case SOUTH:
            x2 = x1;
            y2 = y1 - 1; // Moving south decreases y
            break;
        case EAST:
            x2 = x1 + 1;
            y2 = y1;
            break;
        case NORTH:
            x2 = x1;
            y2 = y1 + 1; // Moving north increases y
            break;
        default:
            ROS_ERROR("Invalid Orientation in detectObstacle");
            return false;
    }

    // Debugging information
    ROS_INFO("Checking obstacle at (%d, %d) for orientation %d", x2, y2, orient);

    return bumped(x1, y1, x2, y2);
}

