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
#include "mashengl_maze_params.h"

bool moveTurtle(QPointF& pos_, int& nw_or) {
    static int count_down = 0;
    static NavigationMode current_mode = NavigationMode::INITIAL;
    static bool last_bumped = false; // Store the last bumped value

    if (count_down == 0) {
        if (current_mode == NavigationMode::INITIAL) {
            addVisit(pos_);
            displayVisits(getVisit(pos_));
        }

        bool at_goal = atend(pos_.x(), pos_.y());

        // Pass the previous bumped value to studentTurtleStep
        TurtleCommand nextMove = studentTurtleStep(last_bumped, at_goal, &current_mode);

        // Update the orientation based on the turtle's move
        nw_or = translateOrnt(nw_or, nextMove);

        // Now detect obstacles using the updated orientation
        bool has_wall = detectObstacle(pos_, nw_or);

        ROS_INFO("Orientation=%d  Mode=%d  NextMove=%d  HasWall=%d", nw_or, static_cast<int>(current_mode), static_cast<int>(nextMove), has_wall);

        if (nextMove == TurtleCommand::ADVANCE && !has_wall) {
            pos_ = translatePos(pos_, nw_or);
            addVisit(pos_);
            displayVisits(getVisit(pos_));
        }

        if (current_mode == NavigationMode::COMPLETE) {
            return false; // Turtle has reached the goal
        }

        // Store the current has_wall value for the next iteration
        last_bumped = has_wall;

        count_down = MOVE_DELAY;
        return true;
    }

    count_down--;
    return false;
}

QPointF translatePos(QPointF pos_, int orientation) {
    switch (static_cast<TurtleDirection>(orientation)) {
        case TurtleDirection::WEST:  pos_.setX(pos_.x() - 1); break;
        case TurtleDirection::SOUTH: pos_.setY(pos_.y() + 1); break;
        case TurtleDirection::EAST:  pos_.setX(pos_.x() + 1); break;
        case TurtleDirection::NORTH: pos_.setY(pos_.y() - 1); break;
    }
    return pos_;
}

int translateOrnt(int orientation, TurtleCommand nextMove) {
    switch (nextMove) {
        case TurtleCommand::ROTATE_CW:  return (orientation + 1) % DIRECTION_COUNT;
        case TurtleCommand::ROTATE_CCW: return (orientation - 1 + DIRECTION_COUNT) % DIRECTION_COUNT;
        default: return orientation;
    }
}

bool detectObstacle(QPointF pos_, int orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (static_cast<TurtleDirection>(orient)) {
        case TurtleDirection::WEST:  x2--; break;
        case TurtleDirection::SOUTH: y2++; break;
        case TurtleDirection::EAST:  x2++; break;
        case TurtleDirection::NORTH: y2--; break;
    }

    return bumped(x1, y1, x2, y2);
}
