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

// mashengl_student_maze.cpp

#include "student.h"

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF& pos_, int& nw_or)
{
    // Determine if the turtle is facing a wall
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    // Calculate the position ahead based on orientation
    switch (nw_or) {
        case 0: x2 = x1 - 1; break; // LEFT
        case 1: y2 = y1 - 1; break; // UP
        case 2: x2 = x1 + 1; break; // RIGHT
        case 3: y2 = y1 + 1; break; // DOWN
        default:
            ROS_ERROR("Invalid orientation: %d", nw_or);
            return false;
    }

    // Check if there's a wall ahead
    bool isBumped = bumped(x1, y1, x2, y2);

    // Call the turtle's decision-making function
    turtleMove nextMove = studentTurtleStep(isBumped);

    // Update the orientation
    nw_or = translateOrnt(nw_or, nextMove);

    bool moveSuccessful = false;

    // Update the position if the turtle wants to move forward and there's no wall
    if (nextMove == FORWARD) {
        // Calculate the new position based on the updated orientation
        int newX = x1, newY = y1;
        switch (nw_or) {
            case 0: newX = x1 - 1; break; // LEFT
            case 1: newY = y1 - 1; break; // UP
            case 2: newX = x1 + 1; break; // RIGHT
            case 3: newY = y1 + 1; break; // DOWN
        }

        // Check if moving to the new position would result in a collision
        bool wouldBump = bumped(x1, y1, newX, newY);

        if (!wouldBump) {
            // Update position
            pos_.setX(newX);
            pos_.setY(newY);
            moveSuccessful = true;
        } else {
            // Cannot move forward due to a wall
            ROS_INFO("Cannot move forward; wall detected ahead.");
        }
    }

    // Inform the turtle whether the move was successful
    updateAfterMove(moveSuccessful);

    // Check if the turtle has reached the end
    bool atEnd = atend(pos_.x(), pos_.y());

    // Return true to continue, false to stop the turtle
    return !atEnd;
}

/*
 * Takes a position, orientation, and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove, int orientation) {
    if (nextMove == FORWARD) {
        switch (orientation) {
            case 0: pos_.setX(pos_.x() - 1); break; // LEFT
            case 1: pos_.setY(pos_.y() - 1); break; // UP
            case 2: pos_.setX(pos_.x() + 1); break; // RIGHT
            case 3: pos_.setY(pos_.y() + 1); break; // DOWN
            default:
                ROS_ERROR("Invalid orientation: %d", orientation);
                break;
        }
    }
    // No position change for turns or stop
    return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orientation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        orientation = (orientation + 3) % 4; // Turn left
    } else if (nextMove == TURN_RIGHT) {
        orientation = (orientation + 1) % 4; // Turn right
    }
    // No orientation change for FORWARD or STOP
    return orientation;
}

