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
#include <ros/ros.h>

// Function prototypes
bool isFacingWall(QPointF pos_, int nw_or);

// Function to check if the turtle is facing a wall
bool isFacingWall(QPointF pos_, int nw_or) {
    int32_t futureX = pos_.x(), futureY = pos_.y();
    int32_t futureX2 = pos_.x(), futureY2 = pos_.y();

    if (nw_or < 2) {  // Facing LEFT or UP
        if (nw_or == 0) futureY2 += 1;
        else            futureX2 += 1;
    } else {  // Facing RIGHT or DOWN
        futureX2 += 1; futureY2 += 1;
        if (nw_or == 2) futureX += 1;
        else            futureY += 1;
    }

    return bumped(futureX, futureY, futureX2, futureY2);
}

bool moveTurtle(QPointF& pos_, int& nw_or) {
    // Compute if the turtle is facing a wall
    bool bumpedStatus = isFacingWall(pos_, nw_or);

    // Call the turtle's step function to get the next move
    turtleMove nextMove = studentTurtleStep(bumpedStatus);

    // Update the absolute position and orientation
    translateOrnt(nw_or, nextMove);
    translatePos(pos_, nw_or, nextMove);

    // Get the number of visits from the turtle code
    int visits = getCurrentVisitCount();

    // Update the display
    displayVisits(visits);

    // Check if at end
    bool atEnd = atend(pos_.x(), pos_.y());

    return !atEnd;
}

// Update the turtle's absolute position based on the move
void translatePos(QPointF& pos_, int nw_or, turtleMove nextMove) {
    if (nextMove == MOVE_FORWARD) {
        switch (nw_or) {
            case 0: pos_.setX(pos_.x() - 1); break;  // LEFT
            case 1: pos_.setY(pos_.y() - 1); break;  // UP
            case 2: pos_.setX(pos_.x() + 1); break;  // RIGHT
            case 3: pos_.setY(pos_.y() + 1); break;  // DOWN
            default:
                ROS_ERROR("Invalid orientation: %d", nw_or);
                break;
        }
    }
    // No position change for turns
}

// Update the turtle's absolute orientation based on the move
void translateOrnt(int& nw_or, turtleMove nextMove) {
    if (nextMove == TURN_LEFT) {
        nw_or = (nw_or + 3) % 4;  // Equivalent to -1 mod 4
    } else if (nextMove == TURN_RIGHT) {
        nw_or = (nw_or + 1) % 4;
    }
    // No orientation change for MOVE_FORWARD or STOP
}

