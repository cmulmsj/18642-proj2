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

// Translates relative move requests to absolute coordinates and updates the maze display.
bool moveTurtle(QPointF& pos_, int& nw_or) {
    bool bumped = checkWall(pos_, nw_or); // Check if the turtle is facing a wall
    turtleMove nextMove = studentTurtleStep(bumped); // Get the next move from the turtle logic

    // Translate the relative move to absolute coordinates
    pos_ = translatePos(pos_, nextMove);
    nw_or = translateOrnt(nw_or, nextMove);

    // Update display based on the number of visits in the current cell
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    int visits = getVisitCount(x, y);
    displayVisits(visits); // Display the number of visits

    // If turtle reaches the goal, stop moving
    if (atEnd(x, y)) {
        return false;
    }

    return true; // Turtle continues to move
}

// Translates turtleMove (e.g., forward, left, right) to new absolute position
QPointF translatePos(QPointF pos_, turtleMove nextMove) {
    switch (nextMove) {
        case FORWARD:
            if (nw_or == NORTH) pos_.setY(pos_.y() - 1);
            if (nw_or == SOUTH) pos_.setY(pos_.y() + 1);
            if (nw_or == EAST) pos_.setX(pos_.x() + 1);
            if (nw_or == WEST) pos_.setX(pos_.x() - 1);
            break;
        default:
            break;
    }
    return pos_;
}

// Translates turtleMove (e.g., left, right) to new orientation (NORTH, SOUTH, EAST, WEST)
int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case LEFT:
            orientation = (orientation + 3) % 4; // Turning left
            break;
        case RIGHT:
            orientation = (orientation + 1) % 4; // Turning right
            break;
        default:
            break;
    }
    return orientation;
}
