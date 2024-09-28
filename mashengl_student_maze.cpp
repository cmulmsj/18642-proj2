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

// Define orientation constants if they aren't already defined elsewhere
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Function to move the turtle and update its position in the maze
bool moveTurtle(QPointF& pos_, int& nw_or) {
    // Use the bumped function to check if the turtle is facing a wall
    int futureX = static_cast<int>(pos_.x());
    int futureY = static_cast<int>(pos_.y());
    int futureX2 = futureX, futureY2 = futureY; // second set of coordinates for boundary check
    
    // Adjust future positions based on the turtle's orientation
    if (nw_or == NORTH) futureY -= 1;
    else if (nw_or == SOUTH) futureY += 1;
    else if (nw_or == EAST)  futureX += 1;
    else if (nw_or == WEST)  futureX -= 1;
    
    // Call the bumped function with the new potential future position
    bool bumped = ::bumped(futureX, futureY, futureX2, futureY2); 

    // Get the next move from the turtle logic
    turtleMove nextMove = studentTurtleStep(bumped);

    // Translate the relative move to absolute coordinates
    pos_ = translatePos(pos_, nextMove, nw_or);
    nw_or = translateOrnt(nw_or, nextMove);

    // Update display based on the number of visits in the current cell
    int x = static_cast<int>(pos_.x());
    int y = static_cast<int>(pos_.y());
    int visits = getVisitCount(x, y);
    displayVisits(visits); // Display the number of visits in the maze

    // If turtle reaches the end, return false to stop moving
    if (atEnd(x, y)) {
        return false;
    }

    return true; // Continue moving if the end is not reached
}

// Translate relative turtle moves into absolute position based on the current orientation
QPointF translatePos(QPointF pos_, turtleMove nextMove, int nw_or) {
    // Update position based on the current orientation and the next move
    switch (nextMove) {
        case FORWARD:
            if (nw_or == NORTH) pos_.setY(pos_.y() - 1);
            if (nw_or == SOUTH) pos_.setY(pos_.y() + 1);
            if (nw_or == EAST)  pos_.setX(pos_.x() + 1);
            if (nw_or == WEST)  pos_.setX(pos_.x() - 1);
            break;
        default:
            break;
    }
    return pos_;
}

// Translate relative moves into absolute orientation changes
int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case LEFT:
            // Turning left
            orientation = (orientation + 3) % 4; 
            break;
        case RIGHT:
            // Turning right
            orientation = (orientation + 1) % 4; 
            break;
        default:
            break;
    }
    return orientation;
}
