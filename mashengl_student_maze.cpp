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

// Define the turtleMove enum if not already defined in student.h
enum class turtleMove {
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    NO_MOVE
};

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    bool bumped = bumped(pos_.x(), pos_.y(), pos_.x(), pos_.y());
    turtleMove nextMove = studentTurtleStep(bumped);
    
    QPointF newPos = translatePos(pos_, nextMove, nw_or);
    int newOrnt = translateOrnt(nw_or, nextMove);
    
    // Check if the new position is valid (not hitting a wall)
    if (!bumped(pos_.x(), pos_.y(), newPos.x(), newPos.y())) {
        pos_ = newPos;
        nw_or = newOrnt;
        
        // Call displayVisits with the number of visits from studentTurtleStep
        int visits = getVisitsFromTurtle(pos_.x(), pos_.y());
        displayVisits(visits);
        
        return true;
    }
    
    return false;
}

QPointF translatePos(QPointF pos_, turtleMove nextMove, int orientation) {
    switch (nextMove) {
        case turtleMove::FORWARD:
            switch (orientation) {
                case 0: return QPointF(pos_.x() - 1, pos_.y()); // Left
                case 1: return QPointF(pos_.x(), pos_.y() - 1); // Up
                case 2: return QPointF(pos_.x() + 1, pos_.y()); // Right
                case 3: return QPointF(pos_.x(), pos_.y() + 1); // Down
            }
        default:
            return pos_; // No change for turns or no move
    }
}

int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case turtleMove::TURN_LEFT:
            return (orientation - 1 + 4) % 4;
        case turtleMove::TURN_RIGHT:
            return (orientation + 1) % 4;
        default:
            return orientation; // No change for forward or no move
    }
}

// This function should be implemented in ANDREWID_student_turtle.cpp
extern turtleMove studentTurtleStep(bool bumped);

// This function should be implemented in ANDREWID_student_turtle.cpp
extern int getVisitsFromTurtle(int x, int y);
