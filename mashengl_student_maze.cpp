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
    bool is_bumped = bumped(pos_.x(), pos_.y(), pos_.x(), pos_.y());
    turtleMove nextMove = studentTurtleStep(is_bumped);
    
    QPointF newPos = translatePos(pos_, nextMove);
    int newOrnt = translateOrnt(nw_or, nextMove);
    
    // Check if the new position is valid (not hitting a wall)
    if (!bumped(pos_.x(), pos_.y(), newPos.x(), newPos.y())) {
        pos_ = newPos;
        nw_or = newOrnt;
        
        // We need to implement a way to get the number of visits
        // This could be done by keeping a static 2D array in this file
        // or by implementing a function in mashengl_student_turtle.cpp
        int visits = getVisitsFromTurtle(pos_.x(), pos_.y());
        displayVisits(visits);
        
        return true;
    }
    
    return false;
}

QPointF translatePos(QPointF pos_, turtleMove nextMove)
{
    // We need to get the current orientation
    // For now, let's assume it's stored in a static variable
    static int currentOrientation = 1; // Start facing up

    if (nextMove == MOVE) {
        switch (currentOrientation) {
            case 0: return QPointF(pos_.x() - 1, pos_.y()); // Left
            case 1: return QPointF(pos_.x(), pos_.y() - 1); // Up
            case 2: return QPointF(pos_.x() + 1, pos_.y()); // Right
            case 3: return QPointF(pos_.x(), pos_.y() + 1); // Down
        }
    }
    return pos_; // No change for other moves (which don't exist in current enum)
}

int translateOrnt(int orientation, turtleMove nextMove)
{
    // In the current implementation, MOVE doesn't change orientation
    // We might need to update this if we add more move types
    return orientation;
}

// This function should be implemented in mashengl_student_turtle.cpp
extern int getVisitsFromTurtle(int x, int y);
