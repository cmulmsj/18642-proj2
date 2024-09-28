/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
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

bool moveTurtle(QPointF& pos_, int& nw_or)
{
    int32_t futureX = pos_.x(), futureY = pos_.y();
    int32_t futureX2 = pos_.x(), futureY2 = pos_.y();

    if (nw_or < 2) {  // LEFT or UP
        if (nw_or == 0) futureY2 += 1;
        else            futureX2 += 1;
    } else {  // RIGHT or DOWN
        futureX2 += 1; futureY2 += 1;
        if (nw_or == 2) futureX += 1;
        else            futureY += 1;
    }

    bool isBumped = bumped(futureX, futureY, futureX2, futureY2);
    turtleMove nextMove = studentTurtleStep(isBumped);
    
    QPointF newPos = translatePos(pos_, nextMove);
    int newOrientation = translateOrnt(nw_or, nextMove);
    
    if (!bumped(newPos.x(), newPos.y(), newPos.x(), newPos.y())) {
        pos_ = newPos;
        nw_or = newOrientation;
        return true;
    }
    
    return false;
}

QPointF translatePos(QPointF pos_, turtleMove nextMove) {
    // Since we only have MOVE, we'll use the current position
    // The actual movement logic should be in studentMoveTurtle
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    // Since we only have MOVE, we'll return the current orientation
    // The actual orientation change logic should be in studentMoveTurtle
    return orientation;
}
