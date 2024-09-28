/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/21/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Forward declaration of getVisitsFromTurtle
int getVisitsFromTurtle(int x, int y);

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
