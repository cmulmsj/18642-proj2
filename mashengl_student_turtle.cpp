/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/06/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT 40    // bigger number slows down simulation so you can see what's happening
float timeoutCounter, currentState;
float futureX, futureY, futureX2, futureY2;
float z, atEnd, mod, isBumped, q;

// This procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
int updateOrientation(int orientation, bool isBumped, float& currentState) {
    if (orientation == 0) {
        if (currentState == 2) { orientation = 1; currentState = 1; }  // Try right
        else if (isBumped)     { orientation = 3; currentState = 0; }  // If bumped, go left
        else currentState = 2;                                          // Otherwise, go straight
    } else if (orientation == 1) {
        if (currentState == 2) { orientation = 2; currentState = 1; }
        else if (isBumped)     { orientation = 0; currentState = 0; }
        else currentState = 2;
    } else if (orientation == 2) {
        if (currentState == 2) { orientation = 3; currentState = 1; }
        else if (isBumped)     { orientation = 1; currentState = 0; }
        else currentState = 2;
    } else if (orientation == 3) {
        if (currentState == 2) { orientation = 0; currentState = 1; }
        else if (isBumped)     { orientation = 2; currentState = 0; }
        else currentState = 2;
    }
    return orientation;
}

bool studentMoveTurtle(QPointF& pos_, int& nw_or) 
{
    ROS_INFO("Turtle update Called timeoutCounter=%f", timeoutCounter);
    mod = true;

    if (timeoutCounter == 0) {
        futureX = pos_.x(); futureY = pos_.y();
        futureX2 = pos_.x(); futureY2 = pos_.y();

        if (nw_or < 2) {
            if (nw_or == 0) futureY2 += 1;
            else            futureX2 += 1;
        } else {
            futureX2 += 1; futureY2 += 1;
            if (nw_or == 2) futureX += 1;
            else            futureY += 1;
        }

        isBumped = bumped(futureX, futureY, futureX2, futureY2);
        atEnd = atend(pos_.x(), pos_.y());

        // Use the helper function for right-hand rule orientation and state transitions
        nw_or = updateOrientation(nw_or, isBumped, currentState);

        ROS_INFO("Orientation=%f  STATE=%f", nw_or, currentState);
        z = currentState == 2;
        mod = true;

        if (z == true && atEnd == false) {
            if (nw_or == 1) pos_.setY(pos_.y() - 1);
            if (nw_or == 2) pos_.setX(pos_.x() + 1);
            if (nw_or == 3) pos_.setY(pos_.y() + 1);
            if (nw_or == 0) pos_.setX(pos_.x() - 1);

            z = false;
            mod = true;
        }
    }

    if (atEnd) return false;

    if (timeoutCounter == 0) timeoutCounter = TIMEOUT;
    else timeoutCounter -= 1;

    if (timeoutCounter == TIMEOUT) return true;

    return false;
}
