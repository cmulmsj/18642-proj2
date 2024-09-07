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
// Enum for directions
enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

// States
const int STATE_MOVING = 2;
const int STATE_TURNED = 1;
const int STATE_BUMPED = 0;

float timeoutCounter, currentState;
float futureX, futureY, futureX2, futureY2;
float z, atEnd, mod, isBumped, q;

// This procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
int updateOrientation(int orientation, bool isBumped, float& currentState) {
    if (orientation == LEFT) {
        if (currentState == STATE_MOVING) { orientation = UP; currentState = STATE_TURNED; }  // Try right
        else if (isBumped)                { orientation = DOWN; currentState = STATE_BUMPED; }  // If bumped, go left
        else currentState = STATE_MOVING;                                              // Otherwise, go straight
    } else if (orientation == UP) {
        if (currentState == STATE_MOVING) { orientation = RIGHT; currentState = STATE_TURNED; }
        else if (isBumped)                { orientation = LEFT; currentState = STATE_BUMPED; }
        else currentState = STATE_MOVING;
    } else if (orientation == RIGHT) {
        if (currentState == STATE_MOVING) { orientation = DOWN; currentState = STATE_TURNED; }
        else if (isBumped)                { orientation = UP; currentState = STATE_BUMPED; }
        else currentState = STATE_MOVING;
    } else if (orientation == DOWN) {
        if (currentState == STATE_MOVING) { orientation = LEFT; currentState = STATE_TURNED; }
        else if (isBumped)                { orientation = RIGHT; currentState = STATE_BUMPED; }
        else currentState = STATE_MOVING;
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

        if (nw_or < RIGHT) {  
            if (nw_or == LEFT) futureY2 += 1;
            else               futureX2 += 1;
        } else {  
            futureX2 += 1; futureY2 += 1;
            if (nw_or == RIGHT) futureX += 1;
            else                futureY += 1;
        }

        isBumped = bumped(futureX, futureY, futureX2, futureY2);
        atEnd = atend(pos_.x(), pos_.y());

        nw_or = updateOrientation(nw_or, isBumped, currentState);

        ROS_INFO("Orientation=%f  STATE=%f", nw_or, currentState);

        if (currentState == STATE_MOVING && !atEnd) {
            switch (nw_or) {
                case LEFT:  pos_.setX(pos_.x() - 1); break;  // Move left
                case UP:    pos_.setY(pos_.y() - 1); break;  // Move up
                case RIGHT: pos_.setX(pos_.x() + 1); break;  // Move right
                case DOWN:  pos_.setY(pos_.y() + 1); break;  // Move down
            }
            mod = true;
        }

        if (atEnd) return false;

        timeoutCounter = DEFAULT_TIMEOUT;
    } else {
        timeoutCounter -= 1;
    }

    return (timeoutCounter == DEFAULT_TIMEOUT);
}
