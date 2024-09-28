/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 09/28/2024
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the right-hand rule. The code is intentionally left obfuscated.
 *
 */

#include "student.h"
#include <stdint.h>
#include <ros/ros.h>

// Constants
const int32_t TIMEOUT = 40;
const int32_t MAZE_SIZE = 23;
const int32_t START_POS = MAZE_SIZE / 2;

// Enum for directions
enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

// States
const int32_t STATE_MOVING = 2;
const int32_t STATE_TURNED = 1;
const int32_t STATE_BUMPED = 0;

// Global variables
static int32_t timeoutCounter = 0;
static int32_t currentState = STATE_MOVING;
static bool atEnd = false;
static int32_t visitMap[MAZE_SIZE][MAZE_SIZE] = {0};
static int32_t currentX = START_POS;
static int32_t currentY = START_POS;
static int32_t currentOrientation = RIGHT;

// Function prototypes
int32_t getVisitCount(int32_t x, int32_t y);
void setVisitCount(int32_t x, int32_t y, int32_t count);
int32_t updateOrientation(int32_t orientation, bool isBumped, int32_t& currentState);

turtleMove studentTurtleStep(bool bumped) {
    currentOrientation = updateOrientation(currentOrientation, bumped, currentState);
    return MOVE;
}

int32_t getVisitCount(int32_t x, int32_t y) {
    return visitMap[y][x];
}

void setVisitCount(int32_t x, int32_t y, int32_t count) {
    visitMap[y][x] = count;
    displayVisits(count);
}

int32_t updateOrientation(int32_t orientation, bool isBumped, int32_t& currentState) {
    if (orientation == LEFT) {
        if (currentState == STATE_MOVING) { orientation = UP; currentState = STATE_TURNED; }
        else if (isBumped)                { orientation = DOWN; currentState = STATE_BUMPED; }
        else currentState = STATE_MOVING;
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
    static bool firstCall = true;
    if (firstCall) {
        setVisitCount(currentX, currentY, 1);
        firstCall = false;
    }

    ROS_INFO("Turtle update Called timeoutCounter=%d", timeoutCounter);

    if (timeoutCounter == 0) {
        int32_t futureX = pos_.x(), futureY = pos_.y();
        int32_t futureX2 = pos_.x(), futureY2 = pos_.y();

        if (nw_or < RIGHT) {  
            if (nw_or == LEFT) futureY2 += 1;
            else               futureX2 += 1;
        } else {  
            futureX2 += 1; futureY2 += 1;
            if (nw_or == RIGHT) futureX += 1;
            else                futureY += 1;
        }

        bool isBumped = bumped(futureX, futureY, futureX2, futureY2);
        atEnd = atend(pos_.x(), pos_.y());

        nw_or = updateOrientation(nw_or, isBumped, currentState);
        ROS_INFO("Orientation=%d  STATE=%d", nw_or, currentState);

        if (currentState == STATE_MOVING && !atEnd) {
            switch (nw_or) {
                case LEFT:  pos_.setX(pos_.x() - 1); currentX--; break;
                case UP:    pos_.setY(pos_.y() - 1); currentY--; break;
                case RIGHT: pos_.setX(pos_.x() + 1); currentX++; break;
                case DOWN:  pos_.setY(pos_.y() + 1); currentY++; break;
                default:
                    ROS_ERROR("Invalid orientation: %d", nw_or);
                    break;
            }
            int32_t visits = getVisitCount(currentX, currentY) + 1;
            setVisitCount(currentX, currentY, visits);
        }

        if (atEnd) return false;
        timeoutCounter = TIMEOUT;
    } else {
        timeoutCounter--;
    }

    return (timeoutCounter == TIMEOUT);
}
