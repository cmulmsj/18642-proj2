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

// Static array to track the turtle's local view of the world
static int localMap[23][23] = {0};

// The state machine that implements the right-hand rule for maze navigation
turtleMove studentTurtleStep(bool bumped) {
    static int direction = NORTH; // Turtle's relative orientation
    static QPointF relPos(11, 11); // Turtle's relative position in the local map
    
    // Update visit count in the local map
    int x = static_cast<int>(relPos.x());
    int y = static_cast<int>(relPos.y());
    
    if (!bumped) {
        localMap[x][y]++; // Increment visit count when moving forward
    }

    // Follow right-hand rule for movement decisions
    if (bumped) {
        // Rotate left if there's a wall on the right
        return LEFT;
    } else {
        // Move forward if no wall
        return FORWARD;
    }
}

// Function to get the number of visits at a particular relative position
int getVisitCount(int x, int y) {
    return localMap[x][y];
}

// Function to set/update the visit count in the local map
void setVisitCount(int x, int y, int visits) {
    localMap[x][y] = visits;
}

