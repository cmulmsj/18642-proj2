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
#include <vector>

const int MAZE_SIZE = 23;
const int START_POS = MAZE_SIZE / 2;

static std::vector<std::vector<int>> visitMap(MAZE_SIZE, std::vector<int>(MAZE_SIZE, 0));
static int currentX = START_POS;
static int currentY = START_POS;
static int currentOrientation = 1; // Start facing up (0: Left, 1: Up, 2: Right, 3: Down)

turtleMove studentTurtleStep(bool bumped)
{
    if (bumped) {
        // Turn right when bumped
        currentOrientation = (currentOrientation + 1) % 4;
    } else {
        // Move forward
        switch (currentOrientation) {
            case 0: currentX--; break; // Left
            case 1: currentY--; break; // Up
            case 2: currentX++; break; // Right
            case 3: currentY++; break; // Down
        }
        
        visitMap[currentY][currentX]++;
    }
    
    return MOVE;
}

int getVisitsFromTurtle(int x, int y)
{
    // Convert absolute coordinates to turtle's local coordinates
    int localX = x - START_POS + currentX;
    int localY = y - START_POS + currentY;
    
    // Check if the coordinates are within the visitMap bounds
    if (localX >= 0 && localX < MAZE_SIZE && localY >= 0 && localY < MAZE_SIZE) {
        return visitMap[localY][localX];
    }
    return 0; // Return 0 for out-of-bounds coordinates
}

// This function is no longer needed, but keeping it here as per the student.h file
bool studentMoveTurtle(QPointF& pos_, int& nw_or)
{
    // This function is now empty as its functionality has been moved to moveTurtle in mashengl_student_maze.cpp
    return true;
}
