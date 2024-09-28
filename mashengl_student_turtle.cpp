#include "student.h"
#include <vector>

const int MAZE_SIZE = 23;
const int START_POS = MAZE_SIZE / 2;

enum TurtleMove {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    NO_MOVE
};

// Local map to store visit counts
static std::vector<std::vector<int>> visitMap(MAZE_SIZE, std::vector<int>(MAZE_SIZE, 0));

// Turtle's relative position and orientation
static int relativeX = 0;
static int relativeY = 0;
static int relativeOrientation = 0; // 0: Left, 1: Up, 2: Right, 3: Down

TurtleMove studentTurtleStep(bool bumped) {
    if (bumped) {
        // If bumped, turn left
        relativeOrientation = (relativeOrientation - 1 + 4) % 4;
        return TURN_LEFT;
    } else {
        // Move forward
        switch (relativeOrientation) {
            case 0: relativeX--; break;
            case 1: relativeY--; break;
            case 2: relativeX++; break;
            case 3: relativeY++; break;
        }
        
        // Update visit count
        int actualX = START_POS + relativeX;
        int actualY = START_POS + relativeY;
        if (actualX >= 0 && actualX < MAZE_SIZE && actualY >= 0 && actualY < MAZE_SIZE) {
            visitMap[actualY][actualX]++;
        }
        
        return MOVE_FORWARD;
    }
}

// Function to get visit count for a relative position
int getVisitCount(int relX, int relY) {
    int actualX = START_POS + relX;
    int actualY = START_POS + relY;
    if (actualX >= 0 && actualX < MAZE_SIZE && actualY >= 0 && actualY < MAZE_SIZE) {
        return visitMap[actualY][actualX];
    }
    return 0;
}

// This function should be called from mashengl_student_maze.cpp
int updateVisitCount(int absoluteX, int absoluteY) {
    if (absoluteX >= 0 && absoluteX < MAZE_SIZE && absoluteY >= 0 && absoluteY < MAZE_SIZE) {
        return ++visitMap[absoluteY][absoluteX];
    }
    return 0;
}
