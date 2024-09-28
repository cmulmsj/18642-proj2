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

enum class turtleMove {
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    NO_MOVE
};

enum class Direction {
    LEFT,
    UP,
    RIGHT,
    DOWN
};

static std::vector<std::vector<int>> visitMap(MAZE_SIZE, std::vector<int>(MAZE_SIZE, 0));
static int currentX = START_POS;
static int currentY = START_POS;
static Direction currentDirection = Direction::UP;

turtleMove studentTurtleStep(bool bumped) {
    if (bumped) {
        currentDirection = static_cast<Direction>((static_cast<int>(currentDirection) + 1) % 4);
        return turtleMove::TURN_RIGHT;
    }
    
    // Move forward
    switch (currentDirection) {
        case Direction::LEFT:  currentX--; break;
        case Direction::UP:    currentY--; break;
        case Direction::RIGHT: currentX++; break;
        case Direction::DOWN:  currentY++; break;
    }
    
    visitMap[currentY][currentX]++;
    return turtleMove::FORWARD;
}

int getVisitsFromTurtle(int x, int y) {
    return visitMap[y][x];
}
