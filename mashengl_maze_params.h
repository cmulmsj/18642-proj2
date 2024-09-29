#ifndef MAZE_PARAMS_H
#define MAZE_PARAMS_H

const int GRID_DIMENSION = 23;
const int MOVE_COOLDOWN = 40;
const int ORIENTATION_COUNT = 4;
const int FULL_ROTATION = 4;

enum class MazeDirection : int {
    WEST = 0,
    SOUTH = 1,
    EAST = 2,
    NORTH = 3
};

enum class TurtleAction : int {
    MOVE,
    TURN_RIGHT,
    TURN_LEFT,
    STOP
};

enum class TurtleState : int {
    EXPLORING,
    TURNING,
    BACKTRACKING,
    FINISHED
};

#endif // MAZE_PARAMS_H
