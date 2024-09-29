#ifndef MASHENGL_MAZE_PARAMS_H
#define MASHENGL_MAZE_PARAMS_H

const int GRID_SIZE = 23;
const int MOVE_DELAY = 10;
const int DIRECTION_COUNT = 4;

enum class TurtleDirection : int {
    WEST = 0,  // x decreasing
    NORTH = 1, // y increasing
    EAST = 2,  // x increasing
    SOUTH = 3  // y decreasing
};

enum class TurtleCommand : int {
    ADVANCE,
    ROTATE_CW,
    ROTATE_CCW,
    HALT
};

enum class NavigationMode : int {
    INITIAL,
    FORWARD,
    ADJUST,
    COMPLETE
};

#endif // MASHENGL_MAZE_PARAMS_H
