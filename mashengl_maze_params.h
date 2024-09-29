#ifndef MASHENGL_MAZE_PARAMS_H
#define MASHENGL_MAZE_PARAMS_H

const int GRID_SIZE = 23;
const int MOVE_DELAY = 40;
const int DIRECTION_COUNT = 4;

enum class TurtleDirection : int {
    WEST = 0,
    SOUTH = 1,
    EAST = 2,
    NORTH = 3
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
