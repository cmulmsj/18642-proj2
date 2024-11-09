/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Mashengjun Li
 * ANDREW ID: mashengl
 * LAST UPDATE: 11/01/2024
 */

// #include "student.h"

// enum DIRECTION { WEST = 0, NORTH = 1, EAST = 2, SOUTH = 3 };

// bool bumpTest(QPointF &pos_, DIRECTION compass_orientation) {
//   coordinate bumptest1, bumptest2;
//   bumptest1.x = static_cast<uint8_t>(std::floor(pos_.x()));
//   bumptest2.x = static_cast<uint8_t>(std::floor(pos_.x()));
//   bumptest1.y = static_cast<uint8_t>(std::floor(pos_.y()));
//   bumptest2.y = static_cast<uint8_t>(std::floor(pos_.y()));

//   ROS_INFO("MAZE: Testing collision from position (%.2f, %.2f)", pos_.x(), pos_.y());

//   switch (compass_orientation) {
//     case WEST: {
//       bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
//       ROS_INFO("MAZE: Testing WEST wall at (%d,%d)->(%d,%d)", 
//                bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
//       break;
//     }
//     case NORTH: {
//       bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
//       ROS_INFO("MAZE: Testing NORTH wall at (%d,%d)->(%d,%d)", 
//                bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
//       break;
//     }
//     case EAST: {
//       bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
//       bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
//       bumptest1.x = static_cast<uint8_t>(bumptest1.x + 1U);
//       ROS_INFO("MAZE: Testing EAST wall at (%d,%d)->(%d,%d)", 
//                bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
//       break;
//     }
//     case SOUTH: {
//       bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
//       bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
//       bumptest1.y = static_cast<uint8_t>(bumptest1.y + 1U);
//       ROS_INFO("MAZE: Testing SOUTH wall at (%d,%d)->(%d,%d)", 
//                bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
//       break;
//     }
//     default: {
//       ROS_ERROR("MAZE: Invalid compass orientation in bumpTest: %d", compass_orientation);
//       break;
//     }
//   }
//   bool hit_wall = bumped(bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
//   ROS_INFO("MAZE: Wall detection result: %s", hit_wall ? "HIT" : "CLEAR");
//   return hit_wall;
// }

// bool moveTurtle(QPointF& pos_, int& compass_orientation) {
//   bool bumped = bumpTest(pos_, static_cast<DIRECTION>(compass_orientation));
//   bool at_end = atend(static_cast<int>(std::floor(pos_.x())), 
//                      static_cast<int>(std::floor(pos_.y())));
  
//   ROS_INFO("MAZE: Current position: (%.2f, %.2f), Orientation: %d", 
//            pos_.x(), pos_.y(), compass_orientation);
//   ROS_INFO("MAZE: Status - Wall: %s, Goal: %s", 
//            bumped ? "YES" : "NO", at_end ? "REACHED" : "NOT REACHED");

//   turtleMove nextMove = studentTurtleStep(bumped, at_end);
  
//   if(nextMove.validAction) {
//     translatePosAndOrientation(nextMove, pos_, compass_orientation);
//     ROS_INFO("MAZE: Executed move: %s", 
//              nextMove.action == FORWARD ? "FORWARD" : 
//              nextMove.action == LEFT ? "LEFT" : "RIGHT");
//   } else {
//     ROS_INFO("MAZE: No valid move available");
//   }
  
//   return nextMove.validAction;
// }

// void translatePosAndOrientation(turtleMove nextMove, QPointF& pos_, int& compass_orientation) {
//   switch (nextMove.action) {
//     case FORWARD: {
//       QPointF old_pos = pos_;
//       pos_ = translatePos(pos_, nextMove, compass_orientation);
//       displayVisits(nextMove.visitCount);
//       ROS_INFO("MAZE: Moved from (%.2f, %.2f) to (%.2f, %.2f)", 
//                old_pos.x(), old_pos.y(), pos_.x(), pos_.y());
//       break;
//     }
//     case LEFT: 
//     case RIGHT: {
//       int old_orientation = compass_orientation;
//       compass_orientation = translateOrnt(compass_orientation, nextMove);
//       ROS_INFO("MAZE: Rotated from %d to %d degrees", 
//                old_orientation * 90, compass_orientation * 90);
//       break;
//     }
//     default: {
//       ROS_ERROR("MAZE: Invalid movement command: %d", nextMove.action);
//       break;
//     }
//   }
// }

// QPointF translatePos(QPointF pos_, turtleMove, int compass_orientation) {
//   QPointF new_pos = pos_;
//   switch (compass_orientation) {
//     case NORTH: {
//       new_pos.setY(pos_.y() - 1.0);
//       ROS_INFO("MAZE: Moving NORTH by 1 unit");
//       break;
//     }
//     case EAST: {
//       new_pos.setX(pos_.x() + 1.0);
//       ROS_INFO("MAZE: Moving EAST by 1 unit");
//       break;
//     }
//     case SOUTH: {
//       new_pos.setY(pos_.y() + 1.0);
//       ROS_INFO("MAZE: Moving SOUTH by 1 unit");
//       break;
//     }
//     case WEST: {
//       new_pos.setX(pos_.x() - 1.0);
//       ROS_INFO("MAZE: Moving WEST by 1 unit");
//       break;
//     }
//     default: {
//       ROS_ERROR("MAZE: Invalid compass orientation in translatePos: %d", compass_orientation);
//       break;
//     }
//   }
//   return new_pos;
// }

// int translateOrnt(int orientation, turtleMove nextMove) {
//   int result;
//   switch (nextMove.action) {
//     case LEFT:
//       result = (orientation + 3) % 4;
//       ROS_INFO("MAZE: Rotating LEFT: %d -> %d", orientation, result);
//       break;
//     case RIGHT:
//       result = (orientation + 1) % 4;
//       ROS_INFO("MAZE: Rotating RIGHT: %d -> %d", orientation, result);
//       break;
//     default:
//       ROS_ERROR("MAZE: Invalid rotation command: %d", nextMove.action);
//       result = orientation;
//       break;
//   }
//   return result;
// }

// #ifdef testing
// #include "new_student_mock.h"
// #else
#include "student.h"
// #endif

// Visit tracking grid
static int visit_grid[GRID_SIZE][GRID_SIZE] = {0};

// Navigation helper functions
void updateVisitMap(QPointF pos) {
    int grid_x = static_cast<int>(pos.x()) + START_OFFSET;
    int grid_y = static_cast<int>(pos.y()) + START_OFFSET;
    if (grid_x >= 0 && grid_x < GRID_SIZE && grid_y >= 0 && grid_y < GRID_SIZE) {
        visit_grid[grid_x][grid_y]++;
    }
}

int getVisitCount(GridPoint loc) {
    int grid_x = loc.x + START_OFFSET;
    int grid_y = loc.y + START_OFFSET;
    if (grid_x >= 0 && grid_x < GRID_SIZE && grid_y >= 0 && grid_y < GRID_SIZE) {
        return visit_grid[grid_x][grid_y];
    }
    return INT_MAX;  // Return max value for out-of-bounds
}

int findOptimalDirection(QPointF pos) {
    int visit_counts[4];
    bool has_obstacle[4];
    int cur_x = static_cast<int>(pos.x());
    int cur_y = static_cast<int>(pos.y());

    // Check each direction
    GridPoint adjacent_cells[4] = {
        {cur_x - 1, cur_y}, // WEST
        {cur_x, cur_y - 1}, // NORTH
        {cur_x + 1, cur_y}, // EAST
        {cur_x, cur_y + 1}  // SOUTH
    };

    for (int dir = 0; dir < 4; dir++) {
        has_obstacle[dir] = moveTurtle(pos, dir);
        visit_counts[dir] = getVisitCount(adjacent_cells[dir]);
    }

    // Find direction with lowest visit count and no obstacle
    int min_visits = INT_MAX;
    int best_dir = -1;
    for (int dir = 0; dir < 4; dir++) {
        if (!has_obstacle[dir] && visit_counts[dir] <= min_visits) {
            min_visits = visit_counts[dir];
            best_dir = dir;
        }
    }

    return best_dir;
}

// Movement calculations
QPointF calculateNextPosition(QPointF pos, int direction) {
    switch (direction) {
        case WEST:  pos.setX(pos.x() - 1); break;
        case EAST:  pos.setX(pos.x() + 1); break;
        case NORTH: pos.setY(pos.y() - 1); break;
        case SOUTH: pos.setY(pos.y() + 1); break;
    }
    return pos;
}

int updateDirection(int current_dir, RobotAction next_action) {
    switch (next_action) {
        case ROTATE_CW:  return (current_dir + 1) % 4;
        case ROTATE_CCW: return (current_dir + 3) % 4;
        default: return current_dir;
    }
}

bool moveTurtle(QPointF& pos, int& orientation) {
    // int x = static_cast<int>(pos.x());
    // int y = static_cast<int>(pos.y());
    // int check_x1 = x, check_y1 = y;
    // int check_x2 = x, check_y2 = y;

    // switch (direction) {
    //     case WEST:
    //         check_y2 = y + 1;
    //         break;
    //     case NORTH:
    //         check_x2 = x + 1;
    //         break;
    //     case EAST:
    //         check_x1 = x + 1;
    //         check_x2 = x + 1;
    //         check_y2 = y + 1;
    //         break;
    //     case SOUTH:
    //         check_x2 = x + 1;
    //         check_y1 = y + 1;
    //         check_y2 = y + 1;
    //         break;
    // }
    // return bumped(check_x1, check_y1, check_x2, check_y2);
    int x = static_cast<int>(pos.x());
    int y = static_cast<int>(pos.y());
    int check_x1 = x, check_y1 = y;
    int check_x2 = x, check_y2 = y;

    // Calculate check points based on orientation
    switch (orientation) {
        case WEST:
            check_y2 = y + 1;
            break;
        case NORTH:
            check_x2 = x + 1;
            break;
        case EAST:
            check_x1 = x + 1;
            check_x2 = x + 1;
            check_y2 = y + 1;
            break;
        case SOUTH:
            check_x2 = x + 1;
            check_y1 = y + 1;
            check_y2 = y + 1;
            break;
    }

    // If no obstacle, update position
    if (!bumped(check_x1, check_y1, check_x2, check_y2)) {
        // Update position based on orientation
        switch (orientation) {
            case WEST:
                pos.setX(pos.x() - 1);
                break;
            case NORTH:
                pos.setY(pos.y() - 1);
                break;
            case EAST:
                pos.setX(pos.x() + 1);
                break;
            case SOUTH:
                pos.setY(pos.y() + 1);
                break;
        }
        return true;
    }
    return false;
}

