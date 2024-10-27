/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// Counter for slowing down movement
static int move_timer = 0;

// Test wall collision for given position and orientation
bool checkWall(QPointF &pos_, int orient) {
    coordinate wall_test1, wall_test2;
    wall_test1.x = pos_.x();
    wall_test2.x = pos_.x();
    wall_test1.y = pos_.y();
    wall_test2.y = pos_.y();

    // Adjust test points based on orientation
    switch (orient) {
        case 0: // West
            ROS_INFO("[WALL] Testing WEST wall at (%d,%d)", 
                     wall_test1.x, wall_test1.y);
            wall_test2.y += 1;
            break;
        case 1: // North
            ROS_INFO("[WALL] Testing NORTH wall at (%d,%d)", 
                     wall_test1.x, wall_test1.y);
            wall_test2.x += 1;
            break;
        case 2: // East
            ROS_INFO("[WALL] Testing EAST wall at (%d,%d)", 
                     wall_test1.x, wall_test1.y);
            wall_test1.x += 1;
            wall_test2.x += 1;
            wall_test2.y += 1;
            break;
        case 3: // South
            ROS_INFO("[WALL] Testing SOUTH wall at (%d,%d)", 
                     wall_test1.x, wall_test1.y);
            wall_test1.y += 1;
            wall_test2.x += 1;
            wall_test2.y += 1;
            break;
        default:
            ROS_ERROR("[WALL] Invalid orientation: %d", orient);
            return true;
    }
    
    bool has_wall = bumped(wall_test1.x, wall_test1.y, wall_test2.x, wall_test2.y);
    ROS_INFO("[WALL] Wall detected: %s", has_wall ? "YES" : "NO");
    return has_wall;
}

// Main movement control function
bool moveTurtle(QPointF& pos_, int& compass_orientation) {
    ROS_INFO("[MOVE] Timer: %d, Position: (%.1f,%.1f), Facing: %d", 
             move_timer, pos_.x(), pos_.y(), compass_orientation);

    // Handle movement delay
    if (move_timer > 0) {
        move_timer--;
        return false;
    }

    // Check current status
    bool wall_hit = checkWall(pos_, compass_orientation);
    bool reached_end = atend(pos_.x(), pos_.y());
    
    ROS_INFO("[STATUS] Wall: %d, End: %d", wall_hit, reached_end);

    // Get next move from algorithm
    turtleMove next_action = studentTurtleStep(wall_hit, reached_end);
    
    if (!next_action.validAction) {
        ROS_INFO("[MOVE] Invalid action, skipping");
        return false;
    }

    // Execute movement
    if (next_action.action == FORWARD && !wall_hit) {
        ROS_INFO("[MOVE] Moving forward");
        pos_ = translatePos(pos_, next_action, compass_orientation);
        displayVisits(next_action.visitCount);
    } else if (next_action.action != NONE) {
        ROS_INFO("[MOVE] Turning %s", next_action.action == RIGHT ? "right" : "left");
        compass_orientation = translateOrnt(compass_orientation, next_action);
    }

    // Reset movement timer
    move_timer = MOVE_WAIT;
    return true;
}

// Update position based on orientation
QPointF translatePos(QPointF pos_, turtleMove nextMove, int compass_orientation) {
    if (nextMove.action != FORWARD) return pos_;
    
    switch (compass_orientation) {
        case 0: pos_.setX(pos_.x() - 1); break; // West
        case 1: pos_.setY(pos_.y() - 1); break; // North
        case 2: pos_.setX(pos_.x() + 1); break; // East
        case 3: pos_.setY(pos_.y() + 1); break; // South
    }
    
    ROS_INFO("[POS] Updated position to (%.1f,%.1f)", pos_.x(), pos_.y());
    return pos_;
}

// Update orientation based on turn
int translateOrnt(int orientation, turtleMove nextMove) {
    int new_orient = orientation;
    
    switch (nextMove.action) {
        case RIGHT: 
            new_orient = (orientation + 1) % 4; 
            break;
        case LEFT:  
            new_orient = (orientation + 3) % 4; 
            break;
        default: break;
    }
    
    ROS_INFO("[ORIENT] Changed from %d to %d", orientation, new_orient);
    return new_orient;
}
