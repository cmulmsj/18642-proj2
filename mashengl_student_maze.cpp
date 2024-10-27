/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

static uint8_t delay_timer = 0;

// Check for wall collision
bool checkWall(QPointF &pos_, int compass_orientation) {
    coordinate test1, test2;
    test1.x = pos_.x();
    test2.x = pos_.x();
    test1.y = pos_.y();
    test2.y = pos_.y();

    switch (compass_orientation) {
        case 0: // West
            test2.y += 1;
            break;
        case 1: // North
            test2.x += 1;
            break;
        case 2: // East
            test1.x += 1;
            test2.x += 1;
            test2.y += 1;
            break;
        case 3: // South
            test1.y += 1;
            test2.x += 1;
            test2.y += 1;
            break;
    }
    
    bool has_wall = bumped(test1.x, test1.y, test2.x, test2.y);
    ROS_INFO("[WALL] Check at (%d,%d), orient %d: %s", 
             (int)pos_.x(), (int)pos_.y(), compass_orientation, 
             has_wall ? "YES" : "NO");
    return has_wall;
}

// Move the turtle based on state
bool moveTurtle(QPointF& pos_, int& compass_orientation) {
    if (delay_timer > 0) {
        delay_timer--;
        return false;
    }

    bool wall_detected = checkWall(pos_, compass_orientation);
    bool goal_reached = atend(pos_.x(), pos_.y());
    
    ROS_INFO("[STATUS] Pos (%.1f,%.1f) Orient %d Wall %d Goal %d", 
             pos_.x(), pos_.y(), compass_orientation, wall_detected, goal_reached);
    
    turtleMove next_move = studentTurtleStep(wall_detected, goal_reached);
    
    if (next_move.validAction) {
        if (next_move.action == FORWARD) {
            if (!wall_detected) {
                pos_ = translatePos(pos_, next_move, compass_orientation);
                displayVisits(next_move.visitCount);
            }
        } else {
            compass_orientation = translateOrnt(compass_orientation, next_move);
        }
    }

    delay_timer = MOVE_WAIT;
    return next_move.validAction;
}

// Translate position based on orientation
QPointF translatePos(QPointF pos_, turtleMove nextMove, int orientation) {
    if (nextMove.action != FORWARD) return pos_;
    
    switch (orientation) {
        case 0: // West
            pos_.setX(pos_.x() - 1);
            break;
        case 1: // North
            pos_.setY(pos_.y() - 1);
            break;
        case 2: // East
            pos_.setX(pos_.x() + 1);
            break;
        case 3: // South
            pos_.setY(pos_.y() + 1);
            break;
    }
    
    ROS_INFO("[POS] Updated to (%.1f,%.1f)", pos_.x(), pos_.y());
    return pos_;
}

// Update orientation after turn
int translateOrnt(int orientation, turtleMove nextMove) {
    int new_orient = orientation;
    
    if (nextMove.action == RIGHT) {
        new_orient = (orientation + 1) % 4;
    } else if (nextMove.action == LEFT) {
        new_orient = (orientation + 3) % 4;
    }
    
    ROS_INFO("[ORIENT] Changed from %d to %d", orientation, new_orient);
    return new_orient;
}
