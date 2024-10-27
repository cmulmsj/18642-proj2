/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 */

#include "student.h"

// Current state tracking for the turtle
static Point current_pos = {0, 0};
static int current_orientation = 1; // Start facing North

// Map dimensions and center position
const int MAP_SIZE = 23;
const int CENTER = 11;
static int visit_count[MAP_SIZE][MAP_SIZE] = {0};

// Record visit count for current position
void record_path(QPointF &pos_) {
    int x = static_cast<int>(pos_.x()) + CENTER;
    int y = static_cast<int>(pos_.y()) + CENTER;
    if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
        visit_count[x][y]++;
    }
}

// Get visit count for a position
int getVisitNumber(Point &pos_) {
    int x = pos_.x + CENTER;
    int y = pos_.y + CENTER;
    if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
        return visit_count[x][y];
    }
    return 999;
}

// Check if the turtle will bump into a wall
bool detectBump(QPointF &pos_, int orientation) {
    Point pt1, pt2;
    pt1.x = pos_.x();
    pt1.y = pos_.y();
    pt2 = pt1;

    switch (orientation) {
        case 0: // West
            pt2.y += 1;
            break;
        case 1: // North
            pt2.x += 1;
            break;
        case 2: // East
            pt1.x += 1;
            pt2.x += 1;
            pt2.y += 1;
            break;
        case 3: // South
            pt1.y += 1;
            pt2.x += 1;
            pt2.y += 1;
            break;
    }
    return bumped(pt1.x, pt1.y, pt2.x, pt2.y);
}

// Update turtle position based on orientation
QPointF updatePosition(QPointF pos_, int orientation) {
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
    return pos_;
}

// Main interface function to move the turtle
bool moveTurtle(QPointF& pos_, int& orientation) {
    static int time_left = 0;
    const int TIMEOUT = 5;

    if (time_left == 0) {
        bool is_bumped = detectBump(pos_, orientation);
        bool is_at_end = atend(pos_.x(), pos_.y());
        
        turtleMove next_move = studentTurtleStep(is_bumped, is_at_end);
        
        // Update orientation based on turn
        if (next_move == TURNRIGHT) {
            orientation = (orientation + 1) % 4;
        } else if (next_move == TURNLEFT) {
            orientation = (orientation + 3) % 4;
        } else if (next_move == MOVE && !is_bumped) {
            pos_ = updatePosition(pos_, orientation);
            record_path(pos_);
            Point current = {static_cast<int32_t>(pos_.x()), 
                           static_cast<int32_t>(pos_.y())};
            displayVisits(getVisitNumber(current));
        }

        time_left = TIMEOUT;
        return true;
    }

    time_left--;
    return false;
}
