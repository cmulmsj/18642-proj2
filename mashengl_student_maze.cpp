/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

static int visit_count[MAZE_GRID_SIZE][MAZE_GRID_SIZE] = {0};
static TurtleState current_state = EXPLORE;  // Changed to match new enum

void addVisit(QPointF& pos_) {
    int x = static_cast<int>(pos_.x()) + CENTER_POS;
    int y = static_cast<int>(pos_.y()) + CENTER_POS;
    if (x >= 0 && x < MAZE_GRID_SIZE && y >= 0 && y < MAZE_GRID_SIZE) {
        visit_count[x][y]++;
    }
}

uint8_t retrieveVisitCount(QPointF& pos_) {
    int x = static_cast<int>(pos_.x()) + CENTER_POS;
    int y = static_cast<int>(pos_.y()) + CENTER_POS;
    if (x >= 0 && x < MAZE_GRID_SIZE && y >= 0 && y < MAZE_GRID_SIZE) {
        return visit_count[x][y];
    }
    return 255;
}

int getVisitNumber(Point &pos_) {
    int x = pos_.x + CENTER_POS;
    int y = pos_.y + CENTER_POS;
    if (x >= 0 && x < MAZE_GRID_SIZE && y >= 0 && y < MAZE_GRID_SIZE) {
        return visit_count[x][y];
    }
    return 999;
}

bool detectObstacle(QPointF pos_, Orientation orient) {
    int x1 = pos_.x(), y1 = pos_.y();
    int x2 = x1, y2 = y1;

    switch (orient) {
        case WEST:
            y2 += 1;
            break;
        case NORTH:
            x2 += 1;
            break;
        case EAST:
            x1 += 1;
            x2 += 1;
            y2 += 1;
            break;
        case SOUTH:
            y1 += 1;
            x2 += 1;
            y2 += 1;
            break;
    }
    return bumped(x1, y1, x2, y2);
}

QPointF translatePos(QPointF pos_, Orientation orientation) {
    switch (orientation) {
        case WEST:
            pos_.setX(pos_.x() - 1);
            break;
        case NORTH:
            pos_.setY(pos_.y() - 1);
            break;
        case EAST:
            pos_.setX(pos_.x() + 1);
            break;
        case SOUTH:
            pos_.setY(pos_.y() + 1);
            break;
    }
    return pos_;
}

int translateOrnt(int orientation, turtleMove nextMove) {
    switch (nextMove) {
        case TURNRIGHT:
            return (orientation + 1) % ORIENTATION_COUNT;
        case TURNLEFT:
            return (orientation + ORIENTATION_COUNT - 1) % ORIENTATION_COUNT;
        default:
            return orientation;
    }
}

bool moveTurtle(QPointF& pos_, int& orientation) {
    static int delay_counter = 0;
    
    if (delay_counter > 0) {
        delay_counter--;
        return false;
    }

    bool is_bumped = detectObstacle(pos_, static_cast<Orientation>(orientation));
    bool is_at_end = atend(pos_.x(), pos_.y());
    
    turtleMove next_move = studentTurtleStep(is_bumped, is_at_end, &current_state);
    
    if (next_move == MOVE && !is_bumped) {
        pos_ = translatePos(pos_, static_cast<Orientation>(orientation));
        addVisit(pos_);
        displayVisits(retrieveVisitCount(pos_));
    } else if (next_move != STOP) {
        orientation = translateOrnt(orientation, next_move);
    }

    delay_counter = MOVE_DELAY;
    return true;
}
