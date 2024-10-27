/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

// enums for directionality
enum DIRECTION { WEST = 0, NORTH = 1, EAST = 2, SOUTH = 3 };

/*
 * This procedure takes in the current map (pos_) and the turtle's orientation
 * (compass_orientation) to figure out if
 * the turtle will bump into a wall on the next move. This procedure will return
 * true if it will bump into something
 * and false if not, taking advantage of the function bumped.
 */
bool bumpTest(QPointF &pos_, DIRECTION compass_orientation) {
  coordinate bumptest1, bumptest2;
  bumptest1.x = pos_.x();
  bumptest2.x = pos_.x();
  bumptest1.y = pos_.y();
  bumptest2.y = pos_.y();
  switch (compass_orientation) {
  case WEST: {
    bumptest2.y += 1;
    break;
  }
  case NORTH: {
    bumptest2.x += 1;
    break;
  }
  case EAST: {
    bumptest2.x += 1;
    bumptest2.y += 1;
    bumptest1.x += 1;
    break;
  }
  case SOUTH: {
    bumptest2.x += 1;
    bumptest2.y += 1;
    bumptest1.y += 1;
    break;
  }
  default: {
    ROS_ERROR("compass_orientation is in undetermined state");
    break;
  }
  }
  return bumped(bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
}

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF& pos_, int& compass_orientation)
{
  bool bumped = bumpTest(pos_, (DIRECTION)compass_orientation); 
  bool at_end = atend(pos_.x(), pos_.y());
  turtleMove nextMove = studentTurtleStep(bumped, at_end); 
  if(nextMove.validAction){
    translatePosAndOrientation(nextMove, pos_, compass_orientation);
  }
  return nextMove.validAction;
}

/* This procedure takes in the nextMove's action field and updates by reference the position
 * and compass orientaiton of the turtle.
 */
void translatePosAndOrientation(turtleMove nextMove, QPointF& pos_, int& compass_orientation){
  switch (nextMove.action){
    case FORWARD: {
      pos_ = translatePos(pos_, nextMove, compass_orientation);
      displayVisits(nextMove.visitCount);
      break;
    }
    case LEFT: {
      compass_orientation = translateOrnt(compass_orientation, nextMove);
      break;
    }
    case RIGHT: {
      compass_orientation = translateOrnt(compass_orientation, nextMove);
      break;
    }
    default: {
      ROS_ERROR("nextMove.action is invalid action");
      break;
    }
  }
  return;
}
/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove, int compass_orientation) {
  switch (compass_orientation) {
  case NORTH: {
    pos_.setY(pos_.y() - 1);
    break;
  }
  case EAST: {
    pos_.setX(pos_.x() + 1);
    break;
  }
  case SOUTH: {
    pos_.setY(pos_.y() + 1);
    break;
  }
  case WEST: {
    pos_.setX(pos_.x() - 1);
    break;
  }
  default: {
    ROS_ERROR("compass_orientation is in undetermined state");
    break;
  }
  }
  return pos_;
}

/*
 * This procedure takes in a compass_orientation and returns an orientation
 * representation after the input orientation has been rotated clockwise 90
 * degrees
 */
DIRECTION rotateClockwise(DIRECTION compass_orientation) {
  switch (compass_orientation) {
  case NORTH: {
    compass_orientation = EAST;
    break;
  }
  case EAST: {
    compass_orientation = SOUTH;
    break;
  }
  case SOUTH: {
    compass_orientation = WEST;
    break;
  }
  case WEST: {
    compass_orientation = NORTH;
    break;
  }
  default: {
    ROS_ERROR("compass_orientation is in undetermined state");
    break;
  }
  }
  return compass_orientation;
}

/*
 * This procedure takes in a compass_orientation and returns an orientation
 * representation after the input orientation has been rotated counter clockwise
 * 90 degrees
 */
DIRECTION rotateCounterClockwise(DIRECTION compass_orientation) {
  switch (compass_orientation) {
  case NORTH: {
    compass_orientation = WEST;
    break;
  }
  case EAST: {
    compass_orientation = NORTH;
    break;
  }
  case SOUTH: {
    compass_orientation = EAST;
    break;
  }
  case WEST: {
    compass_orientation = SOUTH;
    break;
  }
  default: {
    ROS_ERROR("compass_orientation is in undetermined state");
    break;
  }
  }
  return compass_orientation;
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  switch (nextMove.action){
    case LEFT: {
      return rotateCounterClockwise((DIRECTION)orientation);
    }
    case RIGHT: {
      return rotateClockwise((DIRECTION)orientation);
    }
    default: {
      ROS_ERROR("nextMove is Unknown");
      break;

    }
  }
}

