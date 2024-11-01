/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "student.h"

enum DIRECTION { WEST = 0, NORTH = 1, EAST = 2, SOUTH = 3 };

bool bumpTest(QPointF &pos_, DIRECTION compass_orientation) {
  coordinate bumptest1, bumptest2;
  // Fix float to uint8_t conversions
  bumptest1.x = static_cast<uint8_t>(std::floor(pos_.x()));
  bumptest2.x = static_cast<uint8_t>(std::floor(pos_.x()));
  bumptest1.y = static_cast<uint8_t>(std::floor(pos_.y()));
  bumptest2.y = static_cast<uint8_t>(std::floor(pos_.y()));

  switch (compass_orientation) {
    case WEST: {
      bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
      break;
    }
    case NORTH: {
      bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
      break;
    }
    case EAST: {
      bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
      bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
      bumptest1.x = static_cast<uint8_t>(bumptest1.x + 1U);
      break;
    }
    case SOUTH: {
      bumptest2.x = static_cast<uint8_t>(bumptest2.x + 1U);
      bumptest2.y = static_cast<uint8_t>(bumptest2.y + 1U);
      bumptest1.y = static_cast<uint8_t>(bumptest1.y + 1U);
      break;
    }
    default: {
      ROS_ERROR("compass_orientation is in undetermined state");
      break;
    }
  }
  return bumped(bumptest1.x, bumptest1.y, bumptest2.x, bumptest2.y);
}

bool moveTurtle(QPointF& pos_, int& compass_orientation) {
  bool bumped = bumpTest(pos_, static_cast<DIRECTION>(compass_orientation));
  // Fix float to int conversions
  bool at_end = atend(static_cast<int>(std::floor(pos_.x())), 
                     static_cast<int>(std::floor(pos_.y())));
  turtleMove nextMove = studentTurtleStep(bumped, at_end);
  if(nextMove.validAction) {
    translatePosAndOrientation(nextMove, pos_, compass_orientation);
  }
  return nextMove.validAction;
}

void translatePosAndOrientation(turtleMove nextMove, QPointF& pos_, int& compass_orientation) {
  switch (nextMove.action) {
    case FORWARD: {
      pos_ = translatePos(pos_, nextMove, compass_orientation);
      displayVisits(nextMove.visitCount);
      break;
    }
    case LEFT: 
    case RIGHT: {
      compass_orientation = translateOrnt(compass_orientation, nextMove);
      break;
    }
    default: {
      ROS_ERROR("nextMove.action is invalid action");
      break;
    }
  }
}

QPointF translatePos(QPointF pos_, turtleMove /* nextMove */, int compass_orientation) {
  switch (compass_orientation) {
    case NORTH: {
      pos_.setY(pos_.y() - 1.0);
      break;
    }
    case EAST: {
      pos_.setX(pos_.x() + 1.0);
      break;
    }
    case SOUTH: {
      pos_.setY(pos_.y() + 1.0);
      break;
    }
    case WEST: {
      pos_.setX(pos_.x() - 1.0);
      break;
    }
    default: {
      ROS_ERROR("compass_orientation is in undetermined state");
      break;
    }
  }
  return pos_;
}

DIRECTION rotateClockwise(DIRECTION compass_orientation) {
  DIRECTION result;
  switch (compass_orientation) {
    case NORTH:
      result = EAST;
      break;
    case EAST:
      result = SOUTH;
      break;
    case SOUTH:
      result = WEST;
      break;
    case WEST:
      result = NORTH;
      break;
    default:
      ROS_ERROR("compass_orientation is in undetermined state");
      result = NORTH;
      break;
  }
  return result;
}

DIRECTION rotateCounterClockwise(DIRECTION compass_orientation) {
  DIRECTION result;
  switch (compass_orientation) {
    case NORTH:
      result = WEST;
      break;
    case EAST:
      result = NORTH;
      break;
    case SOUTH:
      result = EAST;
      break;
    case WEST:
      result = SOUTH;
      break;
    default:
      ROS_ERROR("compass_orientation is in undetermined state");
      result = NORTH;
      break;
  }
  return result;
}

int translateOrnt(int orientation, turtleMove nextMove) {
  int result;
  switch (nextMove.action) {
    case LEFT:
      result = static_cast<int>(rotateCounterClockwise(static_cast<DIRECTION>(orientation)));
      break;
    case RIGHT:
      result = static_cast<int>(rotateClockwise(static_cast<DIRECTION>(orientation)));
      break;
    default:
      ROS_ERROR("nextMove is Unknown");
      result = orientation;
      break;
  }
  return result;
}
