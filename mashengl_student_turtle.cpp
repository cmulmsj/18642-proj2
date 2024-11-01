/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: GL Ma
 * ANDREW ID: mashengl
 * LAST UPDATE: 10/27/2024
 */

#include "stdint.h"
#include "student.h"

// Visit tracking matrix for maze navigation
static uint8_t visits[30][30] = {{0}};

// Robot navigation states
enum FSM_STATES { FORWARD_STATE = 0, CHECK_UNVISITED_STATE = 1, CHECK_UNBUMPED_STATE = 2};

// Local directional system
enum LOCAL_DIRECTION { LOCAL_WEST = 0, LOCAL_NORTH = 1, LOCAL_EAST = 2, LOCAL_SOUTH = 3 };

/* Get visit count for a cell */
uint8_t getVisit(coordinate local_coord) { 
  return visits[local_coord.x][local_coord.y]; 
}

/* Update visit count for a cell */
void setVisit(coordinate local_coord, uint8_t setVal) {
  visits[local_coord.x][local_coord.y] = setVal;
  return;
}

/* Calculate next grid position based on current position and direction */
coordinate updateLocalTurtlePosition(coordinate current_location, LOCAL_DIRECTION local_orientation) {
  switch (local_orientation) {
    case LOCAL_NORTH: {
      current_location.y -= 1U;
      break;
    }
    case LOCAL_EAST: {
      current_location.x += 1U;
      break;
    }
    case LOCAL_SOUTH: {
      current_location.y += 1U;
      break;
    }
    case LOCAL_WEST: {
      current_location.x -= 1U;
      break;
    }
    default: {
      ROS_ERROR("local compass_orientation is in undetermined state");
      break;
    }
  }
  return current_location;
}

/* Navigation algorithm implementation */
turtleMove studentTurtleStep(bool bumpedIntoWall, bool at_end) {
  static coordinate current_location = {x: 14, y: 14};
  static LOCAL_DIRECTION current_local_direction = LOCAL_NORTH;
  static uint8_t directionsChecked = 0;
  turtleMove futureMove;
  static FSM_STATES current_state = FORWARD_STATE;
  static bool bumpedMap[4] = {false};

  const uint8_t TIMEOUT = 5;
  static uint8_t timeout_counter;

  if (at_end) {
    futureMove.validAction = false;
    return futureMove;
  }

  if (timeout_counter == 0) {
    coordinate check_location = current_location;
    switch (current_local_direction) {
      case LOCAL_NORTH: {
        check_location.y -= 1U;
        break;
      }
      case LOCAL_EAST: {
        check_location.x += 1U;
        break;
      }
      case LOCAL_SOUTH: {
        check_location.y += 1U;
        break;
      }
      case LOCAL_WEST: {
        check_location.x -= 1U;
        break;
      }
      default: {
        ROS_ERROR("local compass_orientation is in undetermined state");
        break;
      }
    }

    uint8_t visitCount = getVisit(check_location);
    bool canMoveForward = !bumpedIntoWall && (visitCount == 0);

    switch (current_state) {
      case FORWARD_STATE: {
        if(canMoveForward) {
          current_state = FORWARD_STATE;
        } else {
          current_state = CHECK_UNVISITED_STATE;
          bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
          current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
          futureMove.action = RIGHT;
          directionsChecked++;
        }
        break;
      }
      case CHECK_UNVISITED_STATE: {
        if(canMoveForward) {
          current_state = FORWARD_STATE;
          directionsChecked = 0;
          for(int i = 0; i < 4; i++) {
            bumpedMap[i] = false;
          }
        } else if(directionsChecked < 4) {
          bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
          current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
          futureMove.action = RIGHT;
          directionsChecked++;
        } else {
          current_state = CHECK_UNBUMPED_STATE;
          bumpedMap[static_cast<int>(current_local_direction)] = bumpedIntoWall;
          current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
          futureMove.action = RIGHT;
          directionsChecked = 0;
        }
        break;
      }
      case CHECK_UNBUMPED_STATE: {
        uint8_t minVisits = UINT8_MAX;
        LOCAL_DIRECTION minDirection = current_local_direction;
        
        for(int i = 0; i < 4; i++) {
          coordinate temp_location = current_location;
          switch (i) {
            case LOCAL_NORTH: temp_location.y -= 1U; break;
            case LOCAL_EAST:  temp_location.x += 1U; break;
            case LOCAL_SOUTH: temp_location.y += 1U; break;
            case LOCAL_WEST:  temp_location.x -= 1U; break;
          }
          uint8_t visits = getVisit(temp_location);
          if(!bumpedMap[i] && visits < minVisits) {
            minVisits = visits;
            minDirection = static_cast<LOCAL_DIRECTION>(i);
          }
        }
        
        if(current_local_direction != minDirection) {
          current_local_direction = static_cast<LOCAL_DIRECTION>((static_cast<int>(current_local_direction) + 1) % 4);
          futureMove.action = RIGHT;
        } else {
          current_state = FORWARD_STATE;
          directionsChecked = 0;
          for(int i = 0; i < 4; i++) {
            bumpedMap[i] = false;
          }
        }
        break;
      }
    }

    if (current_state == FORWARD_STATE) {
      current_location = updateLocalTurtlePosition(current_location, current_local_direction);
      setVisit(current_location, getVisit(current_location) + 1U);
      futureMove.visitCount = getVisit(current_location);
      futureMove.action = FORWARD;
    }

    timeout_counter = TIMEOUT;
    futureMove.validAction = true;
    return futureMove;
  }

  timeout_counter--;
  futureMove.validAction = false;
  return futureMove;
}
