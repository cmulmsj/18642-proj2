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

//file static map for keeping track of visit counts for each tile
static uint8_t visits[30][30] = {{0}};

// enums for current_state FSM
enum FSM_STATES { FORWARD_STATE = 0, CHECK_UNVISITED_STATE = 1, CHECK_UNBUMPED_STATE = 2};

// enums for local directionality
enum LOCAL_DIRECTION { LOCAL_WEST = 0, LOCAL_NORTH = 1, LOCAL_EAST = 2, LOCAL_SOUTH = 3 };

/* This procedure gets the visit array position (x,y)'s value */
uint8_t getVisit(coordinate local_coord) { 
  return visits[local_coord.x][local_coord.y]; 
}

/* This procedure sets the visit array position (x,y) with setVal*/
void setVisit(coordinate local_coord, uint8_t setVal) {
  visits[local_coord.x][local_coord.y] = setVal;
  return;
}

/*
 * This procedure takes in a compass_orientation and returns an orientation
 * representation after the input orientation has been rotated clockwise 90
 * degrees
 */
LOCAL_DIRECTION rotateLocalClockwise(LOCAL_DIRECTION current_local_direction) {
  switch (current_local_direction) {
    case LOCAL_NORTH: {
      current_local_direction = LOCAL_EAST;
      break;
    }
    case LOCAL_EAST: {
      current_local_direction = LOCAL_SOUTH;
      break;
    }
    case LOCAL_SOUTH: {
      current_local_direction = LOCAL_WEST;
      break;
    }
    case LOCAL_WEST: {
      current_local_direction = LOCAL_NORTH;
      break;
    }
    default: {
      ROS_ERROR("current_local_direction is in undetermined state");
      break;
    }
  }
  return current_local_direction;
}

/*
 * This procedure takes in a compass_orientation and returns an orientation
 * representation after the input orientation has been rotated counter clockwise
 * 90 degrees
 */
LOCAL_DIRECTION rotateLocalCounterClockwise(LOCAL_DIRECTION current_local_direction) {
  switch (current_local_direction) {
    case LOCAL_NORTH: {
      current_local_direction = LOCAL_WEST;
      break;
    }
    case LOCAL_EAST: {
      current_local_direction = LOCAL_NORTH;
      break;
    }
    case LOCAL_SOUTH: {
      current_local_direction = LOCAL_EAST;
      break;
    }
    case LOCAL_WEST: {
      current_local_direction = LOCAL_SOUTH;
      break;
    }
    default: {
      ROS_ERROR("current_local_direction is in undetermined state");
      break;
    }
  }
  return current_local_direction;
}


/*
 * This procedure takes in the current map state (pos_) and the intended
 * direction of the turtle (compass_orientation)
 * and updates the turtle position to move one step in the compass orientation.
 * There is no return value for this function
 */
coordinate updateLocalTurtlePosition(coordinate current_location, LOCAL_DIRECTION local_orientation) {
  switch (local_orientation) {
    case LOCAL_NORTH: {
      current_location.y -= 1;
      break;
    }
    case LOCAL_EAST: {
      current_location.x += 1;
      break;
    }
    case LOCAL_SOUTH: {
      current_location.y += 1;
      break;
    }
    case LOCAL_WEST: {
      current_location.x -= 1;
      break;
    }
    default: {
      ROS_ERROR("local compass_orientation is in undetermined state");
      break;
    }
  }
  return current_location;
}

bool checkIdealForward(bool bumpedIntoWall, coordinate current_location, LOCAL_DIRECTION local_orientation) {
  coordinate check_location = {x: current_location.x, y: current_location.y};
  switch (local_orientation) {
    case LOCAL_NORTH: {
      check_location.y -= 1;
      break;
    }
    case LOCAL_EAST: {
      check_location.x += 1;
      break;
    }
    case LOCAL_SOUTH: {
      check_location.y += 1;
      break;
    }
    case LOCAL_WEST: {
      check_location.x -= 1;
      break;
    }
    default: {
      ROS_ERROR("local compass_orientation is in undetermined state");
      break;
    }
  }
  uint8_t visitCount = getVisit(check_location);
  //ROS_ERROR("CURR LOCATION (%d, %d)", current_location.x, current_location.y);
  //ROS_ERROR("CHECK (%d, %d)", check_location.x, check_location.y);
  //ROS_ERROR("VISIT COUNT = %d, BUMPED =%d", visitCount, bumpedIntoWall);
  return (visitCount == 0) && !bumpedIntoWall;
}

LOCAL_DIRECTION leastVisitedNeighbor(coordinate current_location, bool bumpedMap[]) {
  uint8_t minVisits = 10000;
  LOCAL_DIRECTION minDirection;
  
  for(int i = 0; i < 4; i++){
    coordinate check_location = {x: current_location.x, y: current_location.y};
    switch (i) {
      case LOCAL_NORTH: {
        check_location.y -= 1;
        break;
      }
      case LOCAL_EAST: {
        check_location.x += 1;
        break;
      }
      case LOCAL_SOUTH: {
        check_location.y += 1;
        break;
      }
      case LOCAL_WEST: {
        check_location.x -= 1;
        break;
      }
      default: {
        ROS_ERROR("local compass_orientation is in undetermined state");
        break;
      }
    }
    uint8_t visitCount = getVisit(check_location);
    //ROS_ERROR("CHECKING (%d, %d)", check_location.x, check_location.y);
    if(!bumpedMap[i] && visitCount < minVisits){
      minVisits = visitCount;
      //ROS_ERROR("min direction is %d", i);
      minDirection = (LOCAL_DIRECTION)i;
    }
  }
  return minDirection;
}
/* 
 * This procedure takes in a bumpedIntoWall (if turtle has bumped into wall) and 
 * at_end which represents if the turtle is at the goal and outputs a turtleMove struct
 * detailing what the next move is. It will use an algorithm where it will follow its
 * right wall to the goal, and uses an internal map to keep track of where it is.
 * The output Move is either FORWARD, telling the global turtle to move forward,
 * LEFT/RIGHT, which tells the global turtle to turn left or right. It also contains
 * variables to keep track of how many times it visited the tile and if the action is valid
 */
turtleMove studentTurtleStep(bool bumpedIntoWall, bool at_end) {
  //file static variable for relative location in turtle visit map
  static coordinate current_location = {x : 14, y : 14};
  static LOCAL_DIRECTION current_local_direction = LOCAL_NORTH;
  static uint8_t directionsChecked = 0;
  turtleMove futureMove;
  static FSM_STATES current_state = FORWARD_STATE;
  static bool bumpedMap[4] = {false, false, false, false};

  // bigger number slows down simulation so you can see what's happening
  const uint8_t TIMEOUT = 5;
  static uint8_t timeout_counter;
  //S4
  if (at_end) {
    //T1.3
    futureMove.validAction = false;
    return futureMove;
  }
  if (timeout_counter == 0) {
    //CF1
    bool noBumpAndUnvisited = checkIdealForward(bumpedIntoWall, current_location, current_local_direction);
    // test future move on wall
    switch (current_state) {
      //S1
      case FORWARD_STATE: {
        if(noBumpAndUnvisited){
          //T1.1
          current_state = FORWARD_STATE;
        }
        else {
          //T1.2
          current_state = CHECK_UNVISITED_STATE;
          bumpedMap[(int)current_local_direction] = bumpedIntoWall;
          current_local_direction = rotateLocalClockwise(current_local_direction);
          futureMove.action = RIGHT;
          directionsChecked++;
        }
        break;
      }
      //S2
      case CHECK_UNVISITED_STATE: {
        if(noBumpAndUnvisited){
          //T2.1
          current_state = FORWARD_STATE;
          directionsChecked = 0;
          memset(bumpedMap, false, 4);
        }
        else if(directionsChecked < 4){
          //T2.2
          current_state = CHECK_UNVISITED_STATE;
          bumpedMap[(int)current_local_direction] = bumpedIntoWall;
          current_local_direction = rotateLocalClockwise(current_local_direction);
          futureMove.action = RIGHT;
          directionsChecked++;
        }
        else{
          //T2.3
          //directionsChecked == 4
          current_state = CHECK_UNBUMPED_STATE;
          bumpedMap[(int)current_local_direction] = bumpedIntoWall;
          current_local_direction = rotateLocalClockwise(current_local_direction);
          futureMove.action = RIGHT;
          directionsChecked = 0;
        }
        break;
      }
      //S3
      case CHECK_UNBUMPED_STATE: {
        LOCAL_DIRECTION desireDirection = leastVisitedNeighbor(current_location, bumpedMap);
        if(current_local_direction != desireDirection){
          //T3.1
          current_state = CHECK_UNBUMPED_STATE;
          current_local_direction = rotateLocalClockwise(current_local_direction);
          futureMove.action = RIGHT;
        } else {
          //T3.3
          current_state = FORWARD_STATE;
          directionsChecked = 0;
          memset(bumpedMap, false, 4);
        }
        break;
      }
    }
    //futureMove.visitCount = getVisit(current_location);
    // update position if valid move
    if (current_state == FORWARD_STATE) {
      //ROS_ERROR("Setting Visit (%d,%d) to %d", current_location.x, current_location.y, getVisit(current_location));
      current_location = updateLocalTurtlePosition(current_location, (LOCAL_DIRECTION)current_local_direction);
      setVisit(current_location, getVisit(current_location)+1); 
      futureMove.visitCount = getVisit(current_location);
      //increment visited count by 1   
      futureMove.action = FORWARD;
    }
    timeout_counter = TIMEOUT;
    futureMove.validAction = true;
    return futureMove;
  }
  timeout_counter -= 1;
  futureMove.validAction = false;
  return futureMove;
}
