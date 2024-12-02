#include "monitor_interface.h"  // Include this first
#include "monitor_utils.h"
#include <cmath>

Endpoints MonitorUtils::canonicalizeWall(int x1, int y1, int x2, int y2) {
    Endpoints wall;
    // Ensure wall segments are stored in canonical format:
    if (y1 == y2) {
        if (x1 <= x2) {
            wall = {x1, y1, x2, y2};
        } else {
            wall = {x2, y2, x1, y1};
        }
    } else {
        if (y1 <= y2) {
            wall = {x1, y1, x2, y2};
        } else {
            wall = {x2, y2, x1, y1};
        }
    }
    return wall;
}

bool MonitorUtils::wallsEqual(const Endpoints& wall1, const Endpoints& wall2) {
    return wall1.x1 == wall2.x1 && 
           wall1.y1 == wall2.y1 && 
           wall1.x2 == wall2.x2 && 
           wall1.y2 == wall2.y2;
}

Endpoints MonitorUtils::getWallInFront(int x, int y, Orientation o) {
    Endpoints wall;
    switch(o) {
        case WEST:  // x-1
            wall = canonicalizeWall(x, y, x, y+1);
            break;
        case NORTH:  // y-1
            wall = canonicalizeWall(x, y, x+1, y);
            break;
        case EAST:  // x+1
            wall = canonicalizeWall(x+1, y, x+1, y+1);
            break;
        case SOUTH:  // y+1
            wall = canonicalizeWall(x, y+1, x+1, y+1);
            break;
        default:
            ROS_ERROR("Invalid orientation in getWallInFront");
            wall = {0, 0, 0, 0};
    }
    return wall;
}

Endpoints MonitorUtils::wallBetween(const Pose& p1, const Pose& p2) {
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    
    if (std::abs(dx) + std::abs(dy) != 1) {
        ROS_ERROR("Non-adjacent points in wallBetween");
        return {0, 0, 0, 0};
    }

    if (dx == 1) {  // Moving East
        return canonicalizeWall(p1.x+1, p1.y, p1.x+1, p1.y+1);
    } else if (dx == -1) {  // Moving West
        return canonicalizeWall(p1.x, p1.y, p1.x, p1.y+1);
    } else if (dy == 1) {  // Moving South
        return canonicalizeWall(p1.x, p1.y+1, p1.x+1, p1.y+1);
    } else {  // Moving North
        return canonicalizeWall(p1.x, p1.y, p1.x+1, p1.y);
    }
}

Orientation MonitorUtils::getMovementDirection(const Pose& from, const Pose& to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    if (dx == 1) return EAST;
    if (dx == -1) return WEST;
    if (dy == 1) return SOUTH;
    if (dy == -1) return NORTH;
    
    ROS_ERROR("Invalid movement in getMovementDirection");
    return NORTH;  // Default
}
