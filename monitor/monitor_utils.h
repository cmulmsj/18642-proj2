#ifndef MONITOR_UTILS_H
#define MONITOR_UTILS_H

#include "monitor_interface.h"
#include <ctime>
#include <string>
#include <cstdio>

class MonitorUtils {
public:
    // Standardized monitor startup message
    static void logMonitorStart(const std::string& monitorName) {
        time_t now = time(0);
        char* dt = ctime(&now);
        std::string message = "Monitor " + monitorName + " is running at " + dt;
        // Remove newline from ctime
        if (!message.empty() && message[message.length()-1] == '\n') {
            message.erase(message.length()-1);
        }
        
        // Log to both ROS and stderr
        ROS_WARN("%s", message.c_str());
        fprintf(stderr, "%s\n", message.c_str());
    }

    // Wall segment comparison and canonicalization
    static Endpoints canonicalizeWall(int x1, int y1, int x2, int y2) {
        Endpoints wall;
        // Ensure wall segments are stored in canonical format:
        // If y coordinates are equal, x1 should be less than x2
        // If x coordinates are equal, y1 should be less than y2
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

    // Check if two wall segments are the same
    static bool wallsEqual(const Endpoints& wall1, const Endpoints& wall2) {
        return wall1.x1 == wall2.x1 && 
               wall1.y1 == wall2.y1 && 
               wall1.x2 == wall2.x2 && 
               wall1.y2 == wall2.y2;
    }

    // Get wall segment that turtle would be facing based on position and orientation
    static Endpoints getWallInFront(int x, int y, Orientation o) {
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

    // Get wall between two adjacent positions
    static Endpoints wallBetween(const Pose& p1, const Pose& p2) {
        // Ensure points are adjacent
        int dx = p2.x - p1.x;
        int dy = p2.y - p1.y;
        
        if (abs(dx) + abs(dy) != 1) {
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

    // Get movement direction between two adjacent positions
    static Orientation getMovementDirection(const Pose& from, const Pose& to) {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        
        if (dx == 1) return EAST;
        if (dx == -1) return WEST;
        if (dy == 1) return SOUTH;
        if (dy == -1) return NORTH;
        
        ROS_ERROR("Invalid movement in getMovementDirection");
        return NORTH;  // Default
    }
};

#endif // MONITOR_UTILS_H
