#ifndef MONITOR_UTILS_H
#define MONITOR_UTILS_H

#include "ros/ros.h"
#include "monitor_interface.h"  // Need to include this for Pose, Endpoints, and Orientation types
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

    // Wall and position related functions
    static Endpoints canonicalizeWall(int x1, int y1, int x2, int y2);
    static bool wallsEqual(const Endpoints& wall1, const Endpoints& wall2);
    static Endpoints getWallInFront(int x, int y, Orientation o);
    static Endpoints wallBetween(const Pose& p1, const Pose& p2);
    static Orientation getMovementDirection(const Pose& from, const Pose& to);
};

#endif // MONITOR_UTILS_H
