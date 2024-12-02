#include "monitor_interface.h"

static Pose current_pose;
static bool maze_completed = false;
static bool pose_initialized = false;

void tickInterrupt(ros::Time t) {
    // Output success message to flush warnings if maze is completed
    if (maze_completed) {
        ROS_WARN("Successful atEnd");
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_pose = {x, y};
    pose_initialized = true;
    ROS_INFO("[[%ld ns]] Position updated: (%d,%d)", t.toNSec(), x, y);
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (!pose_initialized) {
        ROS_WARN("VIOLATION: atEnd check before position initialization at time %ld", t.toNSec());
        return;
    }

    // Check if atEnd call matches current position
    if (x != current_pose.x || y != current_pose.y) {
        ROS_WARN("VIOLATION: atEnd check at (%d,%d) while turtle is at (%d,%d) at time %ld", 
                 x, y, current_pose.x, current_pose.y, t.toNSec());
    } else {
        ROS_INFO("[[%ld ns]] Valid atEnd check at (%d,%d)", t.toNSec(), x, y);
    }

    // Update completion status
    if (atEnd) {
        maze_completed = true;
    }
}
