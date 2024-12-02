#include "monitor_interface.h"

static bool maze_solved = false;
static Pose last_pose;
static Orientation last_orientation;

void tickInterrupt(ros::Time t) {}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (maze_solved) {
        if (x != last_pose.x || y != last_pose.y) {
            ROS_WARN("VIOLATION: Movement after maze completion at time %ld", t.toNSec());
        }
        if (o != last_orientation) {
            ROS_WARN("VIOLATION: Rotation after maze completion at time %ld", t.toNSec());
        }
    }
    
    last_pose = {x, y};
    last_orientation = o;
    ROS_INFO("[[%ld ns]] Position update: (%d,%d), orientation: %d", t.toNSec(), x, y, o);
}

void visitInterrupt(ros::Time t, int visits) {}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (maze_solved) {
        ROS_WARN("VIOLATION: Bump check after maze completion at time %ld", t.toNSec());
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (atEnd) {
        maze_solved = true;
        last_pose = {x, y};
        ROS_INFO("[[%ld ns]] Maze completed at position (%d,%d)", t.toNSec(), x, y);
    }
}
