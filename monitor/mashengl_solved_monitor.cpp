#include "monitor_interface.h"
#include "monitor_utils.h"

static bool maze_solved = false;
static Pose last_pose;
static Orientation last_orientation;

void tickInterrupt(ros::Time t) {
    // Not needed for solved monitoring
}

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

void visitInterrupt(ros::Time t, int visits) {
    // Not needed for solved monitoring
}

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_solved_monitor");
    MonitorUtils::logMonitorStart("Solved Monitor (mashengl)");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
