#include "monitor_interface.h"
#include "monitor_utils.h"

static Pose current_pose;
static Orientation current_orientation;
static bool pose_initialized = false;

void tickInterrupt(ros::Time t) {
    // Not needed for face monitoring
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    current_pose = {x, y};
    current_orientation = o;
    pose_initialized = true;
    ROS_INFO("[[%ld ns]] Updated position: (%d,%d), orientation: %d", 
             t.toNSec(), x, y, o);
}

void visitInterrupt(ros::Time t, int visits) {
    // Not needed for face monitoring
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (!pose_initialized) {
        ROS_WARN("VIOLATION: Bump check before position initialization at time %ld", t.toNSec());
        return;
    }

    // Get wall segment that should be in front of turtle
    Endpoints expected_wall = MonitorUtils::getWallInFront(current_pose.x, current_pose.y, 
                                                         current_orientation);
    
    // Get actual wall being checked (in canonical form)
    Endpoints checked_wall = MonitorUtils::canonicalizeWall(x1, y1, x2, y2);
    
    // Compare walls
    if (!MonitorUtils::wallsEqual(expected_wall, checked_wall)) {
        ROS_WARN("VIOLATION: Checking wall (%d,%d)->(%d,%d) while facing %d at (%d,%d) at time %ld", 
                 x1, y1, x2, y2, current_orientation, current_pose.x, current_pose.y, t.toNSec());
    } else {
        ROS_INFO("[[%ld ns]] Valid bump check for wall (%d,%d)->(%d,%d)", 
                 t.toNSec(), x1, y1, x2, y2);
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    // Not needed for face monitoring
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_face_monitor");
    MonitorUtils::logMonitorStart("Face Monitor (mashengl)");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
