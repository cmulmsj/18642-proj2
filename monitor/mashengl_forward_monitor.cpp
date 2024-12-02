#include "monitor_interface.h"
#include "monitor_utils.h"

static Pose last_pose;
static Orientation last_orientation;
static bool has_turned = false;
static bool first_move = true;
static bool tick_active = false;

void tickInterrupt(ros::Time t) {
    if (!tick_active) {
        tick_active = true;
        has_turned = false;
    } else {
        tick_active = false;
    }
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (first_move) {
        last_pose = {x, y};
        last_orientation = o;
        first_move = false;
        ROS_INFO("[[%ld ns]] Initial position: (%d,%d), orientation: %d", 
                 t.toNSec(), x, y, o);
        return;
    }

    // Check if position changed
    if (last_pose.x != x || last_pose.y != y) {
        // Verify movement direction matches orientation
        Pose current_pose = {x, y};
        Orientation movement_dir = MonitorUtils::getMovementDirection(last_pose, current_pose);
        
        if (last_orientation != movement_dir) {
            ROS_WARN("VIOLATION: Movement direction (%d) doesn't match orientation (%d) at time %ld", 
                     movement_dir, last_orientation, t.toNSec());
        }
        
        if (has_turned) {
            ROS_WARN("VIOLATION: Movement occurred in same tick as rotation at time %ld",
                     t.toNSec());
        }
        
        ROS_INFO("[[%ld ns]] Movement from (%d,%d) to (%d,%d), orientation: %d", 
                 t.toNSec(), last_pose.x, last_pose.y, x, y, o);
    }
    
    // Check for rotation
    if (last_orientation != o) {
        has_turned = true;
        ROS_INFO("[[%ld ns]] Rotation from %d to %d", t.toNSec(), last_orientation, o);
    }
    
    last_pose = {x, y};
    last_orientation = o;
}

void visitInterrupt(ros::Time t, int visits) {
    // Not needed for forward monitoring
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    // Not needed for forward monitoring
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    // Not needed for forward monitoring
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "forward_monitor");
    MonitorUtils::logMonitorStart("Forward Monitor");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
