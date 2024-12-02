#include "monitor_interface.h"
#include <cstdlib>

static Pose last_pose;
static Orientation last_orientation;
static bool has_turned = false;
static bool first_move = true;
static bool tick_active = false;

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
        Orientation movement_dir = getMovementDirection(last_pose, current_pose);
        
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

void visitInterrupt(ros::Time t, int visits) {}
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {}
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_forward_monitor");
    ROS_WARN("Monitor Forward Monitor (mashengl) is running at %s", ctime(0));
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
