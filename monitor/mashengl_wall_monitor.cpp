#include "monitor_interface.h"
#include "monitor_utils.h"
#include <queue>

// Circular buffer to store recent bump checks
static const int MAX_BUMP_MEMORY = 4;  // Store last 4 bump checks (for a full rotation)
static std::queue<std::pair<Endpoints, bool>> bump_memory;
static Pose last_pose;
static bool first_move = true;

bool bumpedInMemory(const Endpoints& wall) {
    std::queue<std::pair<Endpoints, bool>> temp = bump_memory;
    while (!temp.empty()) {
        auto [stored_wall, was_bumped] = temp.front();
        temp.pop();
        if (MonitorUtils::wallsEqual(stored_wall, wall)) {
            return was_bumped;
        }
    }
    return true;  // If wall not found in memory, assume blocked for safety
}

void tickInterrupt(ros::Time t) {
    // Not needed for wall monitoring
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (first_move) {
        last_pose = {x, y};
        first_move = false;
        return;
    }

    // Check if position changed
    if (last_pose.x != x || last_pose.y != y) {
        Pose current_pose = {x, y};
        
        // Get wall between previous and current position
        Endpoints crossed_wall = MonitorUtils::wallBetween(last_pose, current_pose);
        
        // Check if we tried to cross this wall
        bool wall_blocked = bumpedInMemory(crossed_wall);
        
        if (wall_blocked) {
            ROS_WARN("VIOLATION: Crossed blocked wall from (%d,%d) to (%d,%d) at time %ld", 
                     last_pose.x, last_pose.y, x, y, t.toNSec());
        } else {
            ROS_INFO("[[%ld ns]] Valid movement through checked wall from (%d,%d) to (%d,%d)", 
                     t.toNSec(), last_pose.x, last_pose.y, x, y);
        }
        
        // Clear bump memory after moving
        while (!bump_memory.empty()) {
            bump_memory.pop();
        }
    }
    
    last_pose = {x, y};
}

void visitInterrupt(ros::Time t, int visits) {
    // Not needed for wall monitoring
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    Endpoints wall = MonitorUtils::canonicalizeWall(x1, y1, x2, y2);
    
    // Store bump check in circular buffer
    bump_memory.push({wall, bumped});
    if (bump_memory.size() > MAX_BUMP_MEMORY) {
        bump_memory.pop();
    }
    
    ROS_INFO("[[%ld ns]] Bump check: (%d,%d)->(%d,%d) = %s", 
             t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    // Not needed for wall monitoring
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mashengl_wall_monitor");
    MonitorUtils::logMonitorStart("Wall Monitor (mashengl)");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
