#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm> 


#include "utils.hpp"

namespace path_planning {


struct RRTNode {
    int x, y;           // Grid coordinates
    int parent_id = -1; // Index of the parent node in the tree vector
    
};

class RRTPlanner {
public:
    RRTPlanner(ros::NodeHandle& nh);
    nav_msgs::Path planPath(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            const nav_msgs::OccupancyGrid& map);

private:
    ros::NodeHandle nh_;

    //RRT Parameters 
    double step_size_;         // Max distance to extend from nearest node (in grid units)
    int max_iterations_;     // Max attempts to find path
    double goal_bias_;         // Probability (0-1) of sampling the goal directly
    double goal_tolerance_dist_; // Euclidean distance threshold to consider goal reached (in grid units)
    int obstacle_threshold_; // Occupancy threshold from map data 

    // Sample a random point based on bias
    Node sampleRandomNode(const nav_msgs::OccupancyGrid& map, const Node& goal_node);
    // Finds the index of the node in the tree vector closest to the sample
    int findNearestNode(const std::vector<RRTNode>& tree, const Node& sample_node);
    // Generates a new node by stepping from nearest towards sample
    RRTNode steer(const RRTNode& nearest_node, const Node& sample_node, double step_size);
    // Checks the straight line path between two nodes for collisions
    bool isCollisionFree(const RRTNode& node1, const RRTNode& node2, const nav_msgs::OccupancyGrid& map, int obstacle_threshold);
    // Reconstructs the path from the tree
    nav_msgs::Path reconstructRRTPath(const std::vector<RRTNode>& tree, int goal_node_idx, const nav_msgs::OccupancyGrid& map, const geometry_msgs::PoseStamped& start_pose /* for header */);

    // --- Random Number Generation ---
    std::mt19937 rng_; // Random number generator engine
    std::uniform_real_distribution<double> goal_bias_dist_; // For goal bias check
    std::uniform_int_distribution<int> x_dist_; // For sampling x coord
    std::uniform_int_distribution<int> y_dist_; // For sampling y coord

    // Map info 
    int map_width_;
    int map_height_;
};


RRTPlanner::RRTPlanner(ros::NodeHandle& nh) : nh_(nh), rng_(std::random_device{}()) {
    // Load parameters with default values
    nh_.param<double>("rrt_step_size", step_size_, 5.0); // Adjust default as needed (grid units)
    nh_.param<int>("rrt_max_iterations", max_iterations_, 10000); // Adjust default
    nh_.param<double>("rrt_goal_bias", goal_bias_, 0.1); // 10% chance to sample goal
    nh_.param<double>("rrt_goal_tolerance_dist", goal_tolerance_dist_, 5.0); // Grid units distance
    nh_.param<int>("obstacle_threshold", obstacle_threshold_, 50); // Use same threshold as others [cite: 1, 4]

    // Initialize distributions
    goal_bias_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
    map_width_ = 0; //We will modify once map is recieved
    map_height_ = 0;
}


nav_msgs::Path RRTPlanner::planPath(
    const geometry_msgs::PoseStamped& start_pose,
    const geometry_msgs::PoseStamped& goal_pose,
    const nav_msgs::OccupancyGrid& map) {

    ROS_INFO("RRTPlanner: Starting path planning...");

    // Updationg map dimensions and distributions if needed
    if (map.info.width != map_width_ || map.info.height != map_height_) {
        map_width_ = map.info.width;
        map_height_ = map.info.height;
        x_dist_ = std::uniform_int_distribution<int>(0, map_width_ - 1);
        y_dist_ = std::uniform_int_distribution<int>(0, map_height_ - 1);
        ROS_INFO("RRTPlanner: Map dimensions updated to %d x %d", map_width_, map_height_);
    }
    if (map_width_ == 0 || map_height_ == 0) {
         ROS_ERROR("RRTPlanner: Invalid map dimensions received.");
         return nav_msgs::Path(); // Return empty path
    }


    // Convert start and goal to grid coordinates using utility 
    Node start_node_grid, goal_node_grid;
    mapToGrid(map, start_pose, start_node_grid.x, start_node_grid.y);
    mapToGrid(map, goal_pose, goal_node_grid.x, goal_node_grid.y);


    // Check if start or goal are invalid
     if (!isValidCell(start_node_grid.x, start_node_grid.y, map) || isObstacle(start_node_grid.x, start_node_grid.y, map, obstacle_threshold_)) {
        ROS_ERROR("RRTPlanner: Start node is invalid (out of bounds or obstacle).");
        return nav_msgs::Path(); // Return empty path
    }
     if (!isValidCell(goal_node_grid.x, goal_node_grid.y, map) || isObstacle(goal_node_grid.x, goal_node_grid.y, map, obstacle_threshold_)) {
        ROS_ERROR("RRTPlanner: Goal node is invalid (out of bounds or obstacle).");
        return nav_msgs::Path(); // Return empty path
    }

    // Initialize the RRT tree
    std::vector<RRTNode> tree;
    RRTNode start_node_rrt;
    start_node_rrt.x = start_node_grid.x;
    start_node_rrt.y = start_node_grid.y;
    start_node_rrt.parent_id = -1; // Root node has no parent
    tree.push_back(start_node_rrt);

    int goal_node_index = -1; // Track if goal is reached

    // RRT Algorithm Loop
    for (int i = 0; i < max_iterations_; ++i) {
        // 1. Sample Random Node (with goal bias)
        Node q_rand = sampleRandomNode(map, goal_node_grid);

        // 2. Find Nearest Node in the tree
        int nearest_node_idx = findNearestNode(tree, q_rand);
        const RRTNode& q_near = tree[nearest_node_idx];

        // 3. start from nearest node towards random sample
        RRTNode q_new = steer(q_near, q_rand, step_size_);

        // 4. Check Collision along the path from q_near to q_new
        if (isCollisionFree(q_near, q_new, map, obstacle_threshold_)) {
            // 5. Add q_new to tree
            q_new.parent_id = nearest_node_idx;
            tree.push_back(q_new);
            int new_node_idx = tree.size() - 1;

            // 6. Check if goal reached
            double dist_to_goal = std::sqrt(std::pow(q_new.x - goal_node_grid.x, 2) + std::pow(q_new.y - goal_node_grid.y, 2));
            if (dist_to_goal <= goal_tolerance_dist_) {
                ROS_INFO("RRTPlanner: Goal reached after %d iterations.", i + 1);
                // Optionally add goal itself for cleaner path end
                 RRTNode goal_node_rrt;
                 goal_node_rrt.x = goal_node_grid.x;
                 goal_node_rrt.y = goal_node_grid.y;
                 goal_node_rrt.parent_id = new_node_idx;
                 tree.push_back(goal_node_rrt);
                 goal_node_index = tree.size() - 1;
                break; // Exit loop
            }
        }
        
    }

    // 7. Reconstruct Path if goal was reached
    if (goal_node_index != -1) {
        ROS_INFO("RRTPlanner: Reconstructing path...");
        return reconstructRRTPath(tree, goal_node_index, map, start_pose);
    } else {
        ROS_WARN("RRTPlanner: Failed to find path to goal after %d iterations.", max_iterations_);
        return nav_msgs::Path(); // Return empty path
    }
}



Node RRTPlanner::sampleRandomNode(const nav_msgs::OccupancyGrid& map, const Node& goal_node) {
    Node sample;
    // Goal Bias check
    if (goal_bias_dist_(rng_) < goal_bias_) {
        sample = goal_node;
        
    } else {
        sample.x = x_dist_(rng_);
        sample.y = y_dist_(rng_);
    }
    return sample;
}

int RRTPlanner::findNearestNode(const std::vector<RRTNode>& tree, const Node& sample_node) {
    int nearest_idx = -1;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tree.size(); ++i) {
        double dx = tree[i].x - sample_node.x;
        double dy = tree[i].y - sample_node.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

RRTNode RRTPlanner::steer(const RRTNode& nearest_node, const Node& sample_node, double step_size) {
    RRTNode new_node;
    double dx = sample_node.x - nearest_node.x;
    double dy = sample_node.y - nearest_node.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < step_size_) {
        // If sample is closer than step_size, move directly to sample
        new_node.x = sample_node.x;
        new_node.y = sample_node.y;
    } else {
        // Move step_size distance towards sample
        double ratio = step_size_ / dist;
        new_node.x = nearest_node.x + static_cast<int>(dx * ratio);
        new_node.y = nearest_node.y + static_cast<int>(dy * ratio);
    }
    // Parent ID will be set after collision check
    return new_node;
}

// Basic line collision check
bool RRTPlanner::isCollisionFree(const RRTNode& node1, const RRTNode& node2, const nav_msgs::OccupancyGrid& map, int obstacle_threshold) {
    int x0 = node1.x, y0 = node1.y;
    int x1 = node2.x, y1 = node2.y;

    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy; // error value e_xy

    while (true) {
        // Check current point (x0, y0)
        if (!isValidCell(x0, y0, map) || isObstacle(x0, y0, map, obstacle_threshold)) {
            
            return false;
        }

        if (x0 == x1 && y0 == y1) break; // Reached the end

        int e2 = 2 * err;
        if (e2 >= dy) { 
            if (x0 == x1) break;
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) { 
            if (y0 == y1) break;
            err += dx;
            y0 += sy;
        }
    }
    // check the final point
     if (!isValidCell(x1, y1, map) || isObstacle(x1, y1, map, obstacle_threshold)) {
         
         return false;
     }

    return true; // No collision found
}


nav_msgs::Path RRTPlanner::reconstructRRTPath(const std::vector<RRTNode>& tree, int goal_node_idx, const nav_msgs::OccupancyGrid& map, const geometry_msgs::PoseStamped& start_pose) {
    nav_msgs::Path path;
    path.header = start_pose.header; // Use header from start pose for frame_id/timestamp
    path.header.stamp = ros::Time::now(); // Update timestamp

    std::vector<geometry_msgs::PoseStamped> poses;
    int current_idx = goal_node_idx;

    while (current_idx != -1) {
        const RRTNode& current_node = tree[current_idx];
        geometry_msgs::PoseStamped pose;
        pose.header = path.header; 
        // Convert grid coords back to map coords 
        gridToMap(map, current_node.x, current_node.y, pose);
        // Set a default orientation 
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);

        current_idx = current_node.parent_id; // Move to parent
    }

    // The path is reconstructed backwards, reversing it
    std::reverse(poses.begin(), poses.end());
    path.poses = poses;

    ROS_INFO("RRTPlanner: Path reconstruction complete with %zu poses.", path.poses.size());
    return path;
}

} // namespace path_planning