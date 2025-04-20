#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>
#include <unordered_map>

namespace path_planning {

// Node for path planning
struct Node {
    int x;
    int y;
    double g_cost; 
    double h_cost; 
    double f_cost; 
    
    Node() : x(0), y(0), g_cost(0), h_cost(0), f_cost(0) {}
    
    Node(int x, int y, double g = 0.0, double h = 0.0) 
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h) {}
    
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for Node
struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
};

// Convert map coordinates to grid indices
void mapToGrid(const nav_msgs::OccupancyGrid& map, 
               const geometry_msgs::PoseStamped& pose, 
               int& grid_x, int& grid_y) {
    grid_x = static_cast<int>((pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
    grid_y = static_cast<int>((pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
}

// Convert grid indices to map coordinates
void gridToMap(const nav_msgs::OccupancyGrid& map, 
               int grid_x, int grid_y, 
               geometry_msgs::PoseStamped& pose) {
    pose.pose.position.x = map.info.origin.position.x + (grid_x + 0.5) * map.info.resolution;
    pose.pose.position.y = map.info.origin.position.y + (grid_y + 0.5) * map.info.resolution;
}

// Check if cell is within map bounds
bool isValidCell(int x, int y, const nav_msgs::OccupancyGrid& map) {
    if (x < 0 || y < 0 || x >= map.info.width || y >= map.info.height) {
        return false;
    }
    return true;
}

// Check if cell is an obstacle
bool isObstacle(int x, int y, const nav_msgs::OccupancyGrid& map, int obstacle_threshold) {
    if (!isValidCell(x, y, map)) {
        return true; // Out-of-bounds is obstacle
    }
    
    int index = y * map.info.width + x;
    return map.data[index] >= obstacle_threshold;
}

// Get neighbors of a node
std::vector<Node> getNeighbors(const Node& node, 
                              const nav_msgs::OccupancyGrid& map, 
                              int obstacle_threshold) {
    std::vector<Node> neighbors;
    
    // 8-connected grid directions
    const int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
    const int dy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    
    for (int i = 0; i < 8; i++) {
        int nx = node.x + dx[i];
        int ny = node.y + dy[i];
        
        if (isValidCell(nx, ny, map) && !isObstacle(nx, ny, map, obstacle_threshold)) {
            double cost = (i < 4) ? 1.0 : 1.414; // Diagonal costs more
            neighbors.push_back(Node(nx, ny, 0, 0));
        }
    }
    
    return neighbors;
}

// Calculate distance heuristic
double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Reconstruct path from came_from map
nav_msgs::Path reconstructPath(
    const std::unordered_map<Node, Node, NodeHash>& came_from,
    const Node& start, const Node& goal,
    const nav_msgs::OccupancyGrid& map) {
    
    nav_msgs::Path path;
    path.header.frame_id = map.header.frame_id;
    path.header.stamp = ros::Time::now();
    
    Node current = goal;
    
    std::vector<geometry_msgs::PoseStamped> poses;
    while (!(current == start)) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        gridToMap(map, current.x, current.y, pose);
        poses.push_back(pose);
        
        auto it = came_from.find(current);
        if (it == came_from.end()) {
            ROS_ERROR("Path reconstruction failed! No parent for node (%d, %d)", current.x, current.y);
            break;
        }
        current = it->second;
    }
    
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = path.header;
    gridToMap(map, start.x, start.y, start_pose);
    poses.push_back(start_pose);
    
    std::reverse(poses.begin(), poses.end());
    path.poses = poses;
    
    return path;
}

} // namespace path_planning

#endif // PATH_PLANNING_UTILS_H