#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>


#include "utils.hpp"

namespace path_planning {

class DijkstraPlanner {
public:
    DijkstraPlanner(ros::NodeHandle& nh);
    nav_msgs::Path planPath(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal,
                            const nav_msgs::OccupancyGrid& map);

private:
    ros::NodeHandle nh_;
    
    // Parameters
    double inflation_radius_; 
    int obstacle_threshold_;  
};

DijkstraPlanner::DijkstraPlanner(ros::NodeHandle& nh) : nh_(nh) {
    // Load parameters
    nh_.param<double>("inflation_radius", inflation_radius_, 0.3);
    nh_.param<int>("obstacle_threshold", obstacle_threshold_, 50);
}

nav_msgs::Path DijkstraPlanner::planPath(
    const geometry_msgs::PoseStamped& start_pose, 
    const geometry_msgs::PoseStamped& goal_pose,
    const nav_msgs::OccupancyGrid& map) {
    
    // Convert start and goal positions to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    mapToGrid(map, start_pose, start_x, start_y);
    mapToGrid(map, goal_pose, goal_x, goal_y);
    
    
    Node start(start_x, start_y, 0, 0); 
    Node goal(goal_x, goal_y);
    
    // Open set implemented as a priority queue
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    
    //track of which nodes are in the open set
    std::unordered_map<Node, bool, NodeHash> in_open_set;
    
    //track of where each node came from (for path reconstruction)
    std::unordered_map<Node, Node, NodeHash> came_from;
    
    //track of g scores (cost from start to node)
    std::unordered_map<Node, double, NodeHash> g_score;
    
    // Initialize the open set with the start node
    open_set.push(start);
    in_open_set[start] = true;
    g_score[start] = 0;
    
    while (!open_set.empty()) {
        // Get the node with the lowest g_cost 
        Node current = open_set.top();
        open_set.pop();
        in_open_set[current] = false;
        
        // If we've reached the goal, reconstruct and return the path
        if (current.x == goal.x && current.y == goal.y) {
            return reconstructPath(came_from, start, current, map);
        }
        
        // Consider all neighbors
        for (Node neighbor : getNeighbors(current, map, obstacle_threshold_)) {
            // Calculate tentative g_score (cost from start to neighbor through current)
            double movement_cost = (neighbor.x == current.x || neighbor.y == current.y) ? 1.0 : 1.414;
            double tentative_g_score = g_score[current] + movement_cost;
            
            // If we haven't seen this neighbor before or if this path is better
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                // Update the path and costs
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                neighbor.g_cost = tentative_g_score;
                neighbor.h_cost = 0; // No heuristic in Dijkstra
                neighbor.f_cost = neighbor.g_cost; // f_cost = g_cost in Dijkstra
                
                // Add to the open set if not already there
                if (!in_open_set[neighbor]) {
                    open_set.push(neighbor);
                    in_open_set[neighbor] = true;
                }
            }
        }
    }
    
    // If we've exhausted the open set without finding a path, return an empty path
    ROS_WARN("No path found from start to goal");
    return nav_msgs::Path();
}

} // namespace path_planning