#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>


namespace path_planning {

// Node - x-coordinate, y-coordinate,f_cost,g_cost,h_cost
//f_cost = g_cost + h_cost

struct Node {
    int x;
    int y;
    double g_cost; // Cost from start to current node
    double h_cost; // Heuristic cost 
    double f_cost; // Total cost (g_cost + h_cost)
    
    // Default constructor 
    Node() : x(0), y(0), g_cost(0), h_cost(0), f_cost(0) {}
    
    Node(int x, int y, double g = 0.0, double h = 0.0) 
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h) {}
    
    // For priority queue comparison flippped because cpp pq is max heap
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
    
    // For equality comparison
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for Node to use in unordered_map

struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
};

class AStarPlanner {
public:
    AStarPlanner(ros::NodeHandle& nh);
    //main function to plant the path
    nav_msgs::Path planPath(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal,
                            const nav_msgs::OccupancyGrid& map);

private:
    ros::NodeHandle nh_;
    
    // Parameters
    double inflation_radius_; // Obstacle inflation radius - Approximating the obstacle 
    int obstacle_threshold_;  // Threshold for considering a cell as an obstacle
    
    // Helper methods
    double heuristic(int x1, int y1, int x2, int y2);
    bool isValidCell(int x, int y, const nav_msgs::OccupancyGrid& map);
    bool isObstacle(int x, int y, const nav_msgs::OccupancyGrid& map);
    std::vector<Node> getNeighbors(const Node& node, const nav_msgs::OccupancyGrid& map);
    nav_msgs::Path reconstructPath(
        const std::unordered_map<Node, Node, NodeHash>& came_from,
        const Node& start, const Node& goal,
        const nav_msgs::OccupancyGrid& map);
    
    // Convert between map coordinates and grid indices
    void mapToGrid(const nav_msgs::OccupancyGrid& map, 
                   const geometry_msgs::PoseStamped& pose, 
                   int& grid_x, int& grid_y);
    void gridToMap(const nav_msgs::OccupancyGrid& map, 
                   int grid_x, int grid_y, 
                   geometry_msgs::PoseStamped& pose);
};

AStarPlanner::AStarPlanner(ros::NodeHandle& nh) : nh_(nh) {
    // Load parameters
    nh_.param<double>("inflation_radius", inflation_radius_, 0.3);
    nh_.param<int>("obstacle_threshold", obstacle_threshold_, 50);
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    // Euclidean distance heuristic
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

bool AStarPlanner::isValidCell(int x, int y, const nav_msgs::OccupancyGrid& map) {
    // Check if the cell is within map bounds
    if (x < 0 || y < 0 || x >= map.info.width || y >= map.info.height) {
        return false;
    }
    return true;
}

bool AStarPlanner::isObstacle(int x, int y, const nav_msgs::OccupancyGrid& map) {
    // Check if the cell is an obstacle
    if (!isValidCell(x, y, map)) {
        return true; // Consider out-of-bounds as obstacles
    }
    
    int index = y * map.info.width + x;
    return map.data[index] >= obstacle_threshold_;
}

std::vector<Node> AStarPlanner::getNeighbors(
    const Node& node, const nav_msgs::OccupancyGrid& map) {
    
    std::vector<Node> neighbors;
    
    // 8-connected grid (horizontal, vertical, and diagonal neighbors)
    const int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
    const int dy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    
    for (int i = 0; i < 8; i++) {
        int nx = node.x + dx[i];
        int ny = node.y + dy[i];
        
        if (isValidCell(nx, ny, map) && !isObstacle(nx, ny, map)) {
            // Diagonal movements cost more
            double cost = (i < 4) ? 1.0 : 1.414;
            neighbors.push_back(Node(nx, ny, 0, 0)); // g and h will be set later
        }
    }
    
    return neighbors;
}

void AStarPlanner::mapToGrid(const nav_msgs::OccupancyGrid& map, 
                             const geometry_msgs::PoseStamped& pose, 
                             int& grid_x, int& grid_y) {
    // Convert world coordinates to grid indices - Origin is at the center of the cell
    grid_x = static_cast<int>((pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
    grid_y = static_cast<int>((pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
}

void AStarPlanner::gridToMap(const nav_msgs::OccupancyGrid& map, 
                             int grid_x, int grid_y, 
                             geometry_msgs::PoseStamped& pose) {
    // Convert grid indices to world coordinates - 
    pose.pose.position.x = map.info.origin.position.x + (grid_x + 0.5) * map.info.resolution;
    pose.pose.position.y = map.info.origin.position.y + (grid_y + 0.5) * map.info.resolution;
}

nav_msgs::Path AStarPlanner::reconstructPath(
    const std::unordered_map<Node, Node, NodeHash>& came_from,
    const Node& start, const Node& current,
    const nav_msgs::OccupancyGrid& map) {
    
    nav_msgs::Path path;
    path.header.frame_id = map.header.frame_id;
    path.header.stamp = ros::Time::now();
    
    // Start with the goal
    Node curr = current;
    
    // Reconstruct the path by following parent pointers
    std::vector<geometry_msgs::PoseStamped> poses;
    while (!(curr == start)) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        gridToMap(map, curr.x, curr.y, pose);
        poses.push_back(pose);
        
        // Find the parent
        auto it = came_from.find(curr);
        if (it == came_from.end()) {
            ROS_ERROR("Path reconstruction failed!- No parant found for node (%d, %d)", curr.x, curr.y);
            break;
        }
        curr = it->second;
    }
    
    // Add the start position
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = path.header;
    gridToMap(map, start.x, start.y, start_pose);
    poses.push_back(start_pose);
    
    // Reverse the path to get start-to-goal order
    std::reverse(poses.begin(), poses.end());
    path.poses = poses;
    
    return path;
}

nav_msgs::Path AStarPlanner::planPath(
    const geometry_msgs::PoseStamped& start_pose, 
    const geometry_msgs::PoseStamped& goal_pose,
    const nav_msgs::OccupancyGrid& map) {
    
    // Convert start and goal positions to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    mapToGrid(map, start_pose, start_x, start_y);
    mapToGrid(map, goal_pose, goal_x, goal_y);
    
    Node start(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
    Node goal(goal_x, goal_y);
    
    // Open set implemented as a priority queue
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    
    // Keep track of which nodes are in the open set
    std::unordered_map<Node, bool, NodeHash> in_open_set;
    
    // Keep track of where each node came from (for path reconstruction)
    std::unordered_map<Node, Node, NodeHash> came_from;
    
    // Keep track of g scores (cost from start to node)
    std::unordered_map<Node, double, NodeHash> g_score;
    
    // Initialize the open set with the start node
    open_set.push(start);
    in_open_set[start] = true;
    g_score[start] = 0;
    
    while (!open_set.empty()) {
        // Get the node with the lowest f_cost
        Node current = open_set.top();
        open_set.pop();
        in_open_set[current] = false;
        
        // If we've reached the goal, reconstruct and return the path
        if (current.x == goal.x && current.y == goal.y) {
            return reconstructPath(came_from, start, current, map);
        }
        
        // Consider all neighbors
        for (Node neighbor : getNeighbors(current, map)) {
            // Calculate tentative g_score (cost from start to neighbor through current)
            double movement_cost = (neighbor.x == current.x || neighbor.y == current.y) ? 1.0 : 1.414;
            double tentative_g_score = g_score[current] + movement_cost;
            
            // If we haven't seen this neighbor before or if this path is better
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                // Update the path and costs
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                neighbor.g_cost = tentative_g_score;
                neighbor.h_cost = heuristic(neighbor.x, neighbor.y, goal.x, goal.y);
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost;
                
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
