#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

// Forward declare the classes and functions we'll use

namespace path_planning {
    class AStarPlanner;
}


#include "a_star_planner.cpp"

using namespace path_planning;

class PathPlannerNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    
    tf::TransformListener tf_listener_;
    nav_msgs::OccupancyGrid map_;
    bool map_received_;
    
    AStarPlanner planner_;

public:
    PathPlannerNode() : nh_("~"), map_received_(false), planner_(nh_) {
        // Subscribe to map updates
        map_sub_ = nh_.subscribe("/map", 1, &PathPlannerNode::mapCallback, this);
        
        // Subscribe to goal poses
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PathPlannerNode::goalCallback, this);
        
        // Publish the planned path
        path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);
        
        ROS_INFO("Path planner node initialized");
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        map_ = *map;
        map_received_ = true;
        ROS_INFO("Map received");
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
        if (!map_received_) {
            ROS_WARN("No map received yet, cannot plan path");
            return;
        }
        
        // Get the current robot position from TF
        geometry_msgs::PoseStamped start;
        start.header.frame_id = "base_footprint";
        start.header.stamp = ros::Time(0);
        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;
        start.pose.orientation.w = 1.0;
        
        try {
            tf_listener_.waitForTransform(map_.header.frame_id, start.header.frame_id, 
                                          ros::Time(0), ros::Duration(1.0));
            tf_listener_.transformPose(map_.header.frame_id, start, start);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to transform robot pose: %s", ex.what());
            return;
        }
        
        // Plan the path
        ROS_INFO("Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                 start.pose.position.x, start.pose.position.y,
                 goal->pose.position.x, goal->pose.position.y);
        
        nav_msgs::Path path = planner_.planPath(start, *goal, map_);
        
        // Publish the path
        if (!path.poses.empty()) {
            path_pub_.publish(path);
            ROS_INFO("Path published with %zu points", path.poses.size());
        } else {
            ROS_WARN("No path found");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    PathPlannerNode path_planner;
    ros::spin();
    return 0;
}