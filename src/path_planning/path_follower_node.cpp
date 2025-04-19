#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

class PathFollower {
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_vel_pub_;
    tf::TransformListener tf_listener_;
    
    nav_msgs::Path current_path_;
    bool path_received_;
    
    // Parameters
    double look_ahead_distance_;
    double linear_velocity_;
    double angular_velocity_gain_;
    double goal_tolerance_;
    
public:
    PathFollower() : nh_("~"), path_received_(false) {
        // Load parameters
        nh_.param<double>("look_ahead_distance", look_ahead_distance_, 0.5);
        nh_.param<double>("linear_velocity", linear_velocity_, 0.2);
        nh_.param<double>("angular_velocity_gain", angular_velocity_gain_, 1.0);
        nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        
        // Subscribe to path
        path_sub_ = nh_.subscribe("/path", 1, &PathFollower::pathCallback, this);
        
        // Publish velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Start following loop
        ROS_INFO("Path follower initialized");
        followPath();
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& path) {
        current_path_ = *path;
        path_received_ = true;
        ROS_INFO("Received path with %zu points", path->poses.size());
    }
    
    void followPath() {
        ros::Rate rate(10); // 10 Hz
        
        while (ros::ok()) {
            if (path_received_ && !current_path_.poses.empty()) {
                // Get robot's current position
                tf::StampedTransform transform;
                try {
                    tf_listener_.waitForTransform(current_path_.header.frame_id, "base_footprint", 
                                                  ros::Time(0), ros::Duration(1.0));
                    tf_listener_.lookupTransform(current_path_.header.frame_id, "base_footprint", 
                                                ros::Time(0), transform);
                } catch (tf::TransformException& ex) {
                    ROS_ERROR("TF error: %s", ex.what());
                    rate.sleep();
                    ros::spinOnce();
                    continue;
                }
                
                double robot_x = transform.getOrigin().x();
                double robot_y = transform.getOrigin().y();
                
                // Find the closest point on the path
                int closest_point_idx = 0;
                double min_dist = std::numeric_limits<double>::max();
                
                for (size_t i = 0; i < current_path_.poses.size(); i++) {
                    double dx = current_path_.poses[i].pose.position.x - robot_x;
                    double dy = current_path_.poses[i].pose.position.y - robot_y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_point_idx = i;
                    }
                }
                
                // Find the look-ahead point
                int look_ahead_idx = closest_point_idx;
                double dist_accumulated = 0.0;
                
                for (size_t i = closest_point_idx; i < current_path_.poses.size() - 1; i++) {
                    double dx = current_path_.poses[i+1].pose.position.x - current_path_.poses[i].pose.position.x;
                    double dy = current_path_.poses[i+1].pose.position.y - current_path_.poses[i].pose.position.y;
                    double segment_length = std::sqrt(dx*dx + dy*dy);
                    
                    dist_accumulated += segment_length;
                    look_ahead_idx = i + 1;
                    
                    if (dist_accumulated >= look_ahead_distance_) {
                        break;
                    }
                }
                
                // Check if we're at the goal
                if (closest_point_idx == current_path_.poses.size() - 1 && min_dist < goal_tolerance_) {
                    // We've reached the goal
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Goal reached!");
                    path_received_ = false;
                } else {
                    // Calculate the steering angle
                    double target_x = current_path_.poses[look_ahead_idx].pose.position.x;
                    double target_y = current_path_.poses[look_ahead_idx].pose.position.y;
                    
                    double dx = target_x - robot_x;
                    double dy = target_y - robot_y;
                    
                    double target_angle = std::atan2(dy, dx);
                    
                    // Get robot's current orientation
                    double roll, pitch, yaw;
                    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
                    
                    // Calculate the error in angle
                    double angle_error = target_angle - yaw;
                    
                    // Normalize angle to [-pi, pi]
                    while (angle_error > M_PI) angle_error -= 2*M_PI;
                    while (angle_error < -M_PI) angle_error += 2*M_PI;
                    
                    // Create and publish velocity command
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = linear_velocity_;
                    cmd_vel.angular.z = angle_error * angular_velocity_gain_;
                    
                    cmd_vel_pub_.publish(cmd_vel);
                }
            }
            
            rate.sleep();
            ros::spinOnce();
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    PathFollower follower;
    ros::spin();
    return 0;
}