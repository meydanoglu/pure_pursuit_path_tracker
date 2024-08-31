#include "pure_pursuit.hpp"
#include <std_msgs/msg/string.hpp>
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // initialise parameters
    this->declare_parameter("waypoints_path_topic", "/planned_path");
    this->declare_parameter("odom_topic", "/odometry_integration/car_state");
    this->declare_parameter("car_refFrame", "base_footprint");
    this->declare_parameter("drive_topic", "/cmd");
    this->declare_parameter("rviz_taken_road_topic", "/taken_road");
    this->declare_parameter("rviz_desired_road_topic", "/desired_road");
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("min_lookahead", 0.3);
    this->declare_parameter("max_lookahead", 2.0);
    this->declare_parameter("lookahead_ratio", 6.0);
    this->declare_parameter("K_p", 0.39);
    this->declare_parameter("steering_limit", 40.0);
    this->declare_parameter("velocity_percentage", 0.6);

    // Default Values
    waypoints_path_topic = this->get_parameter("waypoints_path_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    
    
    rviz_taken_road_topic = this->get_parameter("rviz_taken_road_topic").as_string();
    rviz_desired_road_topic = this->get_parameter("rviz_desired_road_topic").as_string();
    
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    K_p = this->get_parameter("K_p").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();

    subscription_odom = this->create_subscription<eufs_msgs::msg::CarState>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    planned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        waypoints_path_topic, 10, std::bind(&PurePursuit::planned_path_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    
    transform_info_pub = this->create_publisher<std_msgs::msg::String>("transform_info", 10);
    

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

    //planned_path_callback();
}

double PurePursuit::to_radians(double degrees) {
    double radians;
    return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    double degrees;
    return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

void PurePursuit::planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    /*
    Takes in the planned path from the path planner and stores the x and y coordinates in the waypoints struct
    */
//TODO: Dont think this works correctly.

    waypoints.X.clear();
    waypoints.Y.clear();

    for (const auto &pose : msg->poses) {
        waypoints.X.push_back(pose.pose.position.x);
        waypoints.Y.push_back(pose.pose.position.y);
 
    }
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", waypoints.X.size());

    get_waypoint(); //updates the waypoints.index and waypoints.velocity_index
    transformandinterp_waypoint(); //updates the waypoints.lookahead_point_car and waypoints.current_point_car
}



void PurePursuit::taken_road_visualization(double x, double y) {
    static visualization_msgs::msg::Marker line_strip;
    static std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> points_with_timestamps;

    // Initialize the marker only once
    if (line_strip.points.empty()) {
        line_strip.header.frame_id = global_refFrame;
        line_strip.ns = "taken_road";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
    }

    // Add the new point with timestamp
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    points_with_timestamps.push_back({p, this->now()});

    // Remove points older than 10 seconds
    rclcpp::Time current_time = this->now();
    points_with_timestamps.erase(
        std::remove_if(points_with_timestamps.begin(), points_with_timestamps.end(),
                       [current_time](const std::pair<geometry_msgs::msg::Point, rclcpp::Time>& point) {
                           return (current_time - point.second).seconds() > 10.0;
                       }),
        points_with_timestamps.end());

    // Update the line strip points
    line_strip.points.clear();
    for (const auto& point_with_timestamp : points_with_timestamps) {
        line_strip.points.push_back(point_with_timestamp.first);
    }

    // Update the timestamp and publish the line strip
    if (++waypoints.publish_counter_taken_road % 3 == 0) { // Publish every 3rd call
        line_strip.header.stamp = this->now();
        vis_taken_road_pub->publish(line_strip);
        waypoints.publish_counter_taken_road = 0; // Reset counter
    }
}




void PurePursuit::get_waypoint() {
    // Calculate the lookahead distance based on the current velocity and the lookahead ratio
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);
    
    int closest_point = 0; // index of the closest point to the car
    double closest_distance = std::numeric_limits<double>::max(); // distance to the closest point
    
    for (int i = 0; i < num_waypoints; i++) { // iterate through all waypoints
        double distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world); // calculate the distance between the car and the waypoint
        if (distance < closest_distance) { // if the distance is less than the closest distance
            closest_distance = distance;
            closest_point = i;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Closest point index: %d, distance: %f", closest_point, closest_distance);
    
    int lookahead_point = closest_point; // assign the closest point as the lookahead point
    double accumulated_distance = 0; // accumulated distance from the closest point to the lookahead point
    
    while (accumulated_distance < lookahead) { // while the accumulated distance is less than the lookahead distance
        int next_point = (lookahead_point + 1) % num_waypoints; // get the next point
        accumulated_distance += p2pdist(waypoints.X[lookahead_point], waypoints.X[next_point],
                                        waypoints.Y[lookahead_point], waypoints.Y[next_point]);
        lookahead_point = next_point;
        
        // Break the loop if we have cycled through all waypoints
        if (lookahead_point == closest_point) {
            break;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Lookahead point index: %d, accumulated distance: %f", lookahead_point, accumulated_distance);
    
    waypoints.index = lookahead_point;
    waypoints.velocity_index = closest_point;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
    double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
    double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
    double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

    double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
    double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
    double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

    double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
    double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
    double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

    rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

void PurePursuit::transformandinterp_waypoint() {  // pass old waypoint here


    

    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;




    geometry_msgs::msg::TransformStamped transformStamped; // declare a variable named transformStamped of type TransformStamped

    // get the transform from the global reference frame to the car reference frame
    try {
        // Get the transform from the base_link reference to world reference frame
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
    }

    // extract translation and rotation from the transformStamped
    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);


    //store the transformed lookahead and current waypoints in the car reference frame 
    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
    


    //Publish transform info
    std_msgs::msg::String transform_info_msg;
    std::stringstream ss; 
    ss << "GLP: [" << waypoints.lookahead_point_world.transpose() << "], "
       << "GCP: [" << waypoints.current_point_world.transpose() << "], "
       << "TrLoPo: [" << waypoints.lookahead_point_car.transpose() << "], "
       << "TV: [" << translation_v.transpose() << "], ";
       
       
    
    transform_info_msg.data = ss.str();
    transform_info_pub->publish(transform_info_msg);
}   



//the lookahead point is now expressed in the car's reference frame
double PurePursuit::p_controller() {
    double r = waypoints.lookahead_point_car.norm();  // r = sqrt(x^2 + y^2)
    double y = waypoints.lookahead_point_car(1);
    double angle = K_p * 2 * y / pow(r, 2);  // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

    return angle;
}

double PurePursuit::get_velocity() {
    /*
    double velocity = 0;

    if (waypoints.V[waypoints.velocity_index]) {
        velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
    } else {  // For waypoints loaded without velocity profiles
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 6.0 * velocity_percentage;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 2.5 * velocity_percentage;
        } else {
            velocity = 2.0 * velocity_percentage;
        }
    }

    return velocity;
    */
    return 13.0;
}

void PurePursuit::publish_message(double steering_angle) {
    auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
    if (steering_angle < 0.0) {
        drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit));  // ensure steering angle is dynamically capable
    } else {
        drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit));  // ensure steering angle is dynamically capable
    }

    curr_velocity = get_velocity();
    drive_msgObj.drive.speed = curr_velocity;

    RCLCPP_INFO(this->get_logger(), "lookahead_index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... ", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle));

    publisher_drive->publish(drive_msgObj);
}

void PurePursuit::odom_callback(const eufs_msgs::msg::CarState::ConstSharedPtr odom_submsgObj) {
    RCLCPP_INFO(this->get_logger(), "odom_callback triggered");
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "x_car_world: %.2f ... y_car_world: %.2f", x_car_world, y_car_world);

    if (waypoints.X.empty() || waypoints.Y.empty()) {
        RCLCPP_WARN(this->get_logger(), "No planned path available. Waiting for waypoints.");
        return;
    }
    taken_road_visualization(x_car_world, y_car_world);

    get_waypoint();
    transformandinterp_waypoint();
    double steering_angle = p_controller();
    publish_message(steering_angle);
}

void PurePursuit::timer_callback() {
    // Periodically check parameters and update
    K_p = this->get_parameter("K_p").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
