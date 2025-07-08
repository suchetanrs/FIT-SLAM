#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <vector>
#include <cmath>
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"


namespace roadmap_explorer
{

class RecoveryController
{
public:
    // Constructor
    RecoveryController(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<nav2_util::LifecycleNode> node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    // Function to compute the velocity command based on free space
    bool computeVelocityCommand(bool backward_only);

private:
    // Member variables
    nav2_costmap_2d::Costmap2D* costmap_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    std::shared_ptr<nav2_util::LifecycleNode> node_ptr_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Private function to evaluate free space in a specific direction
    double evaluateFreeSpaceInDirection(double direction);
};

}