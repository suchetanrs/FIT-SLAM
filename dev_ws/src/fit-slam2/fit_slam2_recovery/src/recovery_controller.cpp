#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "fit_slam2_recovery/recovery_controller.hpp"

namespace roadmap_explorer
{

RecoveryController::RecoveryController(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<nav2_util::LifecycleNode> node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : explore_costmap_ros_(explore_costmap_ros)
{
    node_ptr_ = node_ptr;
    vel_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    vel_publisher_->on_activate();
    costmap_ = explore_costmap_ros_->getCostmap();
    tf_buffer_ = tf_buffer;
}

bool RecoveryController::computeVelocityCommand(bool backward_only)
{
    explore_costmap_ros_->getRobotPose(robot_pose_);
    // Define possible directions (angles in radians) to check (e.g., forward, left, right, etc.)
    // const std::vector<double> directions = {0.0, M_PI_2, -M_PI_2, M_PI}; // forward, left, right, backward
    std::vector<double> directions = {0.0, M_PI};
    if(backward_only)
        directions = {M_PI};
    std::vector<double> free_space_in_directions;

    // Evaluate free space in each direction
    for (double direction : directions)
    {
        auto value = evaluateFreeSpaceInDirection(direction);
        auto distance = value * costmap_->getResolution();
        free_space_in_directions.push_back(distance);
        LOG_WARN("Got " << value << " for " << direction);
    }

    // Find the index of the direction with the most free space
    auto max_free_space_iter = std::max_element(free_space_in_directions.begin(), free_space_in_directions.end());
    int best_direction_idx = std::distance(free_space_in_directions.begin(), max_free_space_iter);

    // Compute corresponding command velocities for the best direction
    geometry_msgs::msg::Twist cmd_vel;
    double best_direction = directions[best_direction_idx];
    double movement_distance = free_space_in_directions[best_direction_idx];
    LOG_WARN("Movement distance: " << movement_distance);
    movement_distance = std::min(movement_distance, 0.84);
    cmd_vel.linear.x = std::cos(best_direction) * 0.6; // Move in the direction
    cmd_vel.linear.y = 0.0; // Only for holonomic robots
    cmd_vel.angular.z = 0.0;                     // No rotation for simplicity
    int sleep_time_ms = 10;
    double count = movement_distance / (0.6 * sleep_time_ms) * 1000;
    LOG_WARN("Count: " << count);
    for(auto i=0; i < count; i++)
    {
        vel_publisher_->publish(cmd_vel);
        rclcpp::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }
    geometry_msgs::msg::Twist cmd_vel_stopped;
    vel_publisher_->publish(cmd_vel_stopped);
    vel_publisher_->publish(cmd_vel_stopped);
    vel_publisher_->publish(cmd_vel_stopped);
    vel_publisher_->publish(cmd_vel_stopped);
    vel_publisher_->publish(cmd_vel_stopped);
    return true;
}

double RecoveryController::evaluateFreeSpaceInDirection(double direction)
{

    LOG_TRACE("Evaluating free space in direction: " << direction);
    std::vector<nav2_costmap_2d::MapLocation> traced_cells;
    LOG_TRACE("Traced cells size: " << traced_cells.size());
    // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
    roadmap_explorer::RayTracedCells cell_gatherer(costmap_, traced_cells, 256, 256, 0, 255);
    LOG_TRACE("Cell gatherer created");
    LOG_TRACE("Exploring costmap ros: " << explore_costmap_ros_);
    explore_costmap_ros_->getRobotPose(robot_pose_);
    double sx = robot_pose_.pose.position.x;
    double sy = robot_pose_.pose.position.y;
    auto yaw = roadmap_explorer::quatToEuler(robot_pose_.pose.orientation)[2];
    double wx = sx + (2.5 * cos(yaw + direction));
    double wy = sy + (2.5 * sin(yaw + direction));
    LOG_TRACE("Getting traced cells");
    if (!roadmap_explorer::getTracedCells(sx, sy, wx, wy, cell_gatherer, 4.0 / costmap_->getResolution(), costmap_))
    {
        LOG_TRACE("Failed to get traced cells");
        return -std::numeric_limits<double>::max();
    }
    LOG_TRACE("Traced cells got");
    auto cells = cell_gatherer.getCells();
    LOG_TRACE("Cells size: " << cells.size());
    int free_space = 0;
    for (auto& cell : cells)
    {
        auto costmapcost = static_cast<int>(costmap_->getCost(cell.x, cell.y));
        if (costmapcost < 253 || costmapcost == 255)
        {
            free_space += 1;
        }
    }
    LOG_TRACE("Free space: " << free_space);
    RosVisualizer::getInstance().observableCellsViz(cells);
    LOG_TRACE("Free space2: " << free_space);

    return free_space;
}
}