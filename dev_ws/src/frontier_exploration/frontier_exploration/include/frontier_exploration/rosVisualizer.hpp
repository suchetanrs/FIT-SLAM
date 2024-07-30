#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP

#include <vector>
#include <string>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <frontier_exploration/Frontier.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/path.hpp>
#include "frontier_exploration/GeometryUtils.hpp"

class RosVisualizer
{
public:
    RosVisualizer(rclcpp::Node::SharedPtr node);
    RosVisualizer(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);
    void landmarkViz(std::vector<std::vector<double>> &points, visualization_msgs::msg::Marker &marker_msg_);
    void observableCellsViz(std::vector<geometry_msgs::msg::Pose> &points);
    void observableCellsViz(std::vector<nav2_costmap_2d::MapLocation> points);
    void landmarkViz(std::vector<Eigen::Vector3f> &points);
    void landmarkViz(std::vector<frontier_exploration::Point2D> &points, double r = 1.0, double g = 1.0, double b = 1.0);
    void visualizeFrontier(const std::vector<Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID);
    void exportMapCoverage(std::vector<double> polygon_xy_min_max, int counter_value_, std::string mode_);
    void frontierPlanViz(nav_msgs::msg::Path& path);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_ = rclcpp::get_logger("rosVisualizer");
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher_;   ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observable_cells_publisher_;   ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr connecting_cells_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;     ///< Publisher for frontier cloud.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_frontier_cloud_pub_; ///< Publisher for every frontier cloud.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    nav2_costmap_2d::Costmap2D *costmap_;
};

#endif // ROS_VISUALIZER_HPP
