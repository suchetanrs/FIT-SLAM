#ifndef ROS_VISUALIZER_HPP_
#define ROS_VISUALIZER_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <fstream>
#include <iomanip>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>

#include "frontier_exploration/util/logger.hpp"
#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

class RosVisualizer
{
public:
    static RosVisualizer &getInstance()
    {
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if(rosVisualizerPtr == nullptr)
            throw std::runtime_error("Cannot de-reference a null rosVisualizer! :(");
        return *rosVisualizerPtr;
    }

    static void createInstance(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
    {
        std::cout << "Creating instance" << std::endl;
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if (rosVisualizerPtr == nullptr)
            rosVisualizerPtr.reset(new RosVisualizer(node, costmap));
    }

    void observableCellsViz(std::vector<geometry_msgs::msg::Pose> &points);
    void observableCellsViz(std::vector<nav2_costmap_2d::MapLocation> points);
    
    void landmarkViz(std::vector<std::vector<double>> &points, visualization_msgs::msg::Marker &marker_msg_);
    void landmarkViz(std::vector<Eigen::Vector3f> &points);
    void landmarkViz(std::vector<frontier_exploration::Point2D> &points, double r = 1.0, double g = 1.0, double b = 1.0);
    
    void visualizeSpatialHashMap(const std::vector<Frontier> &frontier_list, std::string globalFrameID);
    void resetSpatialHashMap()
    {
        spatial_hashmap_viz.clear();
        pcl::PointXYZI spatial_hashmap_viz(5000);
    }
    
    void visualizeFrontier(const std::vector<Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID);
    void visualizeFrontierMarker(const std::vector<Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID);
    void exportMapCoverage(std::vector<double> polygon_xy_min_max, int counter_value_, std::string mode_);
    void frontierPlanViz(nav_msgs::msg::Path &path);

private:
    // Delete copy constructor and assignment operator to prevent copying
    RosVisualizer(const RosVisualizer &) = delete;
    RosVisualizer &operator=(const RosVisualizer &) = delete;
    RosVisualizer(rclcpp::Node::SharedPtr node);
    RosVisualizer(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);
    static std::unique_ptr<RosVisualizer> rosVisualizerPtr;
    static std::mutex instanceMutex_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_ = rclcpp::get_logger("rosVisualizer");
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher_;         ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observable_cells_publisher_; ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr connecting_cells_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;     ///< Publisher for frontier cloud.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_frontier_cloud_pub_; ///< Publisher for every frontier cloud.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr spatial_hashmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_array_publisher_;
    nav2_costmap_2d::Costmap2D *costmap_;


    // pointcloud for visualization purposes
    pcl::PointCloud<pcl::PointXYZI> spatial_hashmap_viz;
};

// using rosVisualizerInstance = RosVisualizer::getInstance;
#endif // ROS_VISUALIZER_HPP
