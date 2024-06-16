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
#include <frontier_msgs/msg/frontier.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/path.hpp>

class RosVisualizer
{
public:
    RosVisualizer(rclcpp::Node::SharedPtr node);
    RosVisualizer(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);
    void landmarkViz(std::vector<std::vector<double>> &points, visualization_msgs::msg::Marker &marker_msg_);
    void observableCellsViz(std::vector<geometry_msgs::msg::Pose> &points);
    void landmarkViz(std::vector<Eigen::Vector3f> &points);
    void visualizeFrontier(const std::vector<frontier_msgs::msg::Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID);
    void exportMapCoverage(std::vector<double> polygon_xy_min_max, int counter_value_, std::string mode_);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_ = rclcpp::get_logger("rosVisualizer");
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher_;   ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observable_cells_publisher_;   ///< Publisher for landmarks in the path FOVs
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;     ///< Publisher for frontier cloud.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_frontier_cloud_pub_; ///< Publisher for every frontier cloud.
    nav2_costmap_2d::Costmap2D *costmap_;
};

#endif // ROS_VISUALIZER_HPP
