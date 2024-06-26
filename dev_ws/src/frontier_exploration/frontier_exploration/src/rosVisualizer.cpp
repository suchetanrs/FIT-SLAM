#include "frontier_exploration/rosVisualizer.hpp"

RosVisualizer::RosVisualizer(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    logger_ = node_->get_logger();
    // Creating publishers with custom QoS settings
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", custom_qos);

    all_frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("all_frontiers", custom_qos);

    landmark_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("landmark_marker", 10);

    observable_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("observable_cells", 10);
    connecting_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("connecting_cells", 10);
    frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);
    costmap_ = nullptr;
}

RosVisualizer::RosVisualizer(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
{
    node_ = node;
    logger_ = node_->get_logger();
    // Creating publishers with custom QoS settings
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", custom_qos);

    all_frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("all_frontiers", custom_qos);

    landmark_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("landmark_marker", 10);

    observable_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("observable_cells", 10);
    connecting_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("connecting_cells", 10);
    frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);
    costmap_ = costmap;
}

void RosVisualizer::landmarkViz(std::vector<std::vector<double>> &points, visualization_msgs::msg::Marker &marker_msg_)
{

    // Initialize the Marker message
    marker_msg_.header.frame_id = "map"; // Set the frame ID
    marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker_msg_.scale.x = 0.05; // Point size
    marker_msg_.scale.y = 0.05;

    // Set the color (green in RGBA format)
    marker_msg_.color.r = 0.0;
    marker_msg_.color.g = 1.0;
    marker_msg_.color.b = 0.0;
    marker_msg_.color.a = 1.0;
    for (auto point : points)
    {
        geometry_msgs::msg::Point point1;
        point1.x = point[0];
        point1.y = point[1];
        marker_msg_.points.push_back(point1);
    }
}

void RosVisualizer::observableCellsViz(std::vector<geometry_msgs::msg::Pose> &points)
{
    if(costmap_ == nullptr)
    {
        throw std::runtime_error("You called the wrong constructor. Costmap is a nullptr");
    }
    visualization_msgs::msg::Marker marker_msg_;
    // Initialize the Marker message
    marker_msg_.header.frame_id = "map"; // Set the frame ID
    marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker_msg_.scale.x = costmap_->getResolution(); // Point size
    marker_msg_.scale.y = costmap_->getResolution();

    // Set the color (green in RGBA format)
    marker_msg_.color.r = 0.0;
    marker_msg_.color.g = 1.0;
    marker_msg_.color.b = 0.0;
    marker_msg_.color.a = 1.0;
    for (auto point : points)
    {
        geometry_msgs::msg::Point point1 = point.position;
        marker_msg_.points.push_back(point1);
    }
    observable_cells_publisher_->publish(marker_msg_);
}

void RosVisualizer::observableCellsViz(std::vector<nav2_costmap_2d::MapLocation> points)
{
    if(costmap_ == nullptr)
    {
        throw std::runtime_error("You called the wrong constructor. Costmap is a nullptr");
    }
    visualization_msgs::msg::Marker marker_msg_;
    // Initialize the Marker message
    marker_msg_.header.frame_id = "map"; // Set the frame ID
    marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker_msg_.scale.x = costmap_->getResolution(); // Point size
    marker_msg_.scale.y = costmap_->getResolution();

    // Set the color (green in RGBA format)
    marker_msg_.color.r = 0.0;
    marker_msg_.color.g = 1.0;
    marker_msg_.color.b = 0.0;
    marker_msg_.color.a = 1.0;
    for (auto point : points)
    {
        geometry_msgs::msg::Point point1;
        costmap_->mapToWorld(point.x, point.y, point1.x, point1.y);
        marker_msg_.points.push_back(point1);
    }
    connecting_cells_publisher_->publish(marker_msg_);
}

void RosVisualizer::landmarkViz(std::vector<Eigen::Vector3f> &points)
{
    visualization_msgs::msg::Marker marker_msg_;
    // Initialize the Marker message
    marker_msg_.header.frame_id = "map"; // Set the frame ID
    marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker_msg_.scale.x = 0.1; // Point size
    marker_msg_.scale.y = 0.1;

    // Set the color (green in RGBA format)
    marker_msg_.color.r = 0.0;
    marker_msg_.color.g = 1.0;
    marker_msg_.color.b = 0.0;
    marker_msg_.color.a = 1.0;
    for (auto point : points)
    {
        geometry_msgs::msg::Point point1;
        point1.x = point.x();
        point1.y = point.y();
        point1.z = point.z();
        marker_msg_.points.push_back(point1);
    }
    landmark_publisher_->publish(marker_msg_);
}

void RosVisualizer::visualizeFrontier(const std::vector<Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID)
{
    // pointcloud for visualization purposes
    pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
    pcl::PointXYZI frontier_point_viz(50);

    // pointcloud for visualization purposes
    pcl::PointCloud<pcl::PointXYZI> all_frontier_cloud_viz;
    pcl::PointXYZI all_frontier_point_viz(50);

    for (const auto &frontier : frontier_list)
    {
        // load frontier into visualization poitncloud
        frontier_point_viz.x = frontier.getGoalPoint().x;
        frontier_point_viz.y = frontier.getGoalPoint().y;
        frontier_cloud_viz.push_back(frontier_point_viz);
    }

    for (const auto &frontier : every_frontier)
    {
        // load frontier into visualization poitncloud
        all_frontier_point_viz.x = frontier[0];
        all_frontier_point_viz.y = frontier[1];
        all_frontier_cloud_viz.push_back(all_frontier_point_viz);
    }

    // publish visualization point cloud
    sensor_msgs::msg::PointCloud2 frontier_viz_output;
    pcl::toROSMsg(frontier_cloud_viz, frontier_viz_output);
    frontier_viz_output.header.frame_id = globalFrameID;
    frontier_viz_output.header.stamp = rclcpp::Clock().now();
    frontier_cloud_pub_->publish(frontier_viz_output);

    // publish visualization point cloud (all frontiers)
    sensor_msgs::msg::PointCloud2 all_frontier_viz_output;
    pcl::toROSMsg(all_frontier_cloud_viz, all_frontier_viz_output);
    all_frontier_viz_output.header.frame_id = globalFrameID;
    all_frontier_viz_output.header.stamp = rclcpp::Clock().now();
    all_frontier_cloud_pub_->publish(all_frontier_viz_output);
}

void RosVisualizer::exportMapCoverage(std::vector<double> polygon_xy_min_max, int counter_value_, std::string mode_)
{
    if(costmap_ == nullptr)
    {
        throw std::runtime_error("You called the wrong constructor. Costmap is a nullptr");
    }
    RCLCPP_INFO_STREAM(logger_, "Point2: " << polygon_xy_min_max[2]);
    RCLCPP_INFO_STREAM(logger_, "Point0: " << polygon_xy_min_max[0]);

    RCLCPP_INFO_STREAM(logger_, "Point1: " << polygon_xy_min_max[1]);
    RCLCPP_INFO_STREAM(logger_, "Point3: " << polygon_xy_min_max[3]);

    RCLCPP_INFO_STREAM(logger_, "Point0: " << polygon_xy_min_max[0]);
    RCLCPP_INFO_STREAM(logger_, "Point2: " << polygon_xy_min_max[2]);
    RCLCPP_INFO_STREAM(logger_, "Costmap is " << costmap_);
    int x_unknown = abs(polygon_xy_min_max[2] - polygon_xy_min_max[0]) / costmap_->getResolution();
    int y_unknown = abs(polygon_xy_min_max[3] - polygon_xy_min_max[1]) / costmap_->getResolution();
    int unknown = x_unknown * y_unknown;
    RCLCPP_INFO_STREAM(logger_, "Total unknown cells is: " << unknown);
    int cell_count = 0;
    // int unknown = std::pow((polygon_xy_min_max[2] - polygon_xy_min_max[0]) / costmap_->getResolution(), 2);
    for (double y = polygon_xy_min_max[1]; y < polygon_xy_min_max[3]; y += costmap_->getResolution())
    {
        for (double x = polygon_xy_min_max[0]; x < polygon_xy_min_max[2]; x += costmap_->getResolution())
        {
            cell_count++;
            // Convert world coordinates to costmap grid coordinates
            unsigned int mx, my;
            if (costmap_->worldToMap(x, y, mx, my))
            {
                // Access the costmap value at grid coordinates (mx, my)
                unsigned char cost = costmap_->getCost(mx, my);
                if ((int)cost != 255)
                {
                    unknown--;
                }
            }
        }
    }
    RCLCPP_INFO_STREAM(logger_, "Cell count is: " << cell_count);
    RCLCPP_INFO_STREAM(logger_, "Total unknown post red cells is: " << unknown);
    std::ofstream file;
    file.open(static_cast<std::string>(logger_.get_name()).substr(0, 7) + "_" + std::to_string(counter_value_) + "_" + mode_ + "_frontier_map_data_coverage.csv", std::ios::app);
    if (!file.is_open())
    {
        RCLCPP_ERROR(logger_, "Failed to open the CSV file for writing.");
        return;
    }
    file << std::fixed;
    file << std::setprecision(2);
    file << node_->get_clock()->now().seconds() << "," << unknown << std::endl;
    file.close();
}

void RosVisualizer::frontierPlanViz(nav_msgs::msg::Path& path)
{
    frontier_plan_pub_->publish(path);
}