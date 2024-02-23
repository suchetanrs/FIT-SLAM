#ifndef TRAVERSABILITY_LAYER_HPP_
#define TRAVERSABILITY_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <memory>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "traversability_gridmap/traversabilityGrid.hpp"
#include <traversability_msgs/msg/pcl2_with_node_id.hpp>
#include <traversability_gridmap/global_traversability_map.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <slam_msgs/msg/map_graph.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

namespace traversability_gridmap
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class TraversabilityLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
{
public:
    TraversabilityLayer();
    ~TraversabilityLayer();

    /**
     * @brief Loads default values and initialize exploration costmap.
     */
    virtual void onInitialize();

    virtual void reset()
    {
        return;
    }

    virtual bool isClearable() 
    {
        return false;
    }

    virtual void onFootprintChanged()
    {
        return;
    }

    /**
     * @brief Calculate bounds of costmap window to update
     */
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y,
        double * max_x,
        double * max_y);

    /**
     * @brief Update requested costmap window
     */
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:

    sensor_msgs::msg::PointCloud2 transformPCL(sensor_msgs::msg::PointCloud2 msg, geometry_msgs::msg::TransformStamped transform);
    
    sensor_msgs::msg::PointCloud2::SharedPtr TransformPCLforAddition(geometry_msgs::msg::Pose pose_selected, sensor_msgs::msg::PointCloud2& pcl_ptr);

    void pointcloud_callback(const traversability_msgs::msg::PCL2WithNodeID::SharedPtr point_cloud);

    bool isPoseChanged(geometry_msgs::msg::Pose new_pose, geometry_msgs::msg::Pose old_pose, double percentage_threshold) {
        // Calculate position and orientation changes
        double position_change_x = (new_pose.position.x - old_pose.position.x) / old_pose.position.x * 100.0;
        double position_change_y = (new_pose.position.y - old_pose.position.y) / old_pose.position.y * 100.0;
        double position_change_z = (new_pose.position.z - old_pose.position.z) / old_pose.position.z * 100.0;

        double orientation_change_x = (new_pose.orientation.x - old_pose.orientation.x) / old_pose.orientation.x * 100.0;
        double orientation_change_y = (new_pose.orientation.y - old_pose.orientation.y) / old_pose.orientation.y * 100.0;
        double orientation_change_z = (new_pose.orientation.z - old_pose.orientation.z) / old_pose.orientation.z * 100.0;
        double orientation_change_w = (new_pose.orientation.w - old_pose.orientation.w) / old_pose.orientation.w * 100.0;

        // Store changes in an array
        std::vector<double> changes = {
            position_change_x,
            position_change_y,
            position_change_z,
            orientation_change_x,
            orientation_change_y,
            orientation_change_z,
            orientation_change_w
        };
        for(auto val : changes) {
            if(val > percentage_threshold)
                return true;
        }
        return false;
    }

    void publishtraversabilityMap(int nodeid, sensor_msgs::msg::PointCloud2& pcl_map, std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_, geometry_msgs::msg::Pose pose_selected);

    void updateMasterCostmap(nav2_costmap_2d::Costmap2D& master_grid, double robot_pose_x, double robot_pose_y, std::map<std::string, double> valueatposition, float positionx, float positiony);
    void updateLayer(nav2_costmap_2d::Costmap2D& master_grid, double robot_pose_x, double robot_pose_y, std::map<std::string, double> valueatposition, float positionx, float positiony);

    // Dynamic parameters handler
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    rclcpp::Subscription<traversability_msgs::msg::PCL2WithNodeID>::SharedPtr subscription_;
    rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr pubTraversability_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pubImage_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancy_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr planeLMSPub_;

    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;
    std::shared_ptr<traversabilityGrid> traversabilityMap;
    double half_size_ ;
    double resolution_;
    double security_distance_;
    double ground_clearance_;
    double max_slope_;
    double robot_height_;
    double robot_length_;
    double robot_width_;
    double draw_isodistance_each_;
    double squareHalfSideLength_;
    bool enabledLayer_;
    std::shared_ptr<GlobalTraversabilityMap> globalTraversabilityMap_;
    std::map<std::vector<double> , std::map<std::string, double>> globalProperties_;
};

}
#endif