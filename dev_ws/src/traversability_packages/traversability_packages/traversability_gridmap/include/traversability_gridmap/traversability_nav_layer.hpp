#ifndef TRAVERSABILITY_LAYER_HPP_
#define TRAVERSABILITY_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>

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
#include <rtabmap_msgs/msg/map_graph.hpp>

#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace traversability_gridmap
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class TraversabilityNavLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
{
public:
    TraversabilityNavLayer();
    ~TraversabilityNavLayer();

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

    // Dynamic parameters handler
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

    unsigned char interpretBinValue(unsigned char value);


    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    nav2_msgs::msg::Costmap latest_costmap_;
    unsigned char traversability_lethal_threshold_;
    double squareHalfSideLength_;

};

}
#endif