#ifndef BOUNDED_EXPLORE_LAYER_HPP_
#define BOUNDED_EXPLORE_LAYER_HPP_

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/geometry_utils.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/FrontierCostsManager.hpp"
#include "frontier_exploration/colorize.hpp"
#include "frontier_exploration/rosVisualizer.hpp"
#include "slam_msgs/srv/get_map.hpp"

namespace frontier_exploration
{

    struct GetFrontierCostsRequest
    {
        geometry_msgs::msg::PoseStamped start_pose;
        std::vector<Frontier> frontier_list;
        std::vector<std::vector<double>> every_frontier;
        std::vector<Frontier> prohibited_frontiers;       
    };

    struct GetFrontierCostsResponse
    {
        bool success;
        std::vector<Frontier> frontier_list;
        std::vector<double> frontier_costs;                     
        std::vector<double> frontier_distances;                 
        std::vector<double> frontier_arrival_information;       
        std::vector<double> frontier_path_information;          
    };

    class BoundedExploreLayer
    {
    public:
        BoundedExploreLayer(nav2_costmap_2d::LayeredCostmap* costmap);
        
        ~BoundedExploreLayer();

        bool updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped& explore_boundary);

        bool getFrontierCosts(std::shared_ptr<GetFrontierCostsRequest> requestData, std::shared_ptr<GetFrontierCostsResponse> resultData);
    
        void logMapData(std::shared_ptr<GetFrontierCostsRequest> requestData);

    protected:

        bool processOurApproach(
            std::vector<Frontier> &frontier_list,
            geometry_msgs::msg::Point& start_point_w);

    private:
        geometry_msgs::msg::Polygon polygon_;
        std::vector<double> polygon_xy_min_max_;
        std::string exploration_mode_;

        std::shared_ptr<frontier_exploration::FrontierCostsManager> frontierSelect_;

        std::string current_robot_namespace_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node;

        // ROS Clients
        rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr client_get_map_data2_;

        rclcpp::Node::SharedPtr internal_node_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> internal_executor_;
        nav2_costmap_2d::LayeredCostmap* layered_costmap_;
        int counter_;
        std::shared_ptr<RosVisualizer> rosViz_;
    };

}
#endif
