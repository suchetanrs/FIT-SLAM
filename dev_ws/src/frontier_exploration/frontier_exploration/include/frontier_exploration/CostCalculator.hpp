#ifndef COST_CALCULATOR_HPP
#define COST_CALCULATOR_HPP

#include <algorithm>
#include <fstream>
#include <thread>
#include <random>
#include <unordered_map>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <frontier_exploration/Frontier.hpp>

#include <frontier_exploration/planners/planner.hpp>
// #include <frontier_exploration/planners/rrt.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

#include <slam_msgs/srv/get_map.hpp>
#include <frontier_exploration/colorize.hpp>
#include <frontier_exploration/rosVisualizer.hpp>
#include <frontier_exploration/Helpers.hpp>
#include <frontier_exploration/planners/FrontierRoadmap.hpp>
#include <frontier_exploration/FisherInfoManager.hpp>

namespace frontier_exploration
{
    class FrontierCostCalculator {
    public:
        FrontierCostCalculator(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);

        // ----------------Arrival information related--------------------
        /**
         * Sets arrival information to 0 if there is an error occurred.
         * Sets goal orientation to 0 rad if there is an error occurred.
         * If successful, sets the values accordingly.
        */
        void setArrivalInformationForFrontier(Frontier& goal_point_w, std::vector<double>& polygon_xy_min_max);

        // ----------------Planning related--------------------
        /**
         * Sets path length to inf if there is an error occurred or no path is found.
         * Sets Fisher information to 0 if there is an error occurred or no path is found.
         * If successful, sets the two values accordingly.
        */
        void setPlanForFrontier(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        void setPlanForFrontierEuclidean(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        void setPlanForFrontierRoadmap(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);
        
        void updateRoadmapData(std::vector<Frontier>& frontiers);
        
        // -----------------Random costs--------------
        double getRandomVal();

        void setRandomMetaData(Frontier& goal_point_w);

        // -----------------For closest frontier implementation-----------------

        void setClosestFrontierMetaData(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                        std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        // --------------Other----------------
        void recomputeNormalizationFactors(Frontier& frontier);

        double getMinPlanDistance()
        {
            return min_traversable_distance;
        };

        double getMaxArrivalInformation()
        {
            return max_arrival_info_per_frontier;
        };

        void reset()
        {
            min_traversable_distance = std::numeric_limits<double>::max();
            max_arrival_info_per_frontier = -1.0;
        };

    private:
        // Add private methods or member variables if needed
        rclcpp::Node::SharedPtr node_;
        // nav2_costmap_2d::Costmap2D *costmap_;
        nav2_costmap_2d::Costmap2D *exploration_costmap_;
        std::shared_ptr<FrontierRoadMap> roadmap_ptr_;
        std::shared_ptr<FisherInformationManager> fisherInfoManager_ptr_;
        rclcpp::Logger logger_ = rclcpp::get_logger("cost_calculator");
        std::shared_ptr<RosVisualizer> rosVisualizer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;                ///< Publisher for planned path to the frontiers.
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_; ///< Publisher for markers (path FOVs)
        std::shared_ptr<RosVisualizer> rosViz_;
        double min_traversable_distance = std::numeric_limits<double>::max();
        double max_arrival_info_per_frontier = 0.0;
    };
};

#endif