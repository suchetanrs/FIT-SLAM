#ifndef COST_CALCULATOR_HPP_
#define COST_CALCULATOR_HPP_

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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

#include <slam_msgs/srv/get_map.hpp>
#include <frontier_exploration/util/logger.hpp>
#include <frontier_exploration/util/rosVisualizer.hpp>
#include <frontier_exploration/Helpers.hpp>
#include <frontier_exploration/planners/FrontierRoadmap.hpp>
#include <frontier_exploration/fisher_information/FisherInfoManager.hpp>
#include <frontier_exploration/util/GeometryUtils.hpp>
#include <frontier_exploration/Parameters.hpp>


namespace frontier_exploration
{
     
    class FrontierCostCalculator {
    public:
        FrontierCostCalculator(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

        // ----------------Arrival information related--------------------
        /**
         * Sets arrival information to 0 if there is an error occurred.
         * Sets goal orientation to 0 rad if there is an error occurred.
         * If successful, sets the values accordingly.
        */
        void setArrivalInformationForFrontier(FrontierPtr& goal_point_w, std::vector<double>& polygon_xy_min_max);

        double setMaxArrivalInformation();

        // ----------------Planning related--------------------
        /**
         * Sets path length to inf if there is an error occurred or no path is found.
         * Sets Fisher information to 0 if there is an error occurred or no path is found.
         * If successful, sets the two values accordingly.
        */
        void setPlanForFrontier(geometry_msgs::msg::Pose start_pose_w, FrontierPtr& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        void setPlanForFrontierEuclidean(geometry_msgs::msg::Pose start_pose_w, FrontierPtr& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        void setPlanForFrontierRoadmap(geometry_msgs::msg::Pose start_pose_w, FrontierPtr& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);
        
        void updateRoadmapData(geometry_msgs::msg::Pose& start_pose_w, std::vector<FrontierPtr>& frontiers);
        
        // -----------------Random costs--------------
        double getRandomVal();

        void setRandomMetaData(FrontierPtr& goal_point_w);

        // -----------------For closest frontier implementation-----------------

        void setClosestFrontierMetaData(geometry_msgs::msg::Pose start_pose_w, FrontierPtr& goal_point_w,
                                                        std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        // --------------Other----------------
        void recomputeNormalizationFactors(FrontierPtr& frontier);

        double getMinPlanDistance()
        {
            return min_traversable_distance;
        };

        double getMaxPlanDistance()
        {
            return max_traversable_distance;
        };

        double getMinArrivalInformation()
        {
            return min_arrival_info_per_frontier;
        };

        double getMaxArrivalInformation()
        {
            return max_arrival_info_gt_;
        };

        void reset()
        {
            min_traversable_distance = std::numeric_limits<double>::max();
            max_traversable_distance = -1.0;
            min_arrival_info_per_frontier = std::numeric_limits<double>::max();
            max_arrival_info_per_frontier = -1.0;
        };

    private:
        // Add private methods or member variables if needed
        nav2_costmap_2d::Costmap2D *exploration_costmap_;
        // std::shared_ptr<RosVisualizer> rosVisualizer_;
        double min_traversable_distance = std::numeric_limits<double>::max();
        double max_traversable_distance = 0.0;
        double min_arrival_info_per_frontier = std::numeric_limits<double>::max();
        double max_arrival_info_per_frontier = 0.0;
        double robot_radius_;
        double max_arrival_info_gt_ = 0.0;
        double min_arrival_info_gt_ = 0.0;
        bool arrival_info_limits_set_ = false;

        double MAX_CAMERA_DEPTH;
        double DELTA_THETA;
        double CAMERA_FOV;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_; ///< Publisher for markers (path FOVs)
    };
};

#endif