#ifndef FRONTIER_COSTS_MANAGER_HPP_
#define FRONTIER_COSTS_MANAGER_HPP_

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
#include <frontier_exploration/colorize.hpp>
#include <frontier_exploration/rosVisualizer.hpp>
#include <frontier_exploration/CostCalculator.hpp>

namespace frontier_exploration
{
    class FrontierCostsManager
    {
    public:
        FrontierCostsManager(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

        /**
         * @param costTypes can take the values
         * "ArrivalInformation", 
         * "A*PlannerDistance" OR "EuclideanDistance" OR "RoadmapPlannerDistance", 
         * 
         * "RandomCosts",
         * 
         * "ClosestFrontier"
         */
        bool assignCosts(std::vector<Frontier> &frontier_list, std::vector<double> polygon_xy_min_max,
                             geometry_msgs::msg::Point start_point_w, std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data,
                             std::vector<std::vector<std::string>>& costTypes);

        void setFrontierBlacklist(std::vector<Frontier> &blacklist);

    private:
        nav2_costmap_2d::Costmap2D *costmap_;
        rclcpp::Node::SharedPtr frontier_costs_manager_node_;
        std::shared_ptr<RosVisualizer> rosVisualizer_;
        std::shared_ptr<FrontierCostCalculator> costCalculator_;
        std::unordered_map<Frontier, bool, FrontierHash, FrontierGoalPointEquality> frontier_blacklist_; ///< Stores the blacklisted frontiers.                                      ///< Variable used to give a unique value for each run. This is used as a prefix for the csv files.
        rclcpp::Logger logger_ = rclcpp::get_logger("frontier_costs_manager");
        bool planner_allow_unknown_;
        std::mutex blacklist_mutex_;

        double frontierDetectRadius_; ///< Sets the minimum detection radius for frontiers.
        double alpha_;                ///< Stores the alpha value used for weights.
        double beta_;                 ///< Stores the beta value used for weights.
        int N_best_for_u2_;           ///< Stores the number of frontiers to consider for u2 computation.
    };

} // namespace frontier_exploration

#endif
