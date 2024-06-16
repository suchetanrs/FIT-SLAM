#ifndef FRONTIER_SELECTION_HPP_
#define FRONTIER_SELECTION_HPP_

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

#include <frontier_msgs/msg/frontier.hpp>

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
#include <frontier_exploration/cost_calculator.hpp>

namespace frontier_exploration
{
    /**
     * @brief Class representing a frontier with associated metadata.
     * This class encapsulates a frontier along with additional metadata
     * such as information, optimal orientation, and path length.
     */
    class FrontierWithMetaData
    {
    public:
        FrontierWithMetaData()
        {
            information_ = 0;
            theta_s_star_ = 0;
            path_length_ = 0;
        }
        /**
         * @brief Constructor for FrontierWithMetaData.
         *
         * @param frontier The frontier location.
         * @param information Information on arrival.
         * @param theta_s_star Optimal orientation for the frontier.
         * @param path_length Path length to the frontier.
         */
        FrontierWithMetaData(frontier_msgs::msg::Frontier frontier, int information, double theta_s_star, double path_length)
        {
            frontier_ = frontier;
            information_ = information;
            theta_s_star_ = theta_s_star;
            path_length_ = path_length;
        }

        void setCost(double cost)
        {
            cost_ = cost;
        }

        frontier_msgs::msg::Frontier frontier_; ///< frontier location
        size_t information_;                    ///< information on arrival
        double theta_s_star_;                   ///< optimal orientation for frontier
        double path_length_;                       ///< path length to frontier
        double cost_;
    };

    /**
     * @brief Struct representing the result of a selection process.
     * This struct contains the result of selecting a frontier, including the chosen frontier,
     * its orientation, success status, and costs associated with candidate frontiers.
     */
    struct SelectionResult
    {
        bool success;
        std::map<frontier_msgs::msg::Frontier, FrontierWithMetaData, FrontierLessThan> frontier_costs;
    };

    /**
     * @brief Functor class for comparing frontiers based on their associated costs.
     * This class provides a functor for comparing frontiers based on the costs associated with them.
     */
    class FrontierU1ComparatorCost
    {
    public:
        /**
         * @brief Function call operator for comparing frontiers.
         *
         * This operator compares two pairs of frontier-metadata and cost based on their associated costs.
         *
         * @param a The first pair of frontier-metadata and cost.
         * @param b The second pair of frontier-metadata and cost.
         * @return bool True if the cost associated with the first pair is
         * less than the cost associated with the second pair, false otherwise.
         */
        bool operator()(const std::pair<FrontierWithMetaData, double> &a, const std::pair<FrontierWithMetaData, double> &b) const
        {
            return a.second < b.second;
        }
    };

    /**
     * @brief Class for frontier selection node.
     * This class handles the selection of frontiers based on various criteria.
     */
    class FrontierSelectionNode
    {
    public:
        /**
         * @brief Constructor for FrontierSelectionNode.
         *
         * @param node Pointer to the node.
         * @param costmap Pointer to the costmap.
         */
        FrontierSelectionNode(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);

        /**
         * @brief Selects a frontier based on the count of unknown cells around the frontier.
         *
         * @param frontier_list List of frontiers.
         * @param polygon_xy_min_max Polygon's XY min-max. Order of polygon points is : minx, miny, maxx, maxy
         * @param res Response object.
         * @param start_point_w Start point in world frame.
         * @param map_data Map data from the SLAM.
         * @param exploration_costmap Traversability costmap.
         * @return SelectionResult The selected frontier along with success status.
         */
        SelectionResult selectFrontierOurs(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            std::vector<double> polygon_xy_min_max,
            geometry_msgs::msg::Point start_point_w,
            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data);

        /**
         * @brief Selects the closest frontier.
         *
         * @param frontier_list List of frontiers.
         * @param every_frontier Every frontier.
         * @param res Response object.
         * @param globalFrameID Global frame ID.
         * @return std::pair<frontier_msgs::msg::Frontier, bool> The selected frontier along with success status.
         */
        std::pair<frontier_msgs::msg::Frontier, bool> selectFrontierClosest(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list);

        /**
         * @brief Selects a random frontier.
         *
         * @param frontier_list List of frontiers.
         * @param every_frontier Every frontier.
         * @param res Response object.
         * @param globalFrameID Global frame ID.
         * @return std::pair<frontier_msgs::msg::Frontier, bool> The selected frontier along with success status.
         */
        std::pair<frontier_msgs::msg::Frontier, bool> selectFrontierRandom(
            std::vector<frontier_msgs::msg::Frontier> &frontier_list);

        void setFrontierBlacklist(std::vector<frontier_msgs::msg::Frontier>& blacklist);

        void frontierPlanViz(nav_msgs::msg::Path path);

    private:
        // Visualization related.
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pose_array_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;
        nav2_costmap_2d::Costmap2D *costmap_;
        rclcpp::Node::SharedPtr frontier_selection_node_;
        std::shared_ptr<RosVisualizer> rosVisualizer_;
        std::shared_ptr<FrontierCostCalculator> costCalculator_;
        std::unordered_map<frontier_msgs::msg::Frontier, bool, FrontierHash, FrontierEquality> frontier_blacklist_; ///< Stores the blacklisted frontiers.                                      ///< Variable used to give a unique value for each run. This is used as a prefix for the csv files.
        rclcpp::Logger logger_ = rclcpp::get_logger("frontier_selection");
        bool planner_allow_unknown_;
        bool use_planning_;
        std::mutex blacklist_mutex_;

        double frontierDetectRadius_; ///< Sets the minimum detection radius for frontiers.
        double alpha_;                ///< Stores the alpha value used for weights.
        double beta_;                 ///< Stores the beta value used for weights.
        int N_best_for_u2_;           ///< Stores the number of frontiers to consider for u2 computation.
    };

} // namespace frontier_exploration

#endif
