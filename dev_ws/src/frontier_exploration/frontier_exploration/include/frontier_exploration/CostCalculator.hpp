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
#include <frontier_exploration/planners/rrt.hpp>

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

namespace frontier_exploration
{
    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    class RayTracedCells
    {
    public:
        /**
         * @brief Constructor for RayTracedCells.
         *
         * @param costmap The reference to the costmap.
         * @param cells The vector of map locations to store ray-traced cells.
         */
        RayTracedCells(
            const nav2_costmap_2d::Costmap2D &costmap,
            std::vector<nav2_costmap_2d::MapLocation> &cells)
            : costmap_(costmap), cells_(cells)
        {
            hit_obstacle = false;
        }

        /**
         * @brief Function call operator to add unexplored cells to the list.
         * This operator adds cells that are currently unexplored to the list of cells.
         * i.e pushes the relevant cells back onto the list.
         * @param offset The offset of the cell to consider.
         */
        inline void operator()(unsigned int offset)
        {
            nav2_costmap_2d::MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            bool presentflag = false;
            for (auto item : cells_)
            {
                if (item.x == loc.x && item.y == loc.y)
                    presentflag = true;
            }
            if (presentflag == false)
            {
                if ((int)costmap_.getCost(offset) == 255 && hit_obstacle == false)
                {
                    cells_.push_back(loc);
                }
                if ((int)costmap_.getCost(offset) > 240 && (int)costmap_.getCost(offset) != 255)
                {
                    hit_obstacle = true;
                }
            }
        }

        /**
         * @brief Getter function for the vector of cells.
         * @return std::vector<nav2_costmap_2d::MapLocation> The vector of map locations.
         */
        std::vector<nav2_costmap_2d::MapLocation> getCells()
        {
            return cells_;
        }

    private:
        const nav2_costmap_2d::Costmap2D &costmap_;
        std::vector<nav2_costmap_2d::MapLocation> &cells_;
        bool hit_obstacle;
    };

    class FrontierCostCalculator {
    public:
        FrontierCostCalculator(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);

        // -----------------General helpers--------------------
        bool getTracedCells(double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells& cell_gatherer, double max_length);

        void bresenham2D(RayTracedCells at, unsigned int abs_da, unsigned int abs_db, int error_b,
                            int offset_a,
                            int offset_b, unsigned int offset,
                            unsigned int max_length,
                            int resolution_cut_factor);

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

        void getPlanForFrontierRRT(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);

        void setPlanForFrontierEuclidean(geometry_msgs::msg::Point start_point_w, Frontier& goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_);
        
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
            max_arrival_info_per_frontier = 0.0;
        };

    private:
        // Add private methods or member variables if needed
        rclcpp::Node::SharedPtr node_;
        // nav2_costmap_2d::Costmap2D *costmap_;
        nav2_costmap_2d::Costmap2D *exploration_costmap_;
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