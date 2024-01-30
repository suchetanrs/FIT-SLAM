#ifndef FRONTIER_SELECTION_HPP_
#define FRONTIER_SELECTION_HPP_

#include <algorithm>
#include <fstream>
#include <thread>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>

#include <frontier_exploration/planner.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <rtabmap_msgs/srv/get_nodes_in_radius.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>

#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

#include <rtabmap_msgs/srv/get_map2.hpp>
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
        /**
         * @brief Constructor for FrontierWithMetaData.
         *
         * @param frontier The frontier location.
         * @param information Information on arrival.
         * @param theta_s_star Optimal orientation for the frontier.
         * @param path_length Path length to the frontier.
         */
        FrontierWithMetaData(frontier_msgs::msg::Frontier frontier, int information, double theta_s_star, int path_length)
        {
            frontier_ = frontier;
            information_ = information;
            theta_s_star_ = theta_s_star;
            path_length_ = path_length;
        }

        /**
         * @brief Less-than operator for comparing instances of FrontierWithMetaData.
         *
         * @param other Another instance of FrontierWithMetaData to compare against.
         * @return True if this instance's path length is less than the other's, false otherwise.
         */
        bool operator<(const FrontierWithMetaData &other) const
        {
            // Compare based on some criteria, e.g., information
            return path_length_ < other.path_length_;
        }

        frontier_msgs::msg::Frontier frontier_; ///< frontier location
        size_t information_;                    ///< information on arrival
        double theta_s_star_;                   ///< optimal orientation for frontier
        int path_length_;                       ///< path length to frontier
    };

    /**
     * @brief Struct representing a path with associated information.
     * This struct encapsulates a path along with the total information associated with it.
     */
    struct PathWithInfo
    {
        nav_msgs::msg::Path path;
        double information_total;
    };

    /**
     * @brief Struct representing the result of a selection process.
     * This struct contains the result of selecting a frontier, including the chosen frontier,
     * its orientation, success status, and costs associated with candidate frontiers.
     */
    struct SelectionResult
    {
        frontier_msgs::msg::Frontier frontier;
        geometry_msgs::msg::Quaternion orientation;
        bool success;
        std::map<FrontierWithMetaData, double> frontier_costs;
    };

    /**
     * @brief Class for ray tracing operations on a costmap.
     * This class provides functionality for ray tracing operations on a given costmap.
     */
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

    /**
     * @brief Functor class for comparing frontiers based on their associated costs.
     * This class provides a functor for comparing frontiers based on the costs associated with them.
     */
    class FrontierU1Comparator
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
         * @param node Pointer to the lifecycle node.
         * @param costmap Pointer to the costmap.
         */
        FrontierSelectionNode(rclcpp_lifecycle::LifecycleNode::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap);

        /**
         * @brief Function to get the sign of an integer.
         * @return integer value with the sign.
         */
        inline int sign(int x)
        {
            return x > 0 ? 1.0 : -1.0;
        }

        /**
         * @brief Performs 2D Bresenham ray tracing.
         *
         * @param at RayTracedCells object.
         * @param abs_da Absolute value of delta A.
         * @param abs_db Absolute value of delta B.
         * @param error_b Error in B.
         * @param offset_a Offset in A.
         * @param offset_b Offset in B.
         * @param offset Offset of the cell.
         * @param max_length Maximum length.
         * @param resolution_cut_factor Resolution cut factor along the x major or y major.
         */
        void bresenham2D(RayTracedCells at, unsigned int abs_da, unsigned int abs_db, int error_b,
                         int offset_a,
                         int offset_b, unsigned int offset,
                         unsigned int max_length,
                         int resolution_cut_factor);

        /**
         * @brief Selects a frontier based on the count of unknown cells around the frontier.
         *
         * @param frontier_list List of frontiers.
         * @param polygon_xy_min_max Polygon's XY min-max. Order of polygon points is : minx, miny, maxx, maxy
         * @param res Response object.
         * @param start_point_w Start point in world frame.
         * @param map_data Map data from the SLAM.
         * @param traversability_costmap Traversability costmap.
         * @return SelectionResult The selected frontier along with success status.
         */
        SelectionResult selectFrontierOurs(
            const std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            std::vector<double> polygon_xy_min_max,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
            geometry_msgs::msg::Point start_point_w,
            std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data,
            nav2_costmap_2d::Costmap2D *traversability_costmap);

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
            const std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            const std::vector<std::vector<double>> &every_frontier,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
            std::string globalFrameID);

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
            const std::vector<frontier_msgs::msg::Frontier> &frontier_list,
            const std::vector<std::vector<double>> &every_frontier,
            std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res,
            std::string globalFrameID);

        /**
         * @brief Visualizes frontiers.
         *
         * @param frontier_list List of frontiers.
         * @param every_frontier Every frontier.
         * @param globalFrameID Global frame ID.
         */
        void visualizeFrontier(const std::vector<frontier_msgs::msg::Frontier> &frontier_list,
                               const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID);

        /**
         * @brief Exports the latest map coverage in terms of cells to a csv file.
         *
         * @param polygon_xy_min_max Polygon's XY min-max. Order of polygon points is : minx, miny, maxx, maxy
         * @param startTime Start time.
         */
        void exportMapCoverage(std::vector<double> polygon_xy_min_max, std::chrono::_V2::system_clock::time_point startTime);

        /**
         * @brief Gets a plan for a frontier along with the information.
         *
         * @param start_point_w Start point in world frame.
         * @param goal_point_w Goal point in world frame.
         * @param map_data Map data.
         * @param compute_information Whether to compute information.
         * @return std::pair<PathWithInfo, bool> The planned path along with success status.
         */
        std::pair<PathWithInfo, bool> getPlanForFrontier(geometry_msgs::msg::Point start_point_w, frontier_msgs::msg::Frontier goal_point_w,
                                                         std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data, bool compute_information);

    private:
        // ROS Publishers.
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;     ///< Publisher for frontier cloud.
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_frontier_cloud_pub_; ///< Publisher for every frontier cloud.
        // Visualization related.
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;                ///< Publisher for planned path to the frontiers.
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_; ///< Publisher for markers (path FOVs)
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr landmark_publisher_;   ///< Publisher for landmarks in the path FOVs
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr viz_pose_publisher_;   ///< Publisher for the best pose after u1 computation.

        // ROS Services.
        rclcpp::Client<rtabmap_msgs::srv::GetNodesInRadius>::SharedPtr get_nodes_in_radius_client_;

        nav2_costmap_2d::Costmap2D *costmap_;
        nav2_costmap_2d::Costmap2D *traversability_costmap_;
        rclcpp::Node::SharedPtr frontier_selection_node_;
        std::vector<frontier_msgs::msg::Frontier> frontier_blacklist_; ///< Stores the blacklisted frontiers.
        int counter_value_;                                            ///< Variable used to give a unique value for each run. This is used as a prefix for the csv files.
        std::string mode_;
        rclcpp::Logger logger_ = rclcpp::get_logger("frontier_selection");
        bool planner_allow_unknown_;

        double frontierDetectRadius_; ///< Sets the minimum detection radius for frontiers.
        double alpha_;                ///< Stores the alpha value used for weights.
        double beta_;                 ///< Stores the beta value used for weights.
        int N_best_for_u2_;           ///< Stores the number of frontiers to consider for u2 computation.
    };

} // namespace frontier_exploration

#endif
