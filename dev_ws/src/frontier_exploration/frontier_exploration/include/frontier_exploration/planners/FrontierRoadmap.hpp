#ifndef FRONTIER_ROADMAP_HPP_
#define FRONTIER_ROADMAP_HPP_

#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/Helpers.hpp"
#include "frontier_exploration/planners/astar.hpp"
#include "frontier_exploration/planners/FrontierRoadmap.hpp"
#include "frontier_exploration/util/event_logger.hpp"
#include "frontier_exploration/util/logger.hpp"
#include "frontier_exploration/util/rosVisualizer.hpp"

const double GRID_CELL_SIZE = 1.0;                           // Assuming each cell is 1x1 in size
const double RADIUS_TO_DECIDE_EDGES = 5.1;                   // a node within this radius of another node is considered a child of the other node.
const double MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES = 0.5;  // minimum distance between any node in the graph and the frontier node that will be added.
const double MIN_DISTANCE_BETWEEN_ROBOT_POSE_AND_NODE = 0.2; // minimum distance between any nodes in the graph and the robot pose that is going to be added.

namespace frontier_exploration
{
    struct RoadmapPlanResult
    {
        std::vector<std::shared_ptr<Node>> path;
        double path_length_m;
        bool path_exists;
    };

    class FrontierRoadMap
    {
    public:
        static FrontierRoadMap &getInstance()
        {
            std::lock_guard<std::mutex> lock(instanceMutex_);
            if (frontierRoadmapPtr == nullptr)
                throw std::runtime_error("Cannot de-reference a null FrontierRoadMap! :(");
            return *frontierRoadmapPtr;
        }

        static void createInstance(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
        {
            std::cout << "Creating instance" << std::endl;
            std::lock_guard<std::mutex> lock(instanceMutex_);
            if (frontierRoadmapPtr == nullptr)
                frontierRoadmapPtr.reset(new FrontierRoadMap(explore_costmap_ros));
        }

        void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

        // Custom hash function for std::pair<int, int>
        struct spatialHash
        {
            template <class T1, class T2>
            std::size_t operator()(const std::pair<T1, T2> &pair) const
            {
                auto hash1 = std::hash<T1>{}(pair.first);
                auto hash2 = std::hash<T2>{}(pair.second);
                return hash1 ^ (hash2 << 1); // Combine the two hashes
            }
        };

        std::pair<int, int> getGridCell(double x, double y);

        void populateNodes(const std::vector<Frontier> &frontiers, bool populateClosest, double min_distance_between_to_add);

        void addNodes(const std::vector<Frontier> &frontiers, bool populateClosest);

        void addRobotPoseAsNode(geometry_msgs::msg::Pose &start_pose_w, bool populateClosest);

        std::size_t countTotalItemsInSpatialMap()
        {
            std::size_t total_items = 0;
            spatial_hash_map_mutex_.lock(); // Lock the mutex to ensure thread safety
            for (const auto &cell : spatial_hash_map_)
            {
                // RosVisualizer::getInstance().visualizeSpatialHashMap(cell.second, "map");
                total_items += cell.second.size(); // Add the size of each grid's list to the total count
            }
            spatial_hash_map_mutex_.unlock(); // Unlock the mutex after iteration
            LOG_INFO("Total items in the map is: " << total_items);
            // RosVisualizer::getInstance().resetSpatialHashMap();
            return total_items;
        }

        void constructNewEdges(const std::vector<Frontier> &frontiers);

        void constructNewEdgeRobotPose(const geometry_msgs::msg::Pose &rPose);

        void reConstructGraph();

        void getNodesWithinRadius(const Frontier &interestNode, std::vector<Frontier> &closestNodeVector, const double radius);

        void getNodesWithinRadius(const geometry_msgs::msg::Point &interestPoint, std::vector<Frontier> &closestNodeVector, const double radius);

        void getClosestNodeInHashmap(const Frontier &interestNode, Frontier &closestNode);

        void getClosestNodeInRoadMap(const Frontier &interestNode, Frontier &closestNode);

        RoadmapPlanResult getPlan(double xs, double ys, bool useClosestToStart, double xe, double ye, bool useClosestToEnd);

        RoadmapPlanResult getPlan(Frontier &startNode, bool useClosestToStart, Frontier &endNode, bool useClosestToEnd);

        void publishRoadMap();

        void publishPlan(const std::vector<std::shared_ptr<Node>> &plan, float r, float g, float b);

        void publishPlan(const std::vector<Frontier> &plan);

        std::mutex &getRoadmapMutex()
        {
            return roadmap_mutex_;
        };

        std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> &getRoadMap()
        {
            return roadmap_;
        };

    private:
        FrontierRoadMap(const FrontierRoadMap &) = delete;
        FrontierRoadMap &operator=(const FrontierRoadMap &) = delete;
        FrontierRoadMap(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);
        static std::unique_ptr<FrontierRoadMap> frontierRoadmapPtr;
        static std::mutex instanceMutex_;

        bool isConnectable(const Frontier &f1, const Frontier &f2);
        nav2_costmap_2d::Costmap2D *costmap_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;

        std::unordered_map<std::pair<int, int>, std::vector<Frontier>, spatialHash> spatial_hash_map_;
        std::mutex spatial_hash_map_mutex_;

        std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> roadmap_;
        std::mutex roadmap_mutex_;
        double max_connection_length_;
        double max_frontier_distance_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_roadmap_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_roadmap_edges_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_roadmap_vertices_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_plan_;
        std::shared_ptr<FrontierRoadmapAStar> astar_planner_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
        std::vector<geometry_msgs::msg::Point> clicked_points_;
    };
}

#endif // NODE_GRAPH_HPP_
