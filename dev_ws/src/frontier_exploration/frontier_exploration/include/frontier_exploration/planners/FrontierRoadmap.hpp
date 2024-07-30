#ifndef FRONTIER_ROADMAP_HPP_
#define FRONTIER_ROADMAP_HPP_

#include <vector>
#include <unordered_map>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/Helpers.hpp"
#include "frontier_exploration/colorize.hpp"
#include "frontier_exploration/planners/astar.hpp"
#include "frontier_exploration/rosVisualizer.hpp"

const double GRID_CELL_SIZE = 1.0; // Assuming each cell is 1x1 in size
const double RADIUS_TO_DECIDE_EDGES = 2.8; // a node within this radius of another node is considered a child of the other node.

namespace frontier_exploration
{
    class FrontierRoadMap
    {
    public:
        FrontierRoadMap(nav2_costmap_2d::Costmap2D *costmap);

        void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

        // Custom hash function for std::pair<int, int>
        struct spatialHash {
            template <class T1, class T2>
            std::size_t operator() (const std::pair<T1, T2> &pair) const {
                auto hash1 = std::hash<T1>{}(pair.first);
                auto hash2 = std::hash<T2>{}(pair.second);
                return hash1 ^ (hash2 << 1); // Combine the two hashes
            }
        };

        std::pair<int, int> getGridCell(double x, double y);

        void addNodes(const std::vector<Frontier> &frontiers, bool populateClosest);

        void constructGraph();

        void populateNodes(const std::vector<Frontier> &frontiers, bool populateClosest);

        void getNodesWithinRadius(const Frontier &interestNode, std::vector<Frontier> &closestNodeVector, const double radius);

        void getNodesWithinRadius(const geometry_msgs::msg::Point &interestPoint, std::vector<Frontier> &closestNodeVector, const double radius);

        void getClosestNode(const Frontier &interestNode, Frontier &closestNode);

        void getPlan(double xs, double ys, bool useClosestToStart, double xe, double ye, bool useClosestToEnd);

        void getPlan(Frontier &startNode, Frontier &endNode);

        void publishRoadMap();

        void publishPlan(const std::vector<std::shared_ptr<Node>> &plan);

        std::mutex& getRoadmapMutex()
        {
            return roadmap_mutex_;
        };

        std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash>& getRoadMap()
        {
            return roadmap_;
        };

    private:
        bool isConnectable(const Frontier &f1, const Frontier &f2);

        nav2_costmap_2d::Costmap2D *costmap_;
        std::unordered_map<std::pair<int, int>, std::vector<Frontier>, spatialHash> spatial_hash_map_;
        std::mutex spatial_hash_map_mutex_;
        std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> roadmap_;
        std::mutex roadmap_mutex_;
        double max_connection_length_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_roadmap_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_plan_;
        std::shared_ptr<FrontierRoadmapAStar> astar_planner_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
        std::vector<geometry_msgs::msg::Point> clicked_points_;
        std::shared_ptr<RosVisualizer> rosViz_;
    };
}

#endif // NODE_GRAPH_HPP_
