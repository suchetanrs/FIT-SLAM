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

namespace frontier_exploration
{
    class FrontierRoadMap
    {
    public:
        FrontierRoadMap(nav2_costmap_2d::Costmap2D *costmap);

        void addNodes(const std::vector<Frontier> &frontiers);

        void constructGraph();

        const std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> &getRoadMap() const;

        void populateNodes(const std::vector<Frontier> &frontiers);

        void getNodesWithinRadius(const Frontier &interestNode, std::vector<Frontier> &closestNodeVector, double radius);

        void getClosestNode(const Frontier &interestNode, Frontier &closestNode);

        void getPlan(double xs, double ys, double xe, double ye);

        void publishRoadMap();

        void publishPlan(const std::vector<Node>& plan);

    private:
        bool isConnectable(const Frontier &f1, const Frontier &f2);

        nav2_costmap_2d::Costmap2D *costmap_;
        std::vector<Frontier> graph_nodes_;
        std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> roadmap_;
        std::map<Frontier, int, FrontierGoalPointEquality> assigned_children_;
        double max_connection_length_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_roadmap_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_plan_;
        std::shared_ptr<FrontierRoadmapAStar> astar_planner_;
    };
}

#endif // NODE_GRAPH_HPP_
