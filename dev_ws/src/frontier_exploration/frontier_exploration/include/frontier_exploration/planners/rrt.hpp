#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include <vector>
#include <random>
#include <cmath>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace rrt_planner {

struct Node {
    double x, y;
    int parent; // Index of the parent node in the tree
    Node(double x, double y, int parent = -1) : x(x), y(y), parent(parent) {}
};

class RRTPlanner {
public:
    RRTPlanner(nav2_costmap_2d::Costmap2D* costmap, double max_dist, int max_iterations, rclcpp::Logger logger);
    std::vector<Node> plan(double start_x, double start_y, double goal_x, double goal_y);

private:
    nav2_costmap_2d::Costmap2D* costmap_;
    rclcpp::Logger logger_ = rclcpp::get_logger("RRT");
    double max_dist_;
    int max_iterations_;
    std::vector<Node> tree_;
    std::mt19937 rng_;
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;

    Node getRandomNode();
    int nearestNeighbor(const Node& random_node);
    Node steer(const Node& nearest_node, const Node& random_node);
    bool isCollisionFree(const Node& node1, const Node& node2);
    std::vector<Node> buildPath(int goal_index);
};

} // namespace rrt_planner

#endif // RRT_PLANNER_HPP
