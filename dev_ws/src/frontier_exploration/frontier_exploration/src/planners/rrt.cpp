#include "frontier_exploration/planners/rrt.hpp"
#include <queue>

namespace rrt_planner {

RRTPlanner::RRTPlanner(nav2_costmap_2d::Costmap2D* costmap, double max_dist, int max_iterations, rclcpp::Logger logger)
    : costmap_(costmap), max_dist_(max_dist), max_iterations_(max_iterations),
      rng_(std::random_device{}()), x_dist_(0.0, costmap_->getSizeInMetersX()),
      y_dist_(0.0, costmap_->getSizeInMetersY()), logger_(logger) {}

Node RRTPlanner::getRandomNode() {
    double x = x_dist_(rng_);
    double y = y_dist_(rng_);
    return Node(x, y);
}

int RRTPlanner::nearestNeighbor(const Node& random_node) {
    double min_dist = std::numeric_limits<double>::infinity();
    int nearest_index = -1;
    for (size_t i = 0; i < tree_.size(); ++i) {
        double dist = std::hypot(tree_[i].x - random_node.x, tree_[i].y - random_node.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }
    return nearest_index;
}

Node RRTPlanner::steer(const Node& nearest_node, const Node& random_node) {
    double theta = std::atan2(random_node.y - nearest_node.y, random_node.x - nearest_node.x);
    double new_x = nearest_node.x + max_dist_ * std::cos(theta);
    double new_y = nearest_node.y + max_dist_ * std::sin(theta);

    // Ensure new node is within map bounds
    new_x = std::clamp(new_x, 0.0, costmap_->getSizeInMetersX());
    new_y = std::clamp(new_y, 0.0, costmap_->getSizeInMetersY());

    return Node(new_x, new_y);
}

bool RRTPlanner::isCollisionFree(const Node& node1, const Node& node2) {
    // Check for obstacles along the line connecting node1 and node2
    double dx = node2.x - node1.x;
    double dy = node2.y - node1.y;
    double steps = std::max(std::abs(dx), std::abs(dy));
    double x_inc = dx / steps;
    double y_inc = dy / steps;
    double x = node1.x;
    double y = node1.y;

    for (int i = 0; i <= steps; ++i) {
        unsigned int mx, my;
        costmap_->worldToMap(x, y, mx, my);
        if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
            return false;
        }
        x += x_inc;
        y += y_inc;
    }
    return true;
}

std::vector<Node> RRTPlanner::buildPath(int goal_index) {
    std::vector<Node> path;
    int current_index = goal_index;
    while (current_index != -1) {
        path.push_back(tree_[current_index]);
        current_index = tree_[current_index].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> RRTPlanner::plan(double start_x, double start_y, double goal_x, double goal_y) {
    tree_.clear();
    tree_.emplace_back(start_x, start_y);
    RCLCPP_INFO_STREAM(logger_, "Start node: " << start_x << ", " << start_y);
    
    for (int i = 0; i < max_iterations_; ++i) {
        Node random_node = getRandomNode();
        RCLCPP_INFO_STREAM(logger_, "Random node: " << random_node.x << ", " << random_node.y);
        int nearest_index = nearestNeighbor(random_node);
        RCLCPP_INFO_STREAM(logger_, "Nearest index: " << nearest_index);
        Node new_node = steer(tree_[nearest_index], random_node);
        RCLCPP_INFO_STREAM(logger_, "New node: " << new_node.x << ", " << new_node.y << ", " << nearest_index);
        if (isCollisionFree(tree_[nearest_index], new_node)) {
            tree_.emplace_back(new_node.x, new_node.y, nearest_index);
            if (std::hypot(new_node.x - goal_x, new_node.y - goal_y) < max_dist_) {
                return buildPath(tree_.size() - 1);
            }
        }
        RCLCPP_INFO_STREAM(logger_, "-------------------");
    }
    return std::vector<Node>(); // Return empty path if no solution found
}

} // namespace rrt_planner