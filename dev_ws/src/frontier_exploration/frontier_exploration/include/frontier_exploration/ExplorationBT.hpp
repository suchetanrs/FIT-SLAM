#ifndef EXPLORE_SERVER_
#define EXPLORE_SERVER_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "frontier_msgs/srv/send_current_goal.hpp"
#include "frontier_multirobot_allocator/taskAllocator.hpp"
#include "frontier_exploration/colorize.hpp"
#include "frontier_exploration/bounded_explore_layer.hpp"
#include "frontier_exploration/FrontierSearchAllCells.hpp"
#include "frontier_exploration/FrontierSearch.hpp"
#include "frontier_exploration/Nav2Interface.hpp"
#include "frontier_exploration/Helpers.hpp"
#include "frontier_exploration/FullPathOptimizer.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace frontier_exploration
{
    struct RobotActiveGoals
    {
        std::mutex mutex;
        std::map<std::string, std::shared_ptr<geometry_msgs::msg::PoseStamped>> goals;
    };

    class FrontierExplorationServer
    {
    public:

        FrontierExplorationServer(rclcpp::Node::SharedPtr node);

        ~FrontierExplorationServer();

        void buildBoundaryAndCenter();

        void makeBTNodes();

        void handle_multirobot_current_goal_request(
            std::shared_ptr<rmw_request_id_t> request_header,
            std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Request> request,
            std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Response> response);

    private:
        // ROS Internal
        std::shared_ptr<tf2_ros::Buffer> tf_listener_;
        rclcpp::CallbackGroup::SharedPtr explore_server_callback_group_;
        rclcpp::CallbackGroup::SharedPtr multirobot_service_callback_group_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;

        // Nav2 related
        int nav2WaitTime_;
        std::mutex process_active_goals_lock_;
        std::shared_ptr<Nav2Interface> nav2_interface_;

        // Exploration related
        geometry_msgs::msg::PolygonStamped explore_boundary_;
        geometry_msgs::msg::PointStamped explore_center_;
        Frontier nextRoadMapParent_;

        std::shared_ptr<FrontierSearch> frontierSearchPtr_;
        std::shared_ptr<FullPathOptimizer> full_path_optimizer_;

        std::mutex moving_lock_;
        bool moving_;
        int retry_;

        rclcpp::Service<frontier_msgs::srv::SendCurrentGoal>::SharedPtr service_send_current_goal_;
        std::vector<std::string> robot_namespaces_;
        bool use_custom_sim_;
        std::vector<Frontier> blacklisted_frontiers_; // these are the frontiers traversed by this robot.
        std::vector<std::string> config_;
        int min_frontier_cluster_size_; ///< Minimum size of a frontier cluster.
        int max_frontier_cluster_size_;
        double max_frontier_distance_;
        std::shared_ptr<BoundedExploreLayer> bel_ptr_;
        bool process_other_robots_;
        RobotActiveGoals robot_active_goals_;
        std::shared_ptr<TaskAllocator> task_allocator_ptr_;
        rclcpp::Node::SharedPtr bt_node_;
        BT::BehaviorTreeFactory factory;
        BT::Blackboard::Ptr blackboard;
        BT::Tree behaviour_tree;
    };
}

#endif