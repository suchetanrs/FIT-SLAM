#ifndef EXPLORE_SERVER_
#define EXPLORE_SERVER_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "frontier_msgs/srv/send_current_goal.hpp"
#include "frontier_multirobot_allocator/taskAllocator.hpp"
#include "frontier_exploration/CostAssigner.hpp"
#include "frontier_exploration/FrontierSearchAllCells.hpp"
#include "frontier_exploration/FrontierSearch.hpp"

namespace frontier_exploration
{
    class FrontierExplorationServer : public rclcpp::Node
    {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        FrontierExplorationServer();

        ~FrontierExplorationServer();

        void buildBoundaryAndCenter();

        // new goal - set goal_complete to true, goal completed, aborted, cancelled - set goal_complete to false
        void processActiveGoalsAllRobots(bool goal_complete);

        std::shared_ptr<frontier_exploration::GetFrontierCostsResponse> processCostsAllRobots(std::shared_ptr<TaskAllocator> taskAllocator, std::vector<Frontier>& frontier_list,
                                   std::vector<std::vector<double>>& every_frontier, geometry_msgs::msg::PoseStamped& robot_pose, std::string robot_name);

        void run();

        void handle_multirobot_current_goal_request(
            std::shared_ptr<rmw_request_id_t> request_header,
            std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Request> request,
            std::shared_ptr<frontier_msgs::srv::SendCurrentGoal::Response> response);

        inline void setMoving(bool moving)
        {
            std::lock_guard<std::mutex> lock(moving_lock_);
            moving_ = moving;
        };

        inline bool isMoving()
        {
            std::lock_guard<std::mutex> lock(moving_lock_);
            return moving_;
        };
        
        // ------------------------NAV2 Related-----------------------
        void performBackupRotation();

        void performBackupReverse();

        void nav2GoalFeedbackCallback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

        void nav2GoalResultCallback(const GoalHandleNav2::WrappedResult &result);

        void nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle);

    private:
        // ROS Internal
        std::shared_ptr<tf2_ros::Buffer> tf_listener_;
        rclcpp::CallbackGroup::SharedPtr explore_server_callback_group_;
        rclcpp::CallbackGroup::SharedPtr nav2_client_callback_group_;
        rclcpp::CallbackGroup::SharedPtr multirobot_service_callback_group_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;

        // Nav2 related
        int nav2WaitTime_;
        std::mutex nav2Clientlock_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions nav2_goal_options_;
        std::mutex nav2_goal_lock_;
        std::shared_ptr<NavigateToPose::Goal> nav2_goal_;
        std::mutex process_active_goals_lock_;

        // Exploration related
        geometry_msgs::msg::PolygonStamped explore_boundary_;
        geometry_msgs::msg::PointStamped explore_center_;
        Frontier nextRoadMapParent_;

        std::mutex moving_lock_;
        bool moving_;
        int retry_;

        rclcpp::Service<frontier_msgs::srv::SendCurrentGoal>::SharedPtr service_send_current_goal_;
        std::vector<std::string> robot_namespaces_;
        std::mutex robot_active_goals_mutex_;
        std::map<std::string, std::shared_ptr<geometry_msgs::msg::PoseStamped>> robot_active_goals_;
        bool use_custom_sim_;
        std::vector<Frontier> blacklisted_frontiers_; // these are the frontiers traversed by this robot.
        std::vector<std::string> config_;
        int min_frontier_cluster_size_;          ///< Minimum size of a frontier cluster.
        int max_frontier_cluster_size_;
        std::shared_ptr<CostAssigner> bel_ptr_;
        bool process_other_robots_;
    };
}

#endif