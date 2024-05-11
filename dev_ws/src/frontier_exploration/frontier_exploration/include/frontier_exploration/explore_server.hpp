#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <action_msgs/msg/goal_status_array.hpp>

#include <frontier_msgs/action/explore_task.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_frontier_costs.hpp>

namespace frontier_exploration
{
    /**
     * @brief Server for frontier exploration action, runs the state machine associated with a
     * structured frontier exploration task and manages robot movement through bt-navigator.
     */
    class FrontierExplorationServer : public rclcpp::Node
    {
    public:
        using GoalHandleExplore = rclcpp_action::ServerGoalHandle<frontier_msgs::action::ExploreTask>;
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        /**
         * @brief Constructor for the server, sets up this node's ActionServer for exploration
         * and ActionClient to bt-navigator for robot movement.
         */
        FrontierExplorationServer();

        ~FrontierExplorationServer();

        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
            std::vector<rclcpp::Parameter> parameters);

    private:
        // ROS Internal
        std::shared_ptr<tf2_ros::Buffer> tf_listener_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        rclcpp::CallbackGroup::SharedPtr explore_server_callback_group_;
        rclcpp::CallbackGroup::SharedPtr nav2_client_callback_group_;
        rclcpp::CallbackGroup::SharedPtr multirobot_service_callback_group_;

        rclcpp_action::Server<frontier_msgs::action::ExploreTask>::SharedPtr action_server_;
        std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> frontier_goal;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;

        double frequency_, goal_aliasing_;
        bool success_, moving_;
        bool explore_client_requested_cancel_;
        int retry_;

        int nav2WaitTime_;
        std::mutex nav2Clientlock_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions nav2_goal_options_;
        NavigateToPose::Goal nav2_goal_;
        NavigateToPose::Goal old_goal; // previously was old_goal

        std::vector<std::string> robot_namespaces_;
        bool currently_processing_ = false;
        std::mutex currently_processing_lock_;
        rclcpp::Client<frontier_msgs::srv::UpdateBoundaryPolygon>::SharedPtr updateBoundaryPolygon;
        rclcpp::Client<frontier_msgs::srv::GetNextFrontier>::SharedPtr getNextFrontier;
        rclcpp::Client<frontier_msgs::srv::GetNextFrontier>::SharedPtr getNextFrontierMultiRobot;
        rclcpp::Service<frontier_msgs::srv::GetFrontierCosts>::SharedPtr service_get_costs_;
        bool layer_configured_;
        bool use_custom_sim_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExplore> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle);

        /**
         * @brief Execute callback for actionserver, run after accepting a new goal
         * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
         */

        void executeCb(const std::shared_ptr<GoalHandleExplore> goal_handle, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal);

        void performBackupRotation();

        void performBackupReverse();

        void handle_multirobot_frontier_cost_request(
            std::shared_ptr<rmw_request_id_t> request_header,
            std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Request> request,
            std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Response> response);

        void nav2GoalFeedbackCallback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

        void nav2GoalResultCallback(const GoalHandleNav2::WrappedResult &result);

        void nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle);
    };
}