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

#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>
#include <frontier_msgs/srv/get_frontier_costs.hpp>
#include <frontier_multirobot_allocator/taskAllocator.hpp>
#include <frontier_exploration/colorize.hpp>

namespace frontier_exploration
{
    /**
     * @brief Server for frontier exploration action, runs the state machine associated with a
     * structured frontier exploration task and manages robot movement through bt-navigator.
     */
    class FrontierExplorationServer : public rclcpp::Node
    {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        /**
         * @brief Constructor for the server, sets up this node's ActionServer for exploration
         * and ActionClient to bt-navigator for robot movement.
         */
        FrontierExplorationServer();

        ~FrontierExplorationServer();

        /**
         * @brief Execute callback for actionserver, run after accepting a new goal
         * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
         */

        void buildBoundaryAndCenter();

        void processAllRobots(std::shared_ptr<TaskAllocator> taskAllocator, std::vector<frontier_msgs::msg::Frontier>& globalFrontierList, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> srv_res);

        void updateAssignedFrontiersAllRobots(frontier_msgs::msg::Frontier frontier);

        void run();
        
        void performBackupRotation();

        void performBackupReverse();

        void handle_multirobot_frontier_cost_request(
            std::shared_ptr<rmw_request_id_t> request_header,
            std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Request> request,
            std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Response> response);

        void nav2GoalFeedbackCallback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

        void nav2GoalResultCallback(const GoalHandleNav2::WrappedResult &result);

        void nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle);

        bool equateFrontierList(const std::vector<frontier_msgs::msg::Frontier>& list1, const std::vector<frontier_msgs::msg::Frontier>& list2);

    private:
        // ROS Internal
        std::shared_ptr<tf2_ros::Buffer> tf_listener_;
        rclcpp::CallbackGroup::SharedPtr explore_server_callback_group_;
        rclcpp::CallbackGroup::SharedPtr nav2_client_callback_group_;
        rclcpp::CallbackGroup::SharedPtr multirobot_service_callback_group_;
        rclcpp::CallbackGroup::SharedPtr assigned_frontiers_callback_group_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;
        geometry_msgs::msg::PolygonStamped explore_boundary_;
        geometry_msgs::msg::PointStamped explore_center_;

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

        bool currently_processing_ = false;
        std::mutex currently_processing_lock_;
        rclcpp::Client<frontier_msgs::srv::UpdateBoundaryPolygon>::SharedPtr updateBoundaryPolygon;
        rclcpp::Client<frontier_msgs::srv::GetNextFrontier>::SharedPtr getNextFrontier;
        rclcpp::Client<frontier_msgs::srv::GetNextFrontier>::SharedPtr getNextFrontierMultiRobot;
        rclcpp::Service<frontier_msgs::srv::GetFrontierCosts>::SharedPtr service_get_costs_;
        std::mutex update_assigned_frontier_lock_;
        std::vector<std::string> robot_namespaces_;
        std::string frontier_travel_point_;      ///< Used to set the frontier travel point. {Closest, Centroid, Middle}
        bool layer_configured_;
        bool use_custom_sim_;
        bool wait_for_other_robot_costs_;
        bool process_other_robots_;
        bool use_pose_from_multirobot_allocator_;
        // these are the frontiers traversed by this robot.
        std::vector<frontier_msgs::msg::Frontier> blacklisted_frontiers_;
        std::vector<std::string> config_;

    };
}