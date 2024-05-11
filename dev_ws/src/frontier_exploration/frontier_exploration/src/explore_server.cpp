#include <frontier_exploration/explore_server.hpp>
#include <frontier_exploration/geometry_tools.hpp>

namespace frontier_exploration
{
    FrontierExplorationServer::FrontierExplorationServer() : Node("explore_server")
    {
        this->declare_parameter("goal_aliasing", 0.1);
        this->declare_parameter("retry_count", 30);
        this->declare_parameter("nav2_goal_timeout_sec", 35);
        this->declare_parameter("use_custom_sim", false);

        this->get_parameter("goal_aliasing", goal_aliasing_);
        this->get_parameter("retry_count", retry_);
        this->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);
        this->get_parameter("robot_namespaces", robot_namespaces_);
        this->get_parameter("use_custom_sim", use_custom_sim_);

        //--------------------------------------------NAV2 CLIENT RELATED--------------------------------
        nav2_client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        nav2Client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose", nav2_client_callback_group_); // was previously move_base, true

        nav2_goal_options_.feedback_callback = std::bind(&FrontierExplorationServer::nav2GoalFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        nav2_goal_options_.result_callback = std::bind(&FrontierExplorationServer::nav2GoalResultCallback, this, std::placeholders::_1);
        nav2_goal_options_.goal_response_callback = std::bind(&FrontierExplorationServer::nav2GoalResponseCallback, this, std::placeholders::_1);

        if (!nav2Client_->wait_for_action_server(std::chrono::seconds(50)))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available after waiting for %d seconds", 50);
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Nav2 action server available");
        }

        //--------------------------------------------EXPLORE SERVER RELATED----------------------------
        explore_server_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        action_server_ = rclcpp_action::create_server<frontier_msgs::action::ExploreTask>(
            this,
            "explore_action",
            std::bind(&FrontierExplorationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FrontierExplorationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FrontierExplorationServer::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            explore_server_callback_group_);

        explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explore_costmap", std::string{get_namespace()}, "explore_costmap");
        explore_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
        explore_costmap_ros_->activate();

        //------------------------------------------BOUNDED EXPLORE LAYER RELATED------------------------
        multirobot_service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_get_costs_ = this->create_service<frontier_msgs::srv::GetFrontierCosts>(
            "multirobot_get_frontier_costs", std::bind(&FrontierExplorationServer::handle_multirobot_frontier_cost_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_default, multirobot_service_callback_group_);

        updateBoundaryPolygon = this->create_client<frontier_msgs::srv::UpdateBoundaryPolygon>("explore_costmap/update_boundary_polygon");
        getNextFrontier = this->create_client<frontier_msgs::srv::GetNextFrontier>("explore_costmap/get_next_frontier");

        //---------------------------------------------ROS RELATED------------------------------------------
        tf_listener_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
                &FrontierExplorationServer::dynamicParametersCallback,
                this, std::placeholders::_1));
        layer_configured_ = false;

        RCLCPP_INFO(this->get_logger(), "FrontierExplorationServer::FrontierExplorationServer()");
    }

    FrontierExplorationServer::~FrontierExplorationServer()
    {
        RCLCPP_INFO(this->get_logger(), "FrontierExplorationServer::~FrontierExplorationServer()");
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
    }

    rcl_interfaces::msg::SetParametersResult FrontierExplorationServer::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters)
        {
            const auto &param_type = parameter.get_type();
            const auto &param_name = parameter.get_name();

            if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param_name == "goal_aliasing")
                {
                    goal_aliasing_ = parameter.as_double();
                }
            }
            else if (param_type == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                if (param_name == "retry_count")
                {
                    retry_ = parameter.as_int();
                }
                else if (param_name == "nav2_goal_timeout_sec")
                {
                    nav2WaitTime_ = parameter.as_int();
                }
            }
        }

        result.successful = true;
        return result;
    }

    rclcpp_action::GoalResponse FrontierExplorationServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal)
    {
        (void)uuid;
        frontier_goal = goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FrontierExplorationServer::handle_cancel(const std::shared_ptr<GoalHandleExplore> goal_handle)
    {
        nav2Client_->async_cancel_all_goals();
        RCLCPP_WARN(this->get_logger(), "Current exploration task cancelled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FrontierExplorationServer::handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle)
    {
        // std::thread{std::bind(&FrontierExplorationServer::executeCb, this, std::placeholders::_1, std::placeholders::_2), goal_handle, frontier_goal}.detach();
        executeCb(goal_handle, frontier_goal);
    }

    void FrontierExplorationServer::executeCb(const std::shared_ptr<GoalHandleExplore> goal_handle, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "FrontierExplorationServer::executeCb");
        success_ = false;
        moving_ = false;
        auto explore_action_res = std::make_shared<frontier_msgs::action::ExploreTask::Result>();

        // Don't compute a plan until costmap is valid (after clear costmap)
        rclcpp::Rate r(100);
        while (!explore_costmap_ros_->isCurrent())
        {
            r.sleep();
        }

        // wait for nav2 and costmap services
        if (!nav2Client_->wait_for_action_server(std::chrono::seconds(10)) || !updateBoundaryPolygon->wait_for_service(std::chrono::seconds(10)) || !getNextFrontier->wait_for_service(std::chrono::seconds(10)))
        {
            goal_handle->abort(explore_action_res);
            RCLCPP_ERROR(this->get_logger(), "Action server or BEL services is not available.");
            return;
        }

        // set region boundary on costmap
        if (rclcpp::ok() && goal_handle->is_active())
        {
            auto srv_req = std::make_shared<frontier_msgs::srv::UpdateBoundaryPolygon::Request>();
            auto srv_res = std::make_shared<frontier_msgs::srv::UpdateBoundaryPolygon::Response>();
            srv_req->explore_boundary = goal->explore_boundary;
            while (!updateBoundaryPolygon->wait_for_service(std::chrono::seconds(10)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(), "ROS shutdown request in between waiting for service.");
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for update boundary polygon service");
            }
            auto srv_future_id = updateBoundaryPolygon->async_send_request(srv_req);
            RCLCPP_INFO(this->get_logger(), "Adding update boundary polygon for spin.");
            if (srv_future_id.wait_for(std::chrono::seconds(15)) == std::future_status::ready)
            {
                srv_res = srv_future_id.get();
                RCLCPP_INFO(this->get_logger(), "Region boundary set");
                layer_configured_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for updateBoundaryPolygon called from within.");
                goal_handle->abort(explore_action_res);
                return;
            }
        }

        // The exploration is active until this while loop runs.
        while (rclcpp::ok() && goal_handle->is_active())
        {
            if (!layer_configured_)
            {
                RCLCPP_INFO(this->get_logger(), "Waitting for the layer to be configued");
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
            auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();

            // placeholder for next goal to be sent to move base
            geometry_msgs::msg::PoseStamped goal_pose;

            // get current robot pose in frame of exploration boundary
            geometry_msgs::msg::PoseStamped robot_pose;
            if (!explore_costmap_ros_->getRobotPose(robot_pose))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get robot position from explore costmap.");
            }
            srv_req->start_pose = robot_pose;
            // make this true only if you are providing a custom frontier list.
            srv_req->override_frontier_list = false;

            // evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::msg::PoseStamped eval_pose = srv_req->start_pose;
            if (eval_pose.header.frame_id != goal->explore_boundary.header.frame_id)
            {
                RCLCPP_ERROR(this->get_logger(), "Frames of robot pose and exploration boundary is not same");
                tf_listener_->transform(srv_req->start_pose, eval_pose, goal->explore_boundary.header.frame_id);
            }
            if (currently_processing_ == true)
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot process getNextFrontier from within, server busy.");
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            currently_processing_lock_.lock();
            currently_processing_ = true;
            currently_processing_lock_.unlock();
            RCLCPP_INFO_STREAM(this->get_logger(), "Currently processing is: " << currently_processing_);
            while (!getNextFrontier->wait_for_service(std::chrono::seconds(10)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(), "ROS shutdown request in between waiting for service.");
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for get next frontier service of the same robot");
            }
            auto srv_future_id = getNextFrontier->async_send_request(srv_req);
            if (srv_future_id.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
            {
                srv_res = srv_future_id.get();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for getNextFrontier called from within the robot.");
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            currently_processing_lock_.lock();
            currently_processing_ = false;
            currently_processing_lock_.unlock();

            // check if robot is not within exploration boundary and needs to return to center of search area
            if (goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position, goal->explore_boundary.polygon))
            {
                if (success_)
                {
                    RCLCPP_ERROR(this->get_logger(), "Robot left exploration boundary, returning to center");
                }
                else
                {
                    // (TODO: suchetan) evaluate this method if it works. Make exploration outside robot position and make it travel to center.
                    RCLCPP_ERROR(this->get_logger(), "Robot not initially in exploration boundary, traveling to center");
                }
                // get current robot position in frame of exploration center
                geometry_msgs::msg::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;
                if (eval_point.header.frame_id != goal->explore_center.header.frame_id)
                {
                    geometry_msgs::msg::PointStamped temp = eval_point;
                    RCLCPP_ERROR(this->get_logger(), "Frames not same");
                    tf_listener_->transform(temp, eval_point, goal->explore_center.header.frame_id);
                }

                // set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(eval_point.point, goal->explore_center.point));
            }
            else if (srv_res->success == true)
            {
                RCLCPP_INFO(this->get_logger(), "Next frontier Result success. Sending goal to Nav2");
                success_ = true;
                goal_pose = srv_res->next_frontier;
            }
            else if (srv_res->success == false)
            {
                // if no frontier found, check if search is successful
                RCLCPP_ERROR_STREAM(this->get_logger(), "Next frontier Result failure.");

                // search is succesful
                // (TODO: suchetan) evaluate the usefulness of success variable.
                if (retry_ == 0 && success_)
                {
                    RCLCPP_WARN(this->get_logger(), "Finished exploring room");
                    goal_handle->succeed(explore_action_res);
                    std::unique_lock<std::mutex> lock(nav2Clientlock_);
                    nav2Client_->async_cancel_goals_before(rclcpp::Clock().now());
                    return;
                }
                else if (retry_ == 0 || !rclcpp::ok())
                {
                    // search is not successful
                    RCLCPP_ERROR(this->get_logger(), "Failed exploration");
                    goal_handle->abort(explore_action_res);
                    return;
                }

                RCLCPP_WARN(this->get_logger(), "Retrying...");
                retry_--;
                // try to find frontier again
                if (retry_ <= this->get_parameter("retry_count").as_int() / 3.0)
                {
                    FrontierExplorationServer::performBackupRotation();
                    FrontierExplorationServer::performBackupReverse();
                    rclcpp::Rate(2).sleep();
                }
                continue;
            }
            // if above conditional does not escape this loop step, search has a valid goal_pose

            // check if new goal is close to old goal, hence no need to resend
            if ((!moving_ || !pointsNearby(old_goal.pose.pose.position, goal_pose.pose.position, goal_aliasing_ * 0.5)) && srv_res->success == true)
            {
                // This is to check if old pose is near new pose.
                old_goal.pose = goal_pose;

                nav2_goal_.pose = old_goal.pose;
                if (use_custom_sim_)
                    nav2_goal_.behavior_tree = get_namespace();
                nav2Client_->async_send_goal(nav2_goal_, nav2_goal_options_);
                moving_ = true;
                int waitCount_ = 0;
                bool minuteWait = false;
                while (moving_ && minuteWait == false && rclcpp::ok())
                {
                    rclcpp::Rate(1).sleep();
                    waitCount_++;
                    RCLCPP_INFO_STREAM(this->get_logger(), "Wait count goal is: " << waitCount_);
                    if (waitCount_ > nav2WaitTime_)
                    {
                        minuteWait = true;
                    }
                }
                // Moving is made false here to treat this as equal to nav2 feedback saying goal is reached (nav2 timed out here.).
                moving_ = false;
            }
        }

        // goal should never be active at this point
        if (!goal_handle->is_active())
        {
            RCLCPP_ERROR(this->get_logger(), "Goal Active when it should not be.");
        }
    }

    void FrontierExplorationServer::performBackupRotation()
    {
        RCLCPP_WARN(this->get_logger(), "FrontierExplorationServer::performBackupRotation");
        geometry_msgs::msg::PoseStamped robot_pose;
        explore_costmap_ros_->getRobotPose(robot_pose);
        tf2::Quaternion quaternion;
        tf2::fromMsg(robot_pose.pose.orientation, quaternion);
        double current_yaw = tf2::getYaw(quaternion);
        double desired_rad = M_PI / 2;
        double new_yaw = current_yaw + desired_rad;
        quaternion.setRPY(0, 0, new_yaw);
        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = robot_pose.header.frame_id;
        new_pose.pose.position = robot_pose.pose.position;
        new_pose.pose.orientation = tf2::toMsg(quaternion);
        RCLCPP_INFO(this->get_logger(), "Rotating in place to get new frontiers");
        nav2_goal_.pose = new_pose;
        nav2Client_->async_send_goal(nav2_goal_, nav2_goal_options_);
        moving_ = true;
        int waitCount_ = 0;
        bool minuteWait = false;
        while (moving_ && minuteWait == false && rclcpp::ok())
        {
            rclcpp::WallRate(1.0).sleep();
            RCLCPP_INFO(this->get_logger(), "Waiting to finish moving while rotating");
            waitCount_++;
            RCLCPP_INFO_STREAM(this->get_logger(), "Wait count rotation is: " << waitCount_);
            // TODO: Assign different wait times for goal and rotate.
            if (waitCount_ > nav2WaitTime_)
            {
                minuteWait = true;
            }
        }
        // Moving is made false here to treat this as equal to nav2 feedback saying goal is reached.
        moving_ = false;
    }

    void FrontierExplorationServer::performBackupReverse()
    {
        moving_ = true;
        // Create a publisher to publish messages on cmd_vel_nav topic
        auto publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = -0.5; // -0.5 m/s for reverse motion
        publisher->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        twist_msg.linear.x = 0.0;
        publisher->publish(twist_msg);
        // Log the action
        RCLCPP_WARN(this->get_logger(), "FrontierExplorationServer::performBackupReverse");
        moving_ = false;
    }

    void FrontierExplorationServer::handle_multirobot_frontier_cost_request(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Request> request,
        std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Response> response)
    {
        if (!layer_configured_)
        {
            RCLCPP_ERROR(this->get_logger(), "Frontier costs handle called but not configured so exiting...");
            response->success = false;
            return;
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "Multirobot frontier costs handle called with cp as: " << currently_processing_);
        if (currently_processing_ == true)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot process getNextFrontier from another robot, server busy.");
            response->success = false;
            return;
        }
        auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
        auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();

        // get current robot pose in frame of exploration boundary
        geometry_msgs::msg::PoseStamped robot_pose;
        explore_costmap_ros_->getRobotPose(robot_pose);

        srv_req->start_pose = robot_pose;
        srv_req->override_frontier_list = true;
        srv_req->frontier_list_to_override = request->requested_frontier_list;

        // evaluate if robot is within exploration boundary using robot_pose in boundary frame
        currently_processing_lock_.lock();
        currently_processing_ = true;
        currently_processing_lock_.unlock();
        RCLCPP_INFO_STREAM(this->get_logger(), "Set Currently processing to true: " << currently_processing_);
        while (!getNextFrontier->wait_for_service(std::chrono::seconds(10)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "ROS shutdown request in between waiting for service.");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for get next frontier service of the same robot called from another robot.");
        }
        auto srv_future_id = getNextFrontier->async_send_request(srv_req);

        RCLCPP_INFO(this->get_logger(), "Multi robot Sent request.");
        if (srv_future_id.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
        {
            srv_res = srv_future_id.get();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response of getNextFrontier Called from outside.");
        }
        RCLCPP_INFO(this->get_logger(), "Multi robot Recieved response.");
        if (srv_res->success == true)
        {
            response->success = true;
            response->frontier_costs = srv_res->frontier_costs;
            response->frontier_list = srv_res->frontier_list;
        }
        else if (srv_res->success == false)
        {
            response->success = false;
        }
        currently_processing_lock_.lock();
        currently_processing_ = false;
        currently_processing_lock_.unlock();
    }

    void FrontierExplorationServer::nav2GoalFeedbackCallback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        auto nav2_feedback_ = std::make_shared<NavigateToPose::Feedback>();
        nav2_feedback_->current_pose = feedback->current_pose;
        nav2_feedback_->distance_remaining = feedback->distance_remaining;
    }

    void FrontierExplorationServer::nav2GoalResultCallback(const GoalHandleNav2::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_WARN(this->get_logger(), "Goal succeeded");
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Nav2 internal fault! Nav2 aborted the goal!");
            FrontierExplorationServer::performBackupReverse();
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Nav2 goal canceled successfully");
            if (explore_client_requested_cancel_)
            {
                RCLCPP_INFO(this->get_logger(), "Explore client canceled goal");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Nav2 internal fault! Nav2 canceled the goal without explicit cancel request from Explore client!");
            }
            moving_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown nav2 goal result code");
        }
        RCLCPP_INFO(this->get_logger(), "Nav2 result callback executed");
    }

    void FrontierExplorationServer::nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2 server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2 server, waiting for result");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_exploration::FrontierExplorationServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}