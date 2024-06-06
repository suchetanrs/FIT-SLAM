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
        robot_namespaces_ = {};
        this->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
        this->declare_parameter("wait_for_other_robot_costs", false);
        this->declare_parameter("process_other_robots", false);
        this->declare_parameter("use_pose_from_multirobot_allocator", true);
        this->declare_parameter("frontier_travel_point", rclcpp::ParameterValue(std::string("closest")));

        this->get_parameter("goal_aliasing", goal_aliasing_);
        this->get_parameter("retry_count", retry_);
        this->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);
        this->get_parameter("use_custom_sim", use_custom_sim_);
        this->get_parameter("robot_namespaces", robot_namespaces_);
        this->get_parameter("wait_for_other_robot_costs", wait_for_other_robot_costs_);
        this->get_parameter("process_other_robots", process_other_robots_);
        this->get_parameter("use_pose_from_multirobot_allocator", use_pose_from_multirobot_allocator_);
        this->get_parameter("frontier_travel_point", frontier_travel_point_);

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
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Nav2 action server available", this->get_namespace()));
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
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR(COLOR_STR("FrontierExplorationServer::FrontierExplorationServer()", this->get_namespace()), this->get_namespace()));
    }

    FrontierExplorationServer::~FrontierExplorationServer()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::~FrontierExplorationServer()", this->get_namespace()));
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
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
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Current exploration task cancelled", this->get_namespace()));
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FrontierExplorationServer::handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle)
    {
        // std::thread{std::bind(&FrontierExplorationServer::executeCb, this, std::placeholders::_1, std::placeholders::_2), goal_handle, frontier_goal}.detach();
        executeCb(goal_handle, frontier_goal);
    }

    void FrontierExplorationServer::processAllRobots(std::shared_ptr<TaskAllocator> taskAllocator, std::vector<frontier_msgs::msg::Frontier>& globalFrontierList, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> srv_res)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("PROCESSING ALL ROBOTS!!!!!!!!!!!!!!!!!!!!!!", this->get_namespace()));
        // process for all robots
        for (auto robot_name : robot_namespaces_)
        {
            if(!process_other_robots_) continue;
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Picked: " + robot_name + " from " + this->get_namespace(), this->get_namespace()));
            if (robot_name != this->get_namespace())
            {
                rclcpp::Client<frontier_msgs::srv::GetFrontierCosts>::SharedPtr client_get_frontier_costs_;
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Processing robot: " + robot_name, this->get_namespace()));
                client_get_frontier_costs_ = this->create_client<frontier_msgs::srv::GetFrontierCosts>(robot_name + "/multirobot_get_frontier_costs");
                auto request_frontier_costs = std::make_shared<frontier_msgs::srv::GetFrontierCosts::Request>();
                request_frontier_costs->requested_frontier_list = srv_res->frontier_list;
                request_frontier_costs->robot_namespace = robot_name;
                request_frontier_costs->prohibited_frontiers = blacklisted_frontiers_;
                while (!client_get_frontier_costs_->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for get frontier costs service from " + robot_name, this->get_namespace()));
                }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Got get frontier costs service from" + robot_name, this->get_namespace()));
                auto result_frontier_costs = client_get_frontier_costs_->async_send_request(request_frontier_costs);
                std::shared_ptr<frontier_msgs::srv::GetFrontierCosts_Response> frontier_costs_srv_res;
                if (result_frontier_costs.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
                {
                    frontier_costs_srv_res = result_frontier_costs.get();
                    if (!frontier_costs_srv_res)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Did not recieve a response.");
                    }
                    if (frontier_costs_srv_res->success == true)
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Processed: " + robot_name, this->get_namespace()));
                        auto frontierCosts = frontier_costs_srv_res->frontier_costs;
                        // for (auto dist : frontier_costs_srv_res->frontier_distances)
                        // {
                        //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Frontier distances: " + std::to_string(dist), this->get_namespace()));
                        // }
                        // for (auto dist : frontier_costs_srv_res->frontier_arrival_information)
                        // {
                        //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Frontier arrival information: " + std::to_string(dist), this->get_namespace()));
                        // }
                        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Size of the frontier list (request) for " + robot_name + " is: " + std::to_string(request_frontier_costs->requested_frontier_list.size()), this->get_namespace()));
                        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Frontier costs size (response): " + std::to_string(frontierCosts.size()), this->get_namespace()));
                        taskAllocator->addRobotTasks(frontierCosts, frontier_costs_srv_res->frontier_distances, robot_name);
                        if(globalFrontierList.empty())
                        {
                            globalFrontierList = frontier_costs_srv_res->frontier_list;
                        }
                        else
                        {
                            if(!equateFrontierList(frontier_costs_srv_res->frontier_list, globalFrontierList))
                            {
                                auto gfl_size = globalFrontierList.size();
                                throw std::runtime_error("The frontier lists are not same for other robot" + std::to_string(gfl_size));
                            }
                        }
                    }
                    else if (frontier_costs_srv_res->success == false)
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Server returned false of robot " << robot_name << ". Probably busy.");
                        // add check to solve hungarian or not
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call the frontier costs handler for another robot.");
                }
            }
        }
    }

    void FrontierExplorationServer::executeCb(const std::shared_ptr<GoalHandleExplore> goal_handle, std::shared_ptr<const frontier_msgs::action::ExploreTask::Goal> goal)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::executeCb", this->get_namespace()));
        success_ = false;
        moving_ = false;
        auto explore_action_res = std::make_shared<frontier_msgs::action::ExploreTask::Result>();

        // Don't compute a plan until costmap is valid (after clear costmap)
        rclcpp::Rate r(100);
        while (!explore_costmap_ros_->isCurrent())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for costmap to be current", this->get_namespace()));
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
                    RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
                }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for update boundary polygon service", this->get_namespace()));
            }
            auto srv_future_id = updateBoundaryPolygon->async_send_request(srv_req);
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Adding update boundary polygon for spin.", this->get_namespace()));
            if (srv_future_id.wait_for(std::chrono::seconds(15)) == std::future_status::ready)
            {
                srv_res = srv_future_id.get();
                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Region boundary set", this->get_namespace()));
                layer_configured_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for updateBoundaryPolygon called from within.");
                goal_handle->abort(explore_action_res);
                return;
            }
        }

        rclcpp::sleep_for(std::chrono::seconds(45));

        // The exploration is active until this while loop runs.
        while (rclcpp::ok() && goal_handle->is_active())
        {
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Blacklist size: " + std::to_string(blacklisted_frontiers_.size()), this->get_namespace()));
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("NEW ITERATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!", this->get_namespace()));
            std::shared_ptr<TaskAllocator> taskAllocator = std::make_shared<TaskAllocator>();
            if (!layer_configured_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waitting for the layer to be configued", this->get_namespace()));
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
            auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();
            srv_req->prohibited_frontiers = blacklisted_frontiers_;

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
                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Cannot process getNextFrontier from within, server busy.", this->get_namespace()));
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            currently_processing_lock_.lock();
            currently_processing_ = true;
            currently_processing_lock_.unlock();
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Currently processing is: " + std::to_string(currently_processing_), this->get_namespace()));
            while (!getNextFrontier->wait_for_service(std::chrono::seconds(10)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
                }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for get next frontier service of the same robot", this->get_namespace()));
            }
            auto srv_future_id = getNextFrontier->async_send_request(srv_req);
            if (srv_future_id.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
            {
                srv_res = srv_future_id.get();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for getNextFrontier called from within the robot.");
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            currently_processing_lock_.lock();
            currently_processing_ = false;
            currently_processing_lock_.unlock();
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Size of the frontier list being used for multirobot is: " + std::to_string(srv_res->frontier_list.size()), this->get_namespace()));

            // check if robot is not within exploration boundary and needs to return to center of search area
            if (goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position, goal->explore_boundary.polygon))
            {
                // if success_ is true that means the robot was outside the boundary and the next goal was found.
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
                std::vector<frontier_msgs::msg::Frontier> globalFrontierList = {};

                processAllRobots(taskAllocator, globalFrontierList, srv_res);
                taskAllocator->addRobotTasks(srv_res->frontier_costs, srv_res->frontier_distances, this->get_namespace());
                // for (auto dist : srv_res->frontier_distances)
                // {
                //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Other Frontier distances: " + std::to_string(dist), this->get_namespace()));
                // }
                // for (auto dist : srv_res->frontier_arrival_information)
                // {
                //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Other Frontier arrival information: " + std::to_string(dist), this->get_namespace()));
                // }
                if(globalFrontierList.empty())
                {
                    globalFrontierList = srv_res->frontier_list;
                }
                else
                {
                    if(!equateFrontierList(globalFrontierList, srv_res->frontier_list))
                    {
                        throw std::runtime_error("The frontier lists are not same for current robot: " + static_cast<std::string>(this->get_namespace()) + " srv_res size: " + std::to_string(srv_res->frontier_list.size()) + " globalFrontier list size: " + std::to_string(globalFrontierList.size()));
                    }
                }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Solving hungarian", this->get_namespace()));
                taskAllocator->solveAllocationHungarian();
                // taskAllocator->solveAllocationMinPos();
                auto allocatedIndex = taskAllocator->getAllocatedTasks()[this->get_namespace()];
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated index" + std::to_string(allocatedIndex), this->get_namespace()));
                auto allocatedFrontier = globalFrontierList[allocatedIndex];
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier x:" + std::to_string(allocatedFrontier.initial.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier y:" + std::to_string(allocatedFrontier.initial.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier z:" + std::to_string(allocatedFrontier.initial.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier oz:" + std::to_string(allocatedFrontier.best_orientation.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier ow:" + std::to_string(allocatedFrontier.best_orientation.w), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier ox:" + std::to_string(allocatedFrontier.best_orientation.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier oy:" + std::to_string(allocatedFrontier.best_orientation.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier uid:" + std::to_string(allocatedFrontier.unique_id), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier x:" + std::to_string(srv_res->next_frontier.initial.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier y:" + std::to_string(srv_res->next_frontier.initial.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier z:" + std::to_string(srv_res->next_frontier.initial.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier oz:" + std::to_string(srv_res->next_frontier.best_orientation.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier ow:" + std::to_string(srv_res->next_frontier.best_orientation.w), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier ox:" + std::to_string(srv_res->next_frontier.best_orientation.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier oy:" + std::to_string(srv_res->next_frontier.best_orientation.y), this->get_namespace()));
                // if(allocatedFrontier.best_orientation != srv_res->next_frontier.pose.orientation)
                // {
                    // throw std::runtime_error("The orientations are not same. What are you doing?");
                // }
                // if(allocatedFrontier.initial != srv_res->next_frontier.pose.position)
                // {
                    // throw std::runtime_error("The initial positions are not same. What are you doing?");
                // }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Next frontier Result success. Sending goal to Nav2", this->get_namespace()));
                success_ = true;
                if(use_pose_from_multirobot_allocator_)
                {
                    RCLCPP_ERROR(this->get_logger(), "USING MULTIROBOT FRONTIER");
                    if (frontier_travel_point_ == "closest")
                    {
                        goal_pose.pose.position = allocatedFrontier.initial;
                    }
                    else if (frontier_travel_point_ == "middle")
                    {
                        goal_pose.pose.position = allocatedFrontier.middle;
                    }
                    else if (frontier_travel_point_ == "centroid")
                    {
                        goal_pose.pose.position = allocatedFrontier.centroid;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
                        goal_pose.pose.position = allocatedFrontier.initial;
                    }
                    goal_pose.pose.orientation = allocatedFrontier.best_orientation;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = rclcpp::Clock().now();
                    RCLCPP_ERROR_STREAM(this->get_logger(), "UID: " << allocatedFrontier.unique_id);
                    if (std::find(blacklisted_frontiers_.begin(), blacklisted_frontiers_.end(), allocatedFrontier) == blacklisted_frontiers_.end()) {
                        blacklisted_frontiers_.push_back(allocatedFrontier);
                    }
                }
                else
                {
                    if (frontier_travel_point_ == "closest")
                    {
                        goal_pose.pose.position = srv_res->next_frontier.initial;
                    }
                    else if (frontier_travel_point_ == "middle")
                    {
                        goal_pose.pose.position = srv_res->next_frontier.middle;
                    }
                    else if (frontier_travel_point_ == "centroid")
                    {
                        goal_pose.pose.position = srv_res->next_frontier.centroid;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
                        goal_pose.pose.position = srv_res->next_frontier.initial;
                    }
                    goal_pose.pose.orientation = srv_res->next_frontier.best_orientation;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = rclcpp::Clock().now();
                    RCLCPP_ERROR_STREAM(this->get_logger(), "UID: " << allocatedFrontier.unique_id);
                    if (std::find(blacklisted_frontiers_.begin(), blacklisted_frontiers_.end(), allocatedFrontier) == blacklisted_frontiers_.end()) {
                        blacklisted_frontiers_.push_back(srv_res->next_frontier);
                    }
                }
            }
            else if (srv_res->success == false)
            {
                // if no frontier found, check if search is successful
                RCLCPP_ERROR_STREAM(this->get_logger(), "Next frontier Result failure.");

                // search is succesful
                // (TODO: suchetan) evaluate the usefulness of success variable.
                if (retry_ == 0 && success_)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Finished exploring room", this->get_namespace()));
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

                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Retrying...", this->get_namespace()));
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
                    // rclcpp::Rate(1).sleep();
                    rclcpp::sleep_for(std::chrono::milliseconds(20));
                    waitCount_++;
                    RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Wait count goal is: " + std::to_string(waitCount_), this->get_namespace()));
                    if (waitCount_ > nav2WaitTime_)
                    {
                        nav2Client_->async_cancel_all_goals();
                        minuteWait = true;
                        FrontierExplorationServer::performBackupRotation();
                        FrontierExplorationServer::performBackupReverse();
                        break;
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
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::performBackupRotation", this->get_namespace()));
        moving_ = true;
        // Create a publisher to publish messages on cmd_vel_nav topic
        auto publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.angular.z = -0.5; // -0.5 m/s for reverse motion
        for (int i = 0; i <= 10; i++)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            publisher->publish(twist_msg);
        }
        twist_msg.angular.z = 0.0;
        for (int j = 0; j <= 3; j++)
            publisher->publish(twist_msg);
        // Log the action
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::performBackupReverse", this->get_namespace()));
        moving_ = false;
    }

    void FrontierExplorationServer::performBackupReverse()
    {
        moving_ = true;
        // Create a publisher to publish messages on cmd_vel_nav topic
        auto publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = -0.5; // -0.5 m/s for reverse motion
        for (int i = 0; i <= 40; i++)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            publisher->publish(twist_msg);
        }
        twist_msg.linear.x = 0.0;
        for (int j = 0; j <= 3; j++)
            publisher->publish(twist_msg);
        // Log the action
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::performBackupReverse", this->get_namespace()));
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
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Multirobot frontier costs handle called with cp as: " + std::to_string(currently_processing_), this->get_namespace()));
        if (wait_for_other_robot_costs_)
        {
            while (currently_processing_ == true)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Waiting for currently processing to become false", this->get_namespace()));
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else
        {
            if (currently_processing_ == true)
            {
                response->success = false;
                return;
            }
        }
        auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
        auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();

        // get current robot pose in frame of exploration boundary
        geometry_msgs::msg::PoseStamped robot_pose;
        explore_costmap_ros_->getRobotPose(robot_pose);

        srv_req->start_pose = robot_pose;
        srv_req->override_frontier_list = true;
        srv_req->frontier_list_to_override = request->requested_frontier_list;
        srv_req->prohibited_frontiers = request->prohibited_frontiers;

        // evaluate if robot is within exploration boundary using robot_pose in boundary frame
        currently_processing_lock_.lock();
        currently_processing_ = true;
        currently_processing_lock_.unlock();
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Set Currently processing to true: " + currently_processing_, this->get_namespace()));
        while (!getNextFrontier->wait_for_service(std::chrono::seconds(10)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
            }
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for get next frontier service of the same robot called from another robot.", this->get_namespace()));
        }
        auto srv_future_id = getNextFrontier->async_send_request(srv_req);

        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Multi robot Sent request.", this->get_namespace()));
        if (srv_future_id.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
        {
            srv_res = srv_future_id.get();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response of getNextFrontier Called from outside.");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Multi robot Recieved response.", this->get_namespace()));
        if (srv_res->success == true)
        {
            response->success = true;
            response->frontier_costs = srv_res->frontier_costs;
            response->frontier_list = srv_res->frontier_list;
            response->frontier_distances = srv_res->frontier_distances;
            response->frontier_arrival_information = srv_res->frontier_arrival_information;
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
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Goal succeeded", this->get_namespace()));
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Nav2 internal fault! Nav2 aborted the goal!");
            FrontierExplorationServer::performBackupRotation();
            FrontierExplorationServer::performBackupReverse();
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Nav2 goal canceled successfully", this->get_namespace()));
            if (explore_client_requested_cancel_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Explore client canceled goal", this->get_namespace()));
            }
            else
            {
                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Nav2 internal fault! Nav2 canceled the goal without explicit cancel request from Explore client!", this->get_namespace()));
            }
            moving_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown nav2 goal result code");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Nav2 result callback executed", this->get_namespace()));
    }

    void FrontierExplorationServer::nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2 server");
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Goal accepted by Nav2 server, waiting for result", this->get_namespace()));
        }
    }

    bool FrontierExplorationServer::equateFrontierList(const std::vector<frontier_msgs::msg::Frontier>& list1, const std::vector<frontier_msgs::msg::Frontier>& list2)
    {
        bool listflag = true;
        // Check if the lists have the same size
        if (list1.size() != list2.size()) {
            return false;
        }

        // Compare each corresponding element in both lists
        for (size_t i = 0; i < list1.size(); ++i) {
            auto f1 = list1[i];
            auto f2 = list2[i];
            if(!equateFrontiers(f1, f2, true))
            {
                listflag = false;
            }
        }
        return listflag;
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
