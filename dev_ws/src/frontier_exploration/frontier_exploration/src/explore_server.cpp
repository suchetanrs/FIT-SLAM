#include <frontier_exploration/explore_server.hpp>
#include <frontier_exploration/geometry_tools.hpp>

#if defined(FRONTIER_POINT_CENTROID) + defined(FRONTIER_POINT_MIDDLE) + defined(FRONTIER_POINT_INITIAL) > 1
    #error "Only one of FRONTIER_POINT_CENTROID, FRONTIER_POINT_MIDDLE, or FRONTIER_POINT_INITIAL can be defined at a time."
#elif !defined(FRONTIER_POINT_CENTROID) && !defined(FRONTIER_POINT_MIDDLE) && !defined(FRONTIER_POINT_INITIAL)
    #error "One of FRONTIER_POINT_CENTROID, FRONTIER_POINT_MIDDLE, or FRONTIER_POINT_INITIAL must be defined."
#endif

namespace frontier_exploration
{
    void print_nodes(const Kdtree::KdNodeVector &nodes) {
    size_t i,j;
    for (i = 0; i < nodes.size(); ++i) {
        if (i > 0)
        cout << " ";
        cout << "(";
        for (j = 0; j < nodes[i].point.size(); j++) {
        if (j > 0)
            cout << ",";
        cout << nodes[i].point[j];
        }
        cout << ")";
    }
    cout << endl;
    }

    FrontierExplorationServer::FrontierExplorationServer() : Node("explore_server")
    {
        this->declare_parameter("retry_count", 30);
        this->declare_parameter("nav2_goal_timeout_sec", 35);
        this->declare_parameter("use_custom_sim", false);
        robot_namespaces_ = {};
        this->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
        this->declare_parameter("wait_for_other_robot_costs", false);
        this->declare_parameter("process_other_robots", false);
        this->declare_parameter("use_pose_from_multirobot_allocator", true);
        this->declare_parameter("frontier_travel_point", rclcpp::ParameterValue(std::string("closest")));
        config_ = {"10.0", "10.0", "10.0", "-10.0", "-10.0", "-10.0", "-10.0", "10.0"};
        this->declare_parameter("config", rclcpp::ParameterValue(config_));
        this->declare_parameter("min_frontier_cluster_size", rclcpp::ParameterValue(1));
        this->declare_parameter("max_frontier_cluster_size", rclcpp::ParameterValue(20));

        this->get_parameter("retry_count", retry_);
        this->get_parameter("nav2_goal_timeout_sec", nav2WaitTime_);
        this->get_parameter("use_custom_sim", use_custom_sim_);
        this->get_parameter("robot_namespaces", robot_namespaces_);
        this->get_parameter("wait_for_other_robot_costs", wait_for_other_robot_costs_);
        this->get_parameter("process_other_robots", process_other_robots_);
        this->get_parameter("use_pose_from_multirobot_allocator", use_pose_from_multirobot_allocator_);
        this->get_parameter("config", config_);
        this->get_parameter("min_frontier_cluster_size", min_frontier_cluster_size_);
        this->get_parameter("max_frontier_cluster_size", max_frontier_cluster_size_);
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

        explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explore_costmap", std::string{get_namespace()}, "explore_costmap");
        explore_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
        explore_costmap_ros_->activate();
        explore_costmap_ros_->getLayeredCostmap();

        bel_ptr_ = std::make_shared<BoundedExploreLayer>(explore_costmap_ros_->getLayeredCostmap());

        //------------------------------------------BOUNDED EXPLORE LAYER RELATED------------------------
        multirobot_service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // service_get_costs_ = this->create_service<frontier_msgs::srv::GetFrontierCosts>(
        //     "multirobot_get_frontier_costs", std::bind(&FrontierExplorationServer::handle_multirobot_frontier_cost_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        //     rmw_qos_profile_default, multirobot_service_callback_group_);

        service_get_current_goal_ = this->create_service<frontier_msgs::srv::GetCurrentGoal>(
            "multirobot_get_current_goal", std::bind(&FrontierExplorationServer::handle_multirobot_current_goal_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_default, multirobot_service_callback_group_);

        //---------------------------------------------ROS RELATED------------------------------------------
        tf_listener_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR(COLOR_STR("FrontierExplorationServer::FrontierExplorationServer()", this->get_namespace()), this->get_namespace()));
        buildBoundaryAndCenter();
    }

    void FrontierExplorationServer::buildBoundaryAndCenter()
    {
        explore_boundary_.header.frame_id = "map";
        explore_boundary_.header.stamp = rclcpp::Clock().now();
        for (int i = 0; i < config_.size(); i += 2)
        {
            geometry_msgs::msg::Point32 point;
            point.x = std::stof(config_[i]);
            point.y = std::stof(config_[i + 1]);
            explore_boundary_.polygon.points.push_back(point);
        }
        for (const auto &point : explore_boundary_.polygon.points)
        {
            RCLCPP_INFO(this->get_logger(), "Sending Polygon from config x: %f, y: %f, z: %f",
                        point.x, point.y, point.z);
        }

        explore_center_.header.frame_id = "map";
        explore_center_.point.x = 5.5;
        explore_center_.point.y = 5.5;
    }

    FrontierExplorationServer::~FrontierExplorationServer()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::~FrontierExplorationServer()", this->get_namespace()));
        explore_costmap_ros_->deactivate();
        explore_costmap_ros_->cleanup();
        explore_costmap_thread_.reset();
    }

    // void FrontierExplorationServer::processAllRobots(std::shared_ptr<TaskAllocator> taskAllocator, std::vector<frontier_msgs::msg::Frontier> &globalFrontierList, std::shared_ptr<GetNextFrontierResponse> srv_res)
    // {
    //     RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("PROCESSING ALL ROBOTS!!!!!!!!!!!!!!!!!!!!!!", this->get_namespace()));
    //     // process for all robots
    //     for (auto robot_name : robot_namespaces_)
    //     {
    //         if (!process_other_robots_)
    //             continue;
    //         RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Picked: " + robot_name + " from " + this->get_namespace(), this->get_namespace()));
    //         if (robot_name != this->get_namespace())
    //         {
    //             rclcpp::Client<frontier_msgs::srv::GetFrontierCosts>::SharedPtr client_get_frontier_costs_;
    //             RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Processing robot: " + robot_name, this->get_namespace()));
    //             client_get_frontier_costs_ = this->create_client<frontier_msgs::srv::GetFrontierCosts>(robot_name + "/multirobot_get_frontier_costs");
    //             auto request_frontier_costs = std::make_shared<frontier_msgs::srv::GetFrontierCosts::Request>();
    //             request_frontier_costs->requested_frontier_list = srv_res->frontier_list;
    //             request_frontier_costs->robot_namespace = robot_name;
    //             request_frontier_costs->prohibited_frontiers = blacklisted_frontiers_;
    //             while (!client_get_frontier_costs_->wait_for_service(std::chrono::seconds(1)))
    //             {
    //                 if (!rclcpp::ok())
    //                 {
    //                     RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
    //                 }
    //                 RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for get frontier costs service from " + robot_name, this->get_namespace()));
    //             }
    //             RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Got get frontier costs service from" + robot_name, this->get_namespace()));
    //             auto result_frontier_costs = client_get_frontier_costs_->async_send_request(request_frontier_costs);
    //             std::shared_ptr<frontier_msgs::srv::GetFrontierCosts_Response> frontier_costs_srv_res;
    //             if (result_frontier_costs.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
    //             {
    //                 frontier_costs_srv_res = result_frontier_costs.get();
    //                 if (!frontier_costs_srv_res)
    //                 {
    //                     RCLCPP_ERROR(this->get_logger(), "Did not recieve a response.");
    //                 }
    //                 if (frontier_costs_srv_res->success == true)
    //                 {
    //                     RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Processed: " + robot_name, this->get_namespace()));
    //                     auto frontierCosts = frontier_costs_srv_res->frontier_costs;
    //                     // for (auto dist : frontier_costs_srv_res->frontier_distances)
    //                     // {
    //                     //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Frontier distances: " + std::to_string(dist), this->get_namespace()));
    //                     // }
    //                     // for (auto dist : frontier_costs_srv_res->frontier_arrival_information)
    //                     // {
    //                     //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Frontier arrival information: " + std::to_string(dist), this->get_namespace()));
    //                     // }
    //                     RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Size of the frontier list (request) for " + robot_name + " is: " + std::to_string(request_frontier_costs->requested_frontier_list.size()), this->get_namespace()));
    //                     RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Frontier costs size (response): " + std::to_string(frontierCosts.size()), this->get_namespace()));
    //                     taskAllocator->addRobotTasks(frontierCosts, frontier_costs_srv_res->frontier_distances, robot_name);
    //                     if (globalFrontierList.empty())
    //                     {
    //                         globalFrontierList = frontier_costs_srv_res->frontier_list;
    //                     }
    //                     else
    //                     {
    //                         if (!equateFrontierList(frontier_costs_srv_res->frontier_list, globalFrontierList))
    //                         {
    //                             auto gfl_size = globalFrontierList.size();
    //                             throw std::runtime_error("The frontier lists are not same for other robot" + std::to_string(gfl_size));
    //                         }
    //                     }
    //                 }
    //                 else if (frontier_costs_srv_res->success == false)
    //                 {
    //                     RCLCPP_ERROR_STREAM(this->get_logger(), "Server returned false of robot " << robot_name << ". Probably busy.");
    //                     // add check to solve hungarian or not
    //                 }
    //             }
    //             else
    //             {
    //                 RCLCPP_ERROR(this->get_logger(), "Failed to call the frontier costs handler for another robot.");
    //             }
    //         }
    //     }
    // }

    void FrontierExplorationServer::run()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::executeCb", this->get_namespace()));
        moving_ = false;

        // Don't compute a plan until costmap is valid (after clear costmap)
        rclcpp::Rate r(100);
        while (!explore_costmap_ros_->isCurrent())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for costmap to be current", this->get_namespace()));
            r.sleep();
        }

        /** 
         * ======================= wait for nav2 and costmap services ==================
         */
        if (!nav2Client_->wait_for_action_server(std::chrono::seconds(30)))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return;
        }

        /** 
         * ======================= set region boundary on costmap =======================
         */
        if (rclcpp::ok())
        {
            auto updateBoundaryResult = bel_ptr_->updateBoundaryPolygon(explore_boundary_);
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Adding update boundary polygon for spin.", this->get_namespace()));
            if (updateBoundaryResult == true)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Region boundary set", this->get_namespace()));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for updateBoundaryPolygon called from within.");
                return;
            }
        }

        rclcpp::sleep_for(std::chrono::seconds(5));

        /** 
         * ======================= The exploration is active until this while loop runs. ======================= 
         */
        while (rclcpp::ok())
        {
            /** 
             * ======================= initialize placeholders ===================
             */
            geometry_msgs::msg::PoseStamped goal_pose;
            geometry_msgs::msg::PoseStamped robot_pose;

            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("NEW ITERATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!", this->get_namespace()));
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Blacklist size: " + std::to_string(blacklisted_frontiers_.size()), this->get_namespace()));
            
            /** 
             * ======================= initialize pointers ======================= 
             */
            std::shared_ptr<TaskAllocator> taskAllocator = std::make_shared<TaskAllocator>();
            auto nextFrontierRequestPtr = std::make_shared<GetNextFrontierRequest>();
            auto nextFrontierResultPtr = std::make_shared<GetNextFrontierResponse>();
            
            /** 
             * ========== set blacklist (previously traversed frontiers) ========= 
             */
            nextFrontierRequestPtr->prohibited_frontiers = blacklisted_frontiers_;

            /** 
             * ======================= get robot pose and set as start pose ======
             */
            if (!explore_costmap_ros_->getRobotPose(robot_pose))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get robot position from explore costmap.");
            }
            nextFrontierRequestPtr->start_pose = robot_pose;

            /** 
             * ======================= search for frontiers ======================= 
             */
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Starting computation", this->get_namespace()));
            // initialize frontier search implementation
            frontier_exploration::FrontierSearch frontierSearch(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()), min_frontier_cluster_size_, max_frontier_cluster_size_);
            // get list of frontiers from search implementation.
            geometry_msgs::msg::Point search_start_pose;
            search_start_pose = nextFrontierRequestPtr->start_pose.pose.position;
            nextFrontierRequestPtr->frontier_list = frontierSearch.searchFrom(search_start_pose);
            nextFrontierRequestPtr->every_frontier = frontierSearch.getAllFrontiers();

            // ----------------construct KD-tree-------------------------
            Kdtree::KdNodeVector nodes;
            for (int i = 0; i < nextFrontierRequestPtr->frontier_list.size(); ++i) {
                nodes.push_back(Kdtree::KdNode(nextFrontierRequestPtr->frontier_list[i]));
            }
            Kdtree::KdTree tree(&nodes);
            // ----------------Print the nodes-------------------------
            cout << "Points in kd-tree:\n  ";
            print_nodes(tree.allnodes);

            // ----------------Search nearest nodes-------------------------
            Kdtree::KdNodeVector result;
            std::vector<double> test_point(2);
            test_point[0] = 8;
            test_point[1] = 3;
            tree.k_nearest_neighbors(test_point, 3, &result);
            cout << "3NNs of (" << test_point[0] << "," << test_point[1] << "):\n  ";
            print_nodes(result);
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Clusterred frontier size: " + std::to_string(nextFrontierRequestPtr->frontier_list.size()), this->get_logger().get_name()));
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Every frontier size: " + std::to_string(nextFrontierRequestPtr->every_frontier.size()), this->get_logger().get_name()));
            bel_ptr_->visualizeFrontier(nextFrontierRequestPtr);

            test_point[0] = 8;
            test_point[1] = 3;
            tree.range_nearest_neighbors(test_point, 0.1, &result);
            cout << "Neighbors of (" << test_point[0] << "," << test_point[1] << ") with distance <= 0.1:\n  ";
            print_nodes(result);
            cout << endl << result.size() << endl;

            /** 
             * =============== get the next frontier and costs ====================
             */
            auto nextFrontierSuccess = bel_ptr_->getNextFrontier(nextFrontierRequestPtr, nextFrontierResultPtr);
            RCLCPP_INFO_STREAM(this->get_logger(), "Sent GNF request");
            if (nextFrontierSuccess == false)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive response for getNextFrontier called from within the robot.");
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Size of the frontier list being used for multirobot is: " + std::to_string(nextFrontierResultPtr->frontier_list.size()), this->get_namespace()));

            /** 
             * =============== Process all the conditions ====================
             */

            // if following true, robot left exploration boundary.
            if (explore_boundary_.polygon.points.size() > 0 && !pointInPolygon(robot_pose.pose.position, explore_boundary_.polygon))
            {
                RCLCPP_ERROR(this->get_logger(), "Robot left exploration boundary, returning to center");

                // set goal pose to exploration center
                goal_pose.header = explore_center_.header;
                goal_pose.pose.position = explore_center_.point;
                goal_pose.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(nextFrontierRequestPtr->start_pose.pose.position, explore_center_.point));
            }
            else if (nextFrontierResultPtr->success == true)
            {
                std::vector<frontier_msgs::msg::Frontier> globalFrontierList = {};

                // processAllRobots(taskAllocator, globalFrontierList, nextFrontierResultPtr);
                taskAllocator->addRobotTasks(nextFrontierResultPtr->frontier_costs, nextFrontierResultPtr->frontier_distances, this->get_namespace());
                // for (auto dist : srv_res->frontier_distances)
                // {
                //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Other Frontier distances: " + std::to_string(dist), this->get_namespace()));
                // }
                // for (auto dist : srv_res->frontier_arrival_information)
                // {
                //     RCLCPP_ERROR_STREAM(this->get_logger(), COLOR_STR("Other Frontier arrival information: " + std::to_string(dist), this->get_namespace()));
                // }
                if (globalFrontierList.empty())
                {
                    globalFrontierList = nextFrontierResultPtr->frontier_list;
                }
                else
                {
                    if (!equateFrontierList(globalFrontierList, nextFrontierResultPtr->frontier_list))
                    {
                        throw std::runtime_error("The frontier lists are not same for current robot: " + static_cast<std::string>(this->get_namespace()) + " srv_res size: " + std::to_string(nextFrontierResultPtr->frontier_list.size()) + " globalFrontier list size: " + std::to_string(globalFrontierList.size()));
                    }
                }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Solving hungarian", this->get_namespace()));
                taskAllocator->solveAllocationHungarian();
                // taskAllocator->solveAllocationMinPos();
                auto allocatedIndex = taskAllocator->getAllocatedTasks()[this->get_namespace()];
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated index" + std::to_string(allocatedIndex), this->get_namespace()));
                auto allocatedFrontier = globalFrontierList[allocatedIndex];
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier x:" + std::to_string(allocatedFrontier.goal_point.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier y:" + std::to_string(allocatedFrontier.goal_point.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier z:" + std::to_string(allocatedFrontier.goal_point.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier oz:" + std::to_string(allocatedFrontier.best_orientation.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier ow:" + std::to_string(allocatedFrontier.best_orientation.w), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier ox:" + std::to_string(allocatedFrontier.best_orientation.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier oy:" + std::to_string(allocatedFrontier.best_orientation.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Allocated frontier uid:" + std::to_string(allocatedFrontier.unique_id), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier x:" + std::to_string(nextFrontierResultPtr->next_frontier.goal_point.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier y:" + std::to_string(nextFrontierResultPtr->next_frontier.goal_point.y), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier z:" + std::to_string(nextFrontierResultPtr->next_frontier.goal_point.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier oz:" + std::to_string(nextFrontierResultPtr->next_frontier.best_orientation.z), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier ow:" + std::to_string(nextFrontierResultPtr->next_frontier.best_orientation.w), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier ox:" + std::to_string(nextFrontierResultPtr->next_frontier.best_orientation.x), this->get_namespace()));
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Selected frontier oy:" + std::to_string(nextFrontierResultPtr->next_frontier.best_orientation.y), this->get_namespace()));
                // if(allocatedFrontier.best_orientation != srv_res->next_frontier.pose.orientation)
                // {
                // throw std::runtime_error("The orientations are not same. What are you doing?");
                // }
                // if(allocatedFrontier.goal_point != srv_res->next_frontier.pose.position)
                // {
                // throw std::runtime_error("The initial positions are not same. What are you doing?");
                // }
                RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Next frontier Result success. Sending goal to Nav2", this->get_namespace()));
                if(allocatedFrontier.goal_point.x == 0 && allocatedFrontier.goal_point.y == 0 && allocatedFrontier.size == 0)
                    throw std::runtime_error("Sanity check detected. Frontier allocated without positive size");
                if (use_pose_from_multirobot_allocator_)
                {
                    RCLCPP_ERROR(this->get_logger(), "USING MULTIROBOT FRONTIER");
                    goal_pose.pose.position = allocatedFrontier.goal_point;
                    goal_pose.pose.orientation = allocatedFrontier.best_orientation;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = rclcpp::Clock().now();
                    RCLCPP_ERROR_STREAM(this->get_logger(), "UID: " << allocatedFrontier.unique_id);
                    if (std::find(blacklisted_frontiers_.begin(), blacklisted_frontiers_.end(), allocatedFrontier) == blacklisted_frontiers_.end())
                    {
                        blacklisted_frontiers_.push_back(allocatedFrontier);
                    }
                }
                else
                {
                    goal_pose.pose.position = allocatedFrontier.goal_point;
                    goal_pose.pose.orientation = nextFrontierResultPtr->next_frontier.best_orientation;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = rclcpp::Clock().now();
                    RCLCPP_ERROR_STREAM(this->get_logger(), "UID: " << allocatedFrontier.unique_id);
                    if (std::find(blacklisted_frontiers_.begin(), blacklisted_frontiers_.end(), allocatedFrontier) == blacklisted_frontiers_.end())
                    {
                        blacklisted_frontiers_.push_back(nextFrontierResultPtr->next_frontier);
                    }
                }
            }
            else if (nextFrontierResultPtr->success == false)
            {
                // if no frontier found, check if search is successful
                RCLCPP_ERROR_STREAM(this->get_logger(), "Next frontier Result failure.");

                // search is succesful
                if (retry_ == 0)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Finished exploring room", this->get_namespace()));
                    std::unique_lock<std::mutex> lock(nav2Clientlock_);
                    nav2Client_->async_cancel_all_goals();
                    return;
                }
                else if (retry_ == 0 || !rclcpp::ok())
                {
                    // search is not successful
                    RCLCPP_ERROR(this->get_logger(), "Failed exploration");
                    return;
                }

                RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Retrying...", this->get_namespace()));
                retry_--;
                // try to find frontier again
                if (retry_ <= this->get_parameter("retry_count").as_int())
                {
                    FrontierExplorationServer::performBackupRotation();
                    FrontierExplorationServer::performBackupReverse();
                    rclcpp::Rate(2).sleep();
                }
                continue;
            }
            // if above conditional does not escape this loop step, search has a valid goal_pose

            if (!moving_ && nextFrontierResultPtr->success == true)
            {
                nav2_goal_lock_.lock();
                nav2_goal_ = std::make_shared<NavigateToPose::Goal>();
                nav2_goal_->pose = goal_pose;
                if (use_custom_sim_)
                    nav2_goal_->behavior_tree = get_namespace();
                nav2Client_->async_send_goal(*nav2_goal_, nav2_goal_options_);
                nav2_goal_lock_.unlock();
                moving_ = true;
                int waitCount_ = 0;
                while (moving_ && rclcpp::ok())
                {
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                    waitCount_++;
                    RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Wait count goal is: " + std::to_string(waitCount_), this->get_namespace()));
                    // get the units of waitTime to match waitCount_
                    if (waitCount_ > nav2WaitTime_ * 10)
                    {
                        nav2Client_->async_cancel_all_goals();
                        FrontierExplorationServer::performBackupRotation();
                        FrontierExplorationServer::performBackupReverse();
                        break;
                    }
                }
                // Moving is made false here to treat this as equal to nav2 feedback saying goal is reached (nav2 timed out here.).
                moving_ = false;
            }
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
        // turn for half a second
        for (int i = 0; i < 10; i++)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            publisher->publish(twist_msg);
        }
        twist_msg.angular.z = 0.0;
        // publish enough to stop the robot.
        for (int j = 0; j <= 30; j++)
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
        // move back for two seconds.
        for (int i = 0; i <= 40; i++)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            publisher->publish(twist_msg);
        }
        twist_msg.linear.x = 0.0;
        // publish enough to stop the robot.
        for (int j = 0; j <= 30; j++)
            publisher->publish(twist_msg);
        // Log the action
        RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("FrontierExplorationServer::performBackupReverse", this->get_namespace()));
        moving_ = false;
    }

    void FrontierExplorationServer::handle_multirobot_current_goal_request(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<frontier_msgs::srv::GetCurrentGoal::Request> request,
        std::shared_ptr<frontier_msgs::srv::GetCurrentGoal::Response> response)
    {
        if(nav2_goal_ == nullptr)
        {
            response->goal_active = false;
        }
        else
        {
            nav2_goal_lock_.lock();
            response->current_goal = nav2_goal_->pose;
            nav2_goal_lock_.unlock();
            response->goal_active = true;
        }
    }

    // void FrontierExplorationServer::handle_multirobot_frontier_cost_request(
    //     std::shared_ptr<rmw_request_id_t> request_header,
    //     std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Request> request,
    //     std::shared_ptr<frontier_msgs::srv::GetFrontierCosts::Response> response)
    // {
    //     if (!layer_configured_)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Frontier costs handle called but not configured so exiting...");
    //         response->success = false;
    //         return;
    //     }
    //     RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Multirobot frontier costs handle called with cp as: " + std::to_string(currently_processing_), this->get_namespace()));
    //     if (wait_for_other_robot_costs_)
    //     {
    //         while (currently_processing_ == true)
    //         {
    //             RCLCPP_WARN_STREAM(this->get_logger(), COLOR_STR("Waiting for currently processing to become false", this->get_namespace()));
    //             rclcpp::sleep_for(std::chrono::milliseconds(100));
    //         }
    //     }
    //     else
    //     {
    //         if (currently_processing_ == true)
    //         {
    //             response->success = false;
    //             return;
    //         }
    //     }
    //     auto srv_req = std::make_shared<frontier_msgs::srv::GetNextFrontier::Request>();
    //     auto srv_res = std::make_shared<frontier_msgs::srv::GetNextFrontier::Response>();

    //     // get current robot pose in frame of exploration boundary
    //     geometry_msgs::msg::PoseStamped robot_pose;
    //     explore_costmap_ros_->getRobotPose(robot_pose);

    //     srv_req->start_pose = robot_pose;
    //     srv_req->override_frontier_list = true;
    //     srv_req->frontier_list_to_override = request->requested_frontier_list;
    //     srv_req->prohibited_frontiers = request->prohibited_frontiers;

    //     // evaluate if robot is within exploration boundary using robot_pose in boundary frame
    //     currently_processing_lock_.lock();
    //     currently_processing_ = true;
    //     currently_processing_lock_.unlock();
    //     RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Set Currently processing to true: " + currently_processing_, this->get_namespace()));
    //     while (!getNextFrontier->wait_for_service(std::chrono::seconds(10)))
    //     {
    //         if (!rclcpp::ok())
    //         {
    //             RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", this->get_namespace()));
    //         }
    //         RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Waiting for get next frontier service of the same robot called from another robot.", this->get_namespace()));
    //     }
    //     auto srv_future_id = getNextFrontier->async_send_request(srv_req);

    //     RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Multi robot Sent request.", this->get_namespace()));
    //     if (srv_future_id.wait_for(std::chrono::seconds(200)) == std::future_status::ready)
    //     {
    //         srv_res = srv_future_id.get();
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to receive response of getNextFrontier Called from outside.");
    //     }
    //     RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Multi robot Recieved response.", this->get_namespace()));
    //     if (srv_res->success == true)
    //     {
    //         response->success = true;
    //         response->frontier_costs = srv_res->frontier_costs;
    //         response->frontier_list = srv_res->frontier_list;
    //         response->frontier_distances = srv_res->frontier_distances;
    //         response->frontier_arrival_information = srv_res->frontier_arrival_information;
    //     }
    //     else if (srv_res->success == false)
    //     {
    //         response->success = false;
    //     }
    //     currently_processing_lock_.lock();
    //     currently_processing_ = false;
    //     currently_processing_lock_.unlock();
    // }

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
            nav2_goal_ = nullptr;
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Nav2 internal fault! Nav2 aborted the goal!");
            FrontierExplorationServer::performBackupRotation();
            FrontierExplorationServer::performBackupReverse();
            nav2_goal_ = nullptr;
            moving_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO_STREAM(this->get_logger(), COLOR_STR("Nav2 goal canceled successfully", this->get_namespace()));
            nav2_goal_ = nullptr;
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

    bool FrontierExplorationServer::equateFrontierList(const std::vector<frontier_msgs::msg::Frontier> &list1, const std::vector<frontier_msgs::msg::Frontier> &list2)
    {
        bool listflag = true;
        // Check if the lists have the same size
        if (list1.size() != list2.size())
        {
            return false;
        }

        // Compare each corresponding element in both lists
        for (size_t i = 0; i < list1.size(); ++i)
        {
            auto f1 = list1[i];
            auto f2 = list2[i];
            if (!equateFrontiers(f1, f2, true))
            {
                listflag = false;
            }
        }
        return listflag;
    }
};