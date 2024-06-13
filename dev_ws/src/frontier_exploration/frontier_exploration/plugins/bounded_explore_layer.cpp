#include <frontier_exploration/bounded_explore_layer.hpp>
#include <frontier_exploration/frontier_search.hpp>

namespace frontier_exploration
{
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;
    using rcl_interfaces::msg::ParameterType;

    BoundedExploreLayer::BoundedExploreLayer(nav2_costmap_2d::LayeredCostmap* costmap) 
    {
        layered_costmap_ = costmap;
        internal_node_ = rclcpp::Node::make_shared("bounded_explore_layer");
        internal_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        internal_executor_->add_node(internal_node_);

        std::thread t1([this]() {
            internal_executor_->spin();
        });
        t1.detach();

        internal_node_->declare_parameter("exploration_mode", rclcpp::ParameterValue(std::string("ours")));
        internal_node_->get_parameter("exploration_mode", exploration_mode_);

        frontierSelect_ = std::make_shared<frontier_exploration::FrontierSelectionNode>(internal_node_, layered_costmap_->getCostmap());

        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        dyn_params_handler_ = internal_node_->add_on_set_parameters_callback(
            std::bind(
                &BoundedExploreLayer::dynamicParametersCallback,
                this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(internal_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::onInitialize", internal_node_->get_logger().get_name()));

        client_get_map_data2_ = internal_node_->create_client<slam_msgs::srv::GetMap>("orb_slam3_get_map_data");
    }

    BoundedExploreLayer::~BoundedExploreLayer()
    {
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::~BoundedExploreLayer()", internal_node_->get_logger().get_name()));
        delete layered_costmap_;
        dyn_params_handler_.reset();
        tf_buffer_.reset();
        rclcpp::shutdown();
    }

    rcl_interfaces::msg::SetParametersResult BoundedExploreLayer::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters)
        {
            const auto &param_type = parameter.get_type();
            const auto &param_name = parameter.get_name();

            if (param_type == ParameterType::PARAMETER_BOOL)
            {
                if (param_name == "explore_clear_space")
                {
                    explore_clear_space_ = parameter.as_bool();
                }
                else if (param_name == "enabled")
                {
                    enabledLayer_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }

    SelectionResult BoundedExploreLayer::processOurApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list, geometry_msgs::msg::Point& start_point_w)
    {
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::processOurApproach", internal_node_->get_logger().get_name()));
        auto startTime = std::chrono::high_resolution_clock::now();

        // Getting map data
        // Create a service request
        auto request_map_data = std::make_shared<slam_msgs::srv::GetMap::Request>();
        auto response_map_data = std::make_shared<slam_msgs::srv::GetMap::Response>();
        /** Uncomment this if you want to use map data.
        request_map_data->tracked_points = true;
        // Adding landmarks from 1 to 100 to the vector
        std::vector<int32_t> kf_id_for_landmarks;
        for (int i = 0; i <= 10000; ++i)
        {
            kf_id_for_landmarks.push_back(i);
        }
        request_map_data->kf_id_for_landmarks = kf_id_for_landmarks;
        while (!client_get_map_data2_->wait_for_service(std::chrono::seconds(1))) {
            if(!rclcpp::ok()) {
                RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("ROS shutdown request in between waiting for service.", internal_node_->get_logger().get_name()));
            }
            RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Waiting for get map data service", internal_node_->get_logger().get_name()));
        }
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Got get map data service.", internal_node_->get_logger().get_name()));
        auto result_map_data = client_get_map_data2_->async_send_request(request_map_data);
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer -- Start map data.", internal_node_->get_logger().get_name()));
        if (result_map_data.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
        {
            response_map_data = result_map_data.get();
            if (response_map_data->data.nodes.empty())
            {
                RCLCPP_WARN(internal_node_->get_logger(), "No map data recieved");
            }
            else
            {
                // Process the received poses as needed
                RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Map data has " << response_map_data->data.graph.poses.size() << " poses"));
                RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Map data has " << response_map_data->data.graph.poses_id.size() << " pose ids"));
                RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Map data has " << response_map_data->data.nodes.size() << " keyframes"));
            }
        }
        else
        {
            RCLCPP_ERROR(internal_node_->get_logger(), "Failed to call the service map_data.");
        }
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer -- End map data.", internal_node_->get_logger().get_name()));
        */
        // Select the frontier
        SelectionResult selection_result;
        selection_result = frontierSelect_->selectFrontierOurs(frontier_list, polygon_xy_min_max_, start_point_w, response_map_data, layered_costmap_->getCostmap());
        if (selection_result.success == false)
        {
            RCLCPP_ERROR(internal_node_->get_logger(), "The selection result for our method is false!");
            return selection_result;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = (endTime - startTime).count() / 1.0;
        // RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Time taken to plan to all frontiers is: " + duration, internal_node_->get_logger().get_name()));
        return selection_result;
    }

    std::pair<frontier_msgs::msg::Frontier, bool> BoundedExploreLayer::processGreedyApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        auto selection_result = frontierSelect_->selectFrontierClosest(frontier_list);
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::processGreedyApproach", internal_node_->get_logger().get_name()));
        return selection_result;
    }

    std::pair<frontier_msgs::msg::Frontier, bool> BoundedExploreLayer::processRandomApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::processRandomApproach", internal_node_->get_logger().get_name()));
        auto selection_result = frontierSelect_->selectFrontierRandom(frontier_list);
        return selection_result;
    }

    void BoundedExploreLayer::visualizeFrontier(std::shared_ptr<GetNextFrontierRequest> requestData)
    {
        // Visualize the frontiers only if frontier list is not overriden.
        frontierSelect_->visualizeFrontier(requestData->frontier_list, requestData->every_frontier, layered_costmap_->getGlobalFrameID());
        frontierSelect_->exportMapCoverage(polygon_xy_min_max_, startTime_);
    }

    bool BoundedExploreLayer::getNextFrontier(std::shared_ptr<GetNextFrontierRequest> requestData, std::shared_ptr<GetNextFrontierResponse> resultData)
    {
        frontierSelect_->setFrontierBlacklist(requestData->prohibited_frontiers);
        // Select the frontier (Modify this for different algorithms)
        frontier_msgs::msg::Frontier selected;
        if (exploration_mode_ == "greedy")
        {
            throw std::runtime_error("Not supported for multirobot");
            auto selection_result = BoundedExploreLayer::processGreedyApproach(requestData->frontier_list);
            if (selection_result.second == false)
            {
                resultData->success = false;
                return resultData->success;
            }
            resultData->success = true;
            // resultData->next_frontier = selection_result.first;
        }

        else if (exploration_mode_ == "random")
        {
            throw std::runtime_error("Not supported for multirobot");
            auto selection_result = BoundedExploreLayer::processRandomApproach(requestData->frontier_list);
            if (selection_result.second == false)
            {
                resultData->success = false;
                return resultData->success;
            }
            resultData->success = true;
            // resultData->next_frontier = selection_result.first;
        }

        else if (exploration_mode_ == "ours")
        {
            geometry_msgs::msg::Point start_point_w;
            start_point_w.x = requestData->start_pose.pose.position.x;
            start_point_w.y = requestData->start_pose.pose.position.y;
            auto selection_result = BoundedExploreLayer::processOurApproach(requestData->frontier_list, start_point_w);
            if (selection_result.success == false)
            {
                resultData->success = false;
                return resultData->success;
            }
            resultData->success = true;
            std::vector<frontier_msgs::msg::Frontier> frontiers_list;
            std::vector<double> frontier_costs;
            std::vector<double> frontier_distances;
            std::vector<double> frontier_arrival_information;
            std::vector<double> frontier_path_information;
            RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Selection result's frontier costs size: " + std::to_string(selection_result.frontier_costs.size()), internal_node_->get_logger().get_name()));
            for (auto& frontier : requestData->frontier_list)
            {
                if(selection_result.frontier_costs.count(frontier) != 1)
                    throw std::runtime_error("Frontier not found");
                frontier.best_orientation = nav2_util::geometry_utils::orientationAroundZAxis(selection_result.frontier_costs[frontier].theta_s_star_);
                frontiers_list.push_back(frontier);
                frontier_costs.push_back(selection_result.frontier_costs[frontier].cost_);
                frontier_distances.push_back(selection_result.frontier_costs[frontier].path_length_);
                frontier_arrival_information.push_back(selection_result.frontier_costs[frontier].information_);
            }
            RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Making list", internal_node_->get_logger().get_name()));
            resultData->frontier_list = frontiers_list;
            resultData->frontier_costs = frontier_costs;
            resultData->frontier_distances = frontier_distances;
            resultData->frontier_arrival_information = frontier_arrival_information;
            if(resultData->frontier_list != requestData->frontier_list)
            {
                throw std::runtime_error("Lists are not SAME!");
            }
        }

        else
        {
            RCLCPP_ERROR(internal_node_->get_logger(), "Invalid mode of exploration");
            rclcpp::shutdown();
        }
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Res frontier costs size: " + std::to_string(resultData->frontier_costs.size()), internal_node_->get_logger().get_name()));
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Res frontier list size: " + std::to_string(resultData->frontier_list.size()), internal_node_->get_logger().get_name()));
        return resultData->success;
    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped& explore_boundary)
    {
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::updateBoundaryPolygonService", internal_node_->get_logger().get_name()));

        // Transform all points of boundary polygon into costmap frame
        geometry_msgs::msg::PointStamped in;
        in.header = explore_boundary.header;
        for (const auto &point32 : explore_boundary.polygon.points)
        {
            in.point = nav2_costmap_2d::toPoint(point32);
            polygon_.points.push_back(nav2_costmap_2d::toPoint32(in.point));
        }

        // if empty boundary provided, set to whole map
        if (polygon_.points.empty())
        {
            geometry_msgs::msg::Point32 temp;
            temp.x = layered_costmap_->getCostmap()->getOriginX();
            temp.y = layered_costmap_->getCostmap()->getOriginY();
            polygon_.points.push_back(temp);
            temp.y = layered_costmap_->getCostmap()->getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = layered_costmap_->getCostmap()->getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = layered_costmap_->getCostmap()->getOriginY();
            polygon_.points.push_back(temp);
        }

        // Find map size and origin by finding min/max points of polygon
        double min_x_polygon = std::numeric_limits<double>::infinity();
        double min_y_polygon = std::numeric_limits<double>::infinity();
        double max_x_polygon = -std::numeric_limits<double>::infinity(); // observe the minus here
        double max_y_polygon = -std::numeric_limits<double>::infinity(); // observe the minus here

        for (const auto &point : polygon_.points)
        {
            min_x_polygon = std::min(min_x_polygon, (double)point.x);
            min_y_polygon = std::min(min_y_polygon, (double)point.y);
            max_x_polygon = std::max(max_x_polygon, (double)point.x);
            max_y_polygon = std::max(max_y_polygon, (double)point.y);
        }

        polygon_xy_min_max_.push_back(min_x_polygon);
        polygon_xy_min_max_.push_back(min_y_polygon);
        polygon_xy_min_max_.push_back(max_x_polygon);
        polygon_xy_min_max_.push_back(max_y_polygon);
        return true;
    }
}