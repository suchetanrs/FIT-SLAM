#include <frontier_exploration/bounded_explore_layer.hpp>
#include <frontier_exploration/FrontierSearch.hpp>

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

        internal_node_->declare_parameter("counter", rclcpp::ParameterValue(11087));
        internal_node_->get_parameter("counter", counter_);

        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::onInitialize", internal_node_->get_logger().get_name()));

        client_get_map_data2_ = internal_node_->create_client<slam_msgs::srv::GetMap>("orb_slam3_get_map_data");
        rosViz_ = std::make_shared<RosVisualizer>(internal_node_, layered_costmap_->getCostmap());
        frontierSelect_ = std::make_shared<frontier_exploration::FrontierCostsManager>(internal_node_, layered_costmap_->getCostmap());
    }

    BoundedExploreLayer::~BoundedExploreLayer()
    {
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("BoundedExploreLayer::~BoundedExploreLayer()", internal_node_->get_logger().get_name()));
        delete layered_costmap_;
        rclcpp::shutdown();
    }

    bool BoundedExploreLayer::processOurApproach(std::vector<Frontier> &frontier_list, geometry_msgs::msg::Point& start_point_w)
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
        std::vector<std::vector<std::string>> costTypes;
        for (auto frontier: frontier_list)
        {
            // costTypes.push_back({"A*PlannerDistance", "ArrivalInformation"});
            costTypes.push_back({"RoadmapPlannerDistance", "ArrivalInformation"});
            // costTypes.push_back({"EuclideanDistance", "ArrivalInformation"});
            // costTypes.push_back({"RandomCosts"});
            // costTypes.push_back({});
        }
        // Select the frontier
        bool costsResult;
        costsResult = frontierSelect_->assignCosts(frontier_list, polygon_xy_min_max_, start_point_w, response_map_data, costTypes);
        if (costsResult == false)
        {
            RCLCPP_ERROR(internal_node_->get_logger(), "The selection result for our method is false!");
            return costsResult;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = (endTime - startTime).count() / 1.0;
        RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Time taken to comppute cost to all frontiers is: " + std::to_string(duration), internal_node_->get_logger().get_name()));
        return costsResult;
    }

    void BoundedExploreLayer::logMapData(std::shared_ptr<GetFrontierCostsRequest> requestData)
    {
        rosViz_->exportMapCoverage(polygon_xy_min_max_, counter_, exploration_mode_);
        rosViz_->visualizeFrontier(requestData->frontier_list, requestData->every_frontier, layered_costmap_->getGlobalFrameID());
    }

    bool BoundedExploreLayer::getFrontierCosts(std::shared_ptr<GetFrontierCostsRequest> requestData, std::shared_ptr<GetFrontierCostsResponse> resultData)
    {
        frontierSelect_->setFrontierBlacklist(requestData->prohibited_frontiers);
        // Select the frontier (Modify this for different algorithms)
        Frontier selected;

        if (exploration_mode_ == "ours")
        {
            bool costsResult = BoundedExploreLayer::processOurApproach(requestData->frontier_list, requestData->start_pose.pose.position);
            if (costsResult == false)
            {
                resultData->success = false;
                return resultData->success;
            }
            resultData->success = true;
            std::vector<Frontier> frontiers_list;
            std::vector<double> frontier_costs;
            std::vector<double> frontier_distances;
            std::vector<double> frontier_arrival_information;
            std::vector<double> frontier_path_information;
            for (auto& frontier : requestData->frontier_list)
            {
                frontiers_list.push_back(frontier);
                // RCLCPP_WARN_STREAM(internal_node_->get_logger(), COLOR_STR("Frontier cost" + std::to_string(frontier.getWeightedCost()), internal_node_->get_logger().get_name()));
                frontier_costs.push_back(frontier.getWeightedCost());
                frontier_distances.push_back(frontier.getPathLength());
                frontier_arrival_information.push_back(frontier.getArrivalInformation());
                // RCLCPP_INFO_STREAM(internal_node_->get_logger(), COLOR_STR("Cost is: " + std::to_string(resultData->frontier_costs.size()), internal_node_->get_logger().get_name()));
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