#include <frontier_exploration/geometry_tools.hpp>
#include <frontier_exploration/bounded_explore_layer.hpp>
#include <frontier_exploration/frontier_search.hpp>

namespace frontier_exploration
{
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;
    using rcl_interfaces::msg::ParameterType;

    BoundedExploreLayer::BoundedExploreLayer() {}

    BoundedExploreLayer::~BoundedExploreLayer()
    {
        polygonService_.reset();
        frontierService_.reset();
        dyn_params_handler_.reset();
        tf_buffer_.reset();
        rclcpp::shutdown();
    }

    void BoundedExploreLayer::onInitialize()
    {
        configured_ = false;
        marked_ = false;
        current_ = true;

        declareParameter("explore_clear_space", rclcpp::ParameterValue(true));
        declareParameter("resize_to_boundary", rclcpp::ParameterValue(true));
        declareParameter("frontier_travel_point", rclcpp::ParameterValue(std::string("closest")));
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("min_frontier_cluster_size", rclcpp::ParameterValue(1));
        declareParameter("max_frontier_cluster_size", rclcpp::ParameterValue(20));
        declareParameter("exploration_mode", rclcpp::ParameterValue(std::string("ours")));

        node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node bounded_explore_layer"};
        }

        frontierSelect_ = std::make_shared<frontier_exploration::FrontierSelectionNode>(node, layered_costmap_->getCostmap());
        node->get_parameter(name_ + "." + "explore_clear_space", explore_clear_space_);
        node->get_parameter(name_ + "." + "resize_to_boundary", resize_to_boundary_);
        node->get_parameter(name_ + "." + "frontier_travel_point", frontier_travel_point_);
        node->get_parameter(name_ + "." + "enabled", enabledLayer_);
        node->get_parameter(name_ + "." + "min_frontier_cluster_size", min_frontier_cluster_size_);
        node->get_parameter(name_ + "." + "max_frontier_cluster_size", max_frontier_cluster_size_);
        node->get_parameter(name_ + "." + "exploration_mode", exploration_mode_);
        if (explore_clear_space_)
        {
            default_value_ = NO_INFORMATION;
        }
        else
        {
            default_value_ = FREE_SPACE;
        }

        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        polygonService_ = node->create_service<frontier_msgs::srv::UpdateBoundaryPolygon>(
            "update_boundary_polygon",
            std::bind(&BoundedExploreLayer::updateBoundaryPolygonService, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        frontierService_ = node->create_service<frontier_msgs::srv::GetNextFrontier>(
            "get_next_frontier",
            std::bind(&BoundedExploreLayer::getNextFrontierService, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(
                &BoundedExploreLayer::dynamicParametersCallback,
                this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        matchSize();
        RCLCPP_INFO(logger_, "BoundedExploreLayer::onInitialize");

        internal_node_ = rclcpp::Node::make_shared("layer_node_bel");
        internal_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        internal_executor_->add_node(internal_node_);
        client_get_map_data2_ = internal_node_->create_client<slam_msgs::srv::GetMap>("orb_slam3_get_map_data");

        std::thread t1([this]() {
            internal_executor_->spin();
        });
        t1.detach();
    }

    void BoundedExploreLayer::matchSize()
    {
        RCLCPP_INFO_STREAM(logger_, "BoundedExploreLayer::matchSize");
        Costmap2D *master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }

    rcl_interfaces::msg::SetParametersResult BoundedExploreLayer::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
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
                else if (param_name == "resize_to_boundary")
                {
                    resize_to_boundary_ = parameter.as_bool();
                }
                else if (param_name == "enabled")
                {
                    enabledLayer_ = parameter.as_bool();
                }
            }

            else if (param_type == ParameterType::PARAMETER_STRING)
            {
                if (param_name == "frontier_travel_point")
                {
                    frontier_travel_point_ = parameter.as_string();
                }
            }
            else if (param_type == ParameterType::PARAMETER_INTEGER)
            {
                if (param_name == "min_frontier_cluster_size")
                {
                    min_frontier_cluster_size_ = parameter.as_int();
                }
            }
        }

        result.successful = true;
        return result;
    }

    SelectionResult BoundedExploreLayer::processOurApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list, geometry_msgs::msg::Point& start_point_w)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processOurApproach");
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
                RCLCPP_INFO(logger_, "ROS shutdown request in between waiting for service.");
            }
            RCLCPP_INFO(logger_, "Waiting for get map data service");
        }
        RCLCPP_INFO(logger_, "Got get map data service.");
        auto result_map_data = client_get_map_data2_->async_send_request(request_map_data);
        RCLCPP_INFO(logger_, "BoundedExploreLayer -- Start map data.");
        if (result_map_data.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
        {
            response_map_data = result_map_data.get();
            if (response_map_data->data.nodes.empty())
            {
                RCLCPP_WARN(logger_, "No map data recieved");
            }
            else
            {
                // Process the received poses as needed
                RCLCPP_WARN_STREAM(logger_, "Map data has " << response_map_data->data.graph.poses.size() << " poses");
                RCLCPP_WARN_STREAM(logger_, "Map data has " << response_map_data->data.graph.poses_id.size() << " pose ids");
                RCLCPP_WARN_STREAM(logger_, "Map data has " << response_map_data->data.nodes.size() << " keyframes");
            }
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to call the service map_data.");
        }
        RCLCPP_INFO(logger_, "BoundedExploreLayer -- End map data.");
        */
        // Select the frontier
        SelectionResult selection_result;
        selection_result = frontierSelect_->selectFrontierOurs(frontier_list, polygon_xy_min_max_, start_point_w, response_map_data, layered_costmap_->getCostmap());
        if (selection_result.success == false)
        {
            RCLCPP_ERROR(logger_, "The selection result for our method is false!");
            return selection_result;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = (endTime - startTime).count() / 1.0;
        RCLCPP_INFO_STREAM(logger_, "Time taken to plan to all frontiers is: " << duration);
        return selection_result;
    }

    std::pair<frontier_msgs::msg::Frontier, bool> BoundedExploreLayer::processGreedyApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        auto selection_result = frontierSelect_->selectFrontierClosest(frontier_list);
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processGreedyApproach");
        return selection_result;
    }

    std::pair<frontier_msgs::msg::Frontier, bool> BoundedExploreLayer::processRandomApproach(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processRandomApproach");
        auto selection_result = frontierSelect_->selectFrontierRandom(frontier_list);
        return selection_result;
    }

    void BoundedExploreLayer::getNextFrontierService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res)
    {
        // wait for costmap to get marked with boundary
        rclcpp::Rate r(10);
        while (!marked_)
        {
            r.sleep();
        }

        if (req->start_pose.header.frame_id != layered_costmap_->getGlobalFrameID())
        {
            // error out if no transform available
            std::string tf_error;
            if (!tf_buffer_->canTransform(layered_costmap_->getGlobalFrameID(), req->start_pose.header.frame_id, tf2::TimePointZero, &tf_error))
            {
                RCLCPP_ERROR_STREAM(logger_, "Couldn't transform from map "
                                                 << " to " << req->start_pose.header.frame_id.c_str());
                res->success = false;
            }
            geometry_msgs::msg::PoseStamped temp_pose = req->start_pose;
            tf_buffer_->transform(temp_pose, req->start_pose, layered_costmap_->getGlobalFrameID());
        }

        std::vector<frontier_msgs::msg::Frontier> frontier_list;
        std::vector<std::vector<double>> every_frontier;
        if (req->override_frontier_list == false)
        {
            RCLCPP_INFO(logger_, "BoundedExploreLayer::getNextFrontierService");
            RCLCPP_INFO(logger_, "Get next frontier called from within.");
            // initialize frontier search implementation
            frontier_exploration::FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_cluster_size_, max_frontier_cluster_size_);
            // get list of frontiers from search implementation
            //  Initialize zero point to start search.
            geometry_msgs::msg::Point search_start_pose;
            search_start_pose = req->start_pose.pose.position;
            frontier_list = frontierSearch.searchFrom(search_start_pose);
            every_frontier = frontierSearch.getAllFrontiers();
            RCLCPP_WARN_STREAM(logger_, "Clusterred frontier size: " << frontier_list.size());

            // Visualize the frontiers only if frontier list is not overriden.
            frontierSelect_->visualizeFrontier(frontier_list, every_frontier, layered_costmap_->getGlobalFrameID());
        }
        if (req->override_frontier_list == true)
        {
            frontier_list = req->frontier_list_to_override;
            RCLCPP_INFO(logger_, "BoundedExploreLayer::getNextFrontierService another robot");
        }

        // Select the frontier (Modify this for different algorithms)
        frontierSelect_->exportMapCoverage(polygon_xy_min_max_, startTime_);
        frontier_msgs::msg::Frontier selected;
        if (exploration_mode_ == "greedy")
        {
            auto selection_result = BoundedExploreLayer::processGreedyApproach(frontier_list);
            if (selection_result.second == false)
            {
                res->success = false;
                return;
            }
            res->success = true;
            selected = selection_result.first;
            // set orientation here. The position is set later in the code based on ```frontier_travel_point_```
            res->next_frontier.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(req->start_pose.pose.position, res->next_frontier.pose.position));
        }

        else if (exploration_mode_ == "random")
        {
            auto selection_result = BoundedExploreLayer::processRandomApproach(frontier_list);
            if (selection_result.second == false)
            {
                res->success = false;
                return;
            }
            res->success = true;
            selected = selection_result.first;
            // set orientation here. The position is set later in the code based on ```frontier_travel_point_```
            res->next_frontier.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(req->start_pose.pose.position, res->next_frontier.pose.position));
        }

        else if (exploration_mode_ == "ours")
        {
            geometry_msgs::msg::Point start_point_w;
            start_point_w.x = req->start_pose.pose.position.x;
            start_point_w.y = req->start_pose.pose.position.y;
            auto selection_result = BoundedExploreLayer::processOurApproach(frontier_list, start_point_w);
            if (selection_result.success == false)
            {
                res->success = false;
                return;
            }
            res->success = true;
            selected = selection_result.frontier;
            // Uncomment the next line if you are using information acquired after reaching. The pose is important in that case.
            res->next_frontier.pose.orientation = selection_result.orientation;
            std::vector<frontier_msgs::msg::Frontier> frontiers_list;
            std::vector<double> frontier_costs;
            RCLCPP_WARN_STREAM(logger_, "Selection result's frontier costs size: " << selection_result.frontier_costs.size());
            std::vector<std::pair<frontier_exploration::FrontierWithMetaData, double>> frontier_costs_vector;
            for (auto pair : selection_result.frontier_costs)
            {
                frontier_costs_vector.push_back(pair);
            }
            frontier_exploration::FrontierU1ComparatorUnique frontier_u1_comp;
            std::sort(frontier_costs_vector.begin(), frontier_costs_vector.end(), frontier_u1_comp);
            for (auto pair : frontier_costs_vector)
            {
                // Extract key and value
                auto key = pair.first.frontier_; // frontier with meta data
                auto value = pair.second;        // cost.

                // Push them into respective vectors
                frontiers_list.push_back(key);
                frontier_costs.push_back(value);
            }
            res->frontier_costs = frontier_costs;
            res->frontier_list = frontiers_list;
        }

        else
        {
            RCLCPP_ERROR(logger_, "Invalid mode of exploration");
            rclcpp::shutdown();
        }
        RCLCPP_INFO_STREAM(logger_, "Res frontier costs size: " << res->frontier_costs.size());
        RCLCPP_INFO_STREAM(logger_, "Res frontier list size: " << res->frontier_list.size());
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);

        // set goal pose to next frontier
        res->next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        res->next_frontier.header.stamp = rclcpp::Clock().now();

        if (frontier_travel_point_ == "closest")
        {
            res->next_frontier.pose.position = selected.initial;
        }
        else if (frontier_travel_point_ == "middle")
        {
            res->next_frontier.pose.position = selected.middle;
        }
        else if (frontier_travel_point_ == "centroid")
        {
            res->next_frontier.pose.position = selected.centroid;
        }
        else
        {
            RCLCPP_ERROR(logger_, "Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
            res->next_frontier.pose.position = selected.initial;
        }
    }

    void BoundedExploreLayer::updateBoundaryPolygonService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Request> req, std::shared_ptr<frontier_msgs::srv::UpdateBoundaryPolygon::Response> res)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::updateBoundaryPolygonService");
        // clear existing boundary, if any
        polygon_.points.clear();
        std::string tf_error;
        // error if no transform available between polygon and costmap
        if (!tf_buffer_->canTransform(layered_costmap_->getGlobalFrameID(), req->explore_boundary.header.frame_id, tf2::TimePointZero, &tf_error))
        {
            RCLCPP_ERROR_STREAM(logger_, "Couldn't transform from " << layered_costmap_->getGlobalFrameID().c_str() << " to " << req->explore_boundary.header.frame_id.c_str());
            res->success = false;
        }

        // Transform all points of boundary polygon into costmap frame
        geometry_msgs::msg::PointStamped in;
        geometry_msgs::msg::PointStamped out;
        in.header = req->explore_boundary.header;
        for (const auto &point32 : req->explore_boundary.polygon.points)
        {
            in.point = nav2_costmap_2d::toPoint(point32);
            tf_buffer_->transform(in, out, layered_costmap_->getGlobalFrameID());
            polygon_.points.push_back(nav2_costmap_2d::toPoint32(out.point));
        }

        // if empty boundary provided, set to whole map
        if (polygon_.points.empty())
        {
            geometry_msgs::msg::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if (resize_to_boundary_)
        {
            updateOrigin(0, 0);

            // Find map size and origin by finding min/max points of polygon
            double min_x_polygon = std::numeric_limits<double>::infinity();
            double min_y_polygon = std::numeric_limits<double>::infinity();
            double max_x_polygon = -std::numeric_limits<double>::infinity();
            double max_y_polygon = -std::numeric_limits<double>::infinity();

            for (const auto &point : polygon_.points)
            {
                min_x_polygon = std::min(min_x_polygon, (double)point.x);
                min_y_polygon = std::min(min_y_polygon, (double)point.y);
                max_x_polygon = std::max(max_x_polygon, (double)point.x);
                max_y_polygon = std::max(max_y_polygon, (double)point.y);
            }

            // resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x_polygon - min_x_polygon, max_y_polygon - min_y_polygon, size_x, size_y);
            matchSize();

            polygon_xy_min_max_.push_back(min_x_polygon);
            polygon_xy_min_max_.push_back(min_y_polygon);
            polygon_xy_min_max_.push_back(max_x_polygon);
            polygon_xy_min_max_.push_back(max_y_polygon);
        }

        configured_ = true;
        marked_ = false;
        res->success = true;
    }

    void BoundedExploreLayer::reset()
    {
        // reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
    }

    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                           double *min_y, double *max_x, double *max_y)
    {

        // check if layer is enabled and configured with a boundary
        if (!enabledLayer_ || !configured_)
        {
            return;
        }

        *min_x = -std::numeric_limits<float>::max();
        *max_x = std::numeric_limits<float>::max();
        *min_y = -std::numeric_limits<float>::max();
        *max_y = std::numeric_limits<float>::max();
    }

    void BoundedExploreLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        // check if layer is enabled and configured with a boundary
        if (!enabledLayer_ || !configured_)
        {
            return;
        }

        current_ = true;

        // draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        // TODO (suchetan): Check if cells are actually marked on the costmap.
        // circular iterator
        for (int i = 0, j = polygon_.points.size() - 1; i < polygon_.points.size(); j = i++)
        {
            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1, y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2, y_2);

            raytraceLine(marker, x_1, y_1, x_2, y_2);
        }
        // update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);
    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabledLayer_)
            return;
        if (marked_ == false)
        {
            unsigned char *master = master_grid.getCharMap();
            unsigned int span = master_grid.getSizeInCellsX();
            for (int j = min_j; j < max_j; j++)
            {
                unsigned int it = span * j + min_i;
                for (int i = min_i; i < max_i; i++)
                {
                    // only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                    if (master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it]))
                    {
                        master[it] = costmap_[it];
                    }
                    it++;
                }
            }
            marked_ = true;
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, nav2_costmap_2d::Layer)
