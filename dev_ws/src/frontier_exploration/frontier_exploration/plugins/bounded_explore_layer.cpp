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

        declareParameter("explore_clear_space", rclcpp::ParameterValue(true));
        declareParameter("resize_to_boundary", rclcpp::ParameterValue(true));
        declareParameter("frontier_travel_point", rclcpp::ParameterValue(std::string("closest")));
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("min_frontier_cluster_size", rclcpp::ParameterValue(1));
        declareParameter("exploration_mode", rclcpp::ParameterValue(std::string("ours")));
        robot_namespaces_ = {"/scout_2", "/scout_1"};
        declareParameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));

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
        node->get_parameter(name_ + "." + "exploration_mode", exploration_mode_);
        node->get_parameter(name_ + "." + "robot_namespaces", robot_namespaces_);
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

        matchSize();
        RCLCPP_INFO(logger_, "BoundedExploreLayer::onInitialize");

        client_node_ = rclcpp::Node::make_shared("client_node_bel");
        client_get_map_data2_ = client_node_->create_client<rtabmap_msgs::srv::GetMap2>("get_map_data2");

        current_robot_namespace_ = node->get_namespace();
        current_robot_namespace_ = current_robot_namespace_.substr(0, current_robot_namespace_.find('/', current_robot_namespace_.find('/') + 1));
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

    void BoundedExploreLayer::processOurApproach(frontier_msgs::msg::Frontier &selected, std::vector<frontier_msgs::msg::Frontier> &frontier_list, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processOurApproach");
        geometry_msgs::msg::Point start_point_w;
        start_point_w.x = req->start_pose.pose.position.x;
        start_point_w.y = req->start_pose.pose.position.y;
        auto startTime = std::chrono::high_resolution_clock::now();

        // Getting map data
        // Create a service request
        auto request_map_data = std::make_shared<rtabmap_msgs::srv::GetMap2::Request>();
        request_map_data->global_map = true;
        request_map_data->optimized = true;
        request_map_data->with_images = true;
        request_map_data->with_scans = true;
        request_map_data->with_user_data = true;
        request_map_data->with_grids = true;
        request_map_data->with_words = true;
        request_map_data->with_global_descriptors = true;
        auto result_map_data = client_get_map_data2_->async_send_request(request_map_data);
        std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data_srv_res;
        RCLCPP_INFO(logger_, "BoundedExploreLayer -- Start map data.");
        if (rclcpp::spin_until_future_complete(client_node_, result_map_data, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            map_data_srv_res = result_map_data.get();
            if (map_data_srv_res->data.nodes.empty())
            {
                RCLCPP_DEBUG(logger_, "No map data recieved");
            }
            else
            {
                // Process the received poses as needed
                RCLCPP_INFO(logger_, "Received %zu map poses.", map_data_srv_res->data.nodes.size());
            }
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to call the service map_data.");
        }
        RCLCPP_INFO(logger_, "BoundedExploreLayer -- End map data.");
        // Select the frontier
        SelectionResult selection_result;
        selection_result = frontierSelect_->selectFrontierOurs(frontier_list, polygon_xy_min_max_, res, start_point_w, map_data_srv_res, layered_costmap_->getCostmap());
        if (selection_result.success == false)
        {
            RCLCPP_ERROR(logger_, "The selection result for Count Unknowns is false!");
            return;
        }
        selected = selection_result.frontier;
        // Uncomment the next line if you are using information acquired after reaching. The pose is important in that case.
        res->next_frontier.pose.orientation = selection_result.orientation;

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = (endTime - startTime).count() / 1.0;
        RCLCPP_INFO_STREAM(logger_, "Time taken to plan to all frontiers is: " << duration);

        std::vector<frontier_msgs::msg::Frontier> frontiers_list;
        std::vector<double> frontier_costs;
        for (auto pair : selection_result.frontier_costs)
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

    void BoundedExploreLayer::processGreedyApproach(frontier_msgs::msg::Frontier &selected, std::vector<frontier_msgs::msg::Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res)
    {
        auto selection_result = frontierSelect_->selectFrontierClosest(frontier_list, every_frontier, res, layered_costmap_->getGlobalFrameID());
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processGreedyApproach");
        if (selection_result.second == false)
        {
            return;
        }
        selected = selection_result.first;
        res->next_frontier.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(req->start_pose.pose.position, res->next_frontier.pose.position));
    }

    void BoundedExploreLayer::processRandomApproach(frontier_msgs::msg::Frontier &selected, std::vector<frontier_msgs::msg::Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::processRandomApproach");
        auto selection_result = frontierSelect_->selectFrontierRandom(frontier_list, every_frontier, res, layered_costmap_->getGlobalFrameID());
        if (selection_result.second == false)
        {
            return;
        }
        selected = selection_result.first;
        res->next_frontier.pose.orientation = createQuaternionMsgFromYaw(yawOfVector(req->start_pose.pose.position, res->next_frontier.pose.position));
    }

    void BoundedExploreLayer::getNextFrontierService(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Request> req, std::shared_ptr<frontier_msgs::srv::GetNextFrontier::Response> res)
    {
        RCLCPP_INFO(logger_, "BoundedExploreLayer::getNextFrontierService");
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
            tf_buffer_->transform(temp_pose, req->start_pose, layered_costmap_->getGlobalFrameID(), tf2::durationFromSec(10));
        }

        std::vector<frontier_msgs::msg::Frontier> frontier_list;
        std::vector<std::vector<double>> every_frontier;
        if (req->override_frontier_list == false)
        {
            RCLCPP_INFO(logger_, "Get next frontier called from within.");
            // initialize frontier search implementation
            frontier_exploration::FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_cluster_size_);
            // get list of frontiers from search implementation
            //  Initialize zero point to start search.
            geometry_msgs::msg::Point search_start_pose;
            frontier_list = frontierSearch.searchFrom(search_start_pose);
            every_frontier = frontierSearch.getAllFrontiers();
            RCLCPP_DEBUG_STREAM(logger_, "Clusterred frontier size: " << frontier_list.size());

            // process for all robots
            for (auto robot_name : robot_namespaces_)
            {
                RCLCPP_INFO_STREAM(logger_, "Picked: " << robot_name << " from " << current_robot_namespace_);
                if (robot_name != current_robot_namespace_)
                {
                    RCLCPP_INFO_STREAM(logger_, "Processing: " << robot_name);
                    client_get_frontier_costs_ = client_node_->create_client<frontier_msgs::srv::GetFrontierCosts>(robot_name + "/multirobot_get_frontier_costs");
                    auto request_frontier_costs = std::make_shared<frontier_msgs::srv::GetFrontierCosts::Request>();
                    request_frontier_costs->requested_frontier_list = frontier_list;
                    request_frontier_costs->robot_namespace = robot_name;
                    auto result_frontier_costs = client_get_frontier_costs_->async_send_request(request_frontier_costs);
                    std::shared_ptr<frontier_msgs::srv::GetFrontierCosts_Response> frontier_costs_srv_res;
                    if (rclcpp::spin_until_future_complete(client_node_, result_frontier_costs, std::chrono::seconds(2000)) == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        frontier_costs_srv_res = result_frontier_costs.get();
                        if (!frontier_costs_srv_res)
                        {
                            RCLCPP_ERROR(logger_, "Did not recieve a response.");
                        }
                        if (frontier_costs_srv_res->success == true)
                        {
                            RCLCPP_INFO_STREAM(logger_, "Processed: " << robot_name);
                        }
                        else if (frontier_costs_srv_res->success == false)
                        {
                            RCLCPP_INFO(logger_, "Server returned false. Probably busy.");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(logger_, "Failed to call the frontier costs handler for another robot.");
                    }
                }
            }
        }
        if (req->override_frontier_list == true)
        {
            frontier_list = req->frontier_list_to_override;
            RCLCPP_INFO(logger_, "Get next frontier called from another robot.");
        }

        // Select the frontier (Modify this for different algorithms)
        frontierSelect_->exportMapCoverage(polygon_xy_min_max_, startTime_);
        frontier_msgs::msg::Frontier selected;
        if (exploration_mode_ == "greedy")
        {
            BoundedExploreLayer::processGreedyApproach(selected, frontier_list, every_frontier, req, res);
        }

        else if (exploration_mode_ == "random")
        {
            BoundedExploreLayer::processRandomApproach(selected, frontier_list, every_frontier, req, res);
        }

        else if (exploration_mode_ == "ours")
        {
            BoundedExploreLayer::processOurApproach(selected, frontier_list, req, res);
        }

        else
        {
            RCLCPP_ERROR(logger_, "Invalid mode of exploration");
            rclcpp::shutdown();
        }
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);
        RCLCPP_INFO_STREAM(logger_, "Is the list overriden? : " << req->override_frontier_list);

        // Visualize the frontiers
        frontierSelect_->visualizeFrontier(frontier_list, every_frontier, layered_costmap_->getGlobalFrameID());

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
        res->success = true;
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
            tf_buffer_->transform(in, out, layered_costmap_->getGlobalFrameID(), tf2::durationFromSec(10));
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

        // draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

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
