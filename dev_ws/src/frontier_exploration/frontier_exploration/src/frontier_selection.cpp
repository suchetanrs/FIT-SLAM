#include <frontier_exploration/util.hpp>
#include <frontier_exploration/frontier_selection.hpp>

namespace frontier_exploration
{

    FrontierSelectionNode::FrontierSelectionNode(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
    {
        // Creating a client node for internal use
        frontier_selection_node_ = rclcpp::Node::make_shared("frontier_selection_node");

        logger_ = rclcpp::get_logger(static_cast<std::string>(node->get_namespace()) + ".frontier_selection");

        // Creating publishers with custom QoS settings
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        frontier_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", custom_qos);

        all_frontier_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("all_frontiers", custom_qos);

        frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);

        fov_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("path_fovs", 10);

        landmark_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("landmark_marker", 10);

        viz_pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("u1_pose", 10);

        path_pose_array_ = node->create_publisher<geometry_msgs::msg::PoseArray>("frontier_plan_poses", 10);

        // Setting default parameters and fetching them if available
        frontier_selection_node_->declare_parameter("exploration_mode", "random");
        frontier_selection_node_->get_parameter("exploration_mode", mode_);

        frontier_selection_node_->declare_parameter("counter", 1);
        frontier_selection_node_->get_parameter("counter", counter_value_);

        frontier_selection_node_->declare_parameter("alpha", 0.35);
        frontier_selection_node_->get_parameter("alpha", alpha_);

        frontier_selection_node_->declare_parameter("beta", 0.50);
        frontier_selection_node_->get_parameter("beta", beta_);

        frontier_selection_node_->declare_parameter("frontier_detect_radius", 0.80);
        frontier_selection_node_->get_parameter("frontier_detect_radius", frontierDetectRadius_);

        frontier_selection_node_->declare_parameter("planner_allow_unknown", false);
        frontier_selection_node_->get_parameter("planner_allow_unknown", planner_allow_unknown_);

        frontier_selection_node_->declare_parameter("N_best_for_u2", 6);
        frontier_selection_node_->get_parameter("N_best_for_u2", N_best_for_u2_);

        frontier_selection_node_->declare_parameter("use_planning", true);
        frontier_selection_node_->get_parameter("use_planning", use_planning_);

        costmap_ = costmap;
    }

    /**
     * @brief Visualizes landmarks using a marker message.
     * This function generates a visualization marker message for landmarks defined by a set of points.
     *
     * @param points A vector containing vectors of doubles representing the landmarks' coordinates.
     * @param marker_msg_ Reference to a visualization_msgs::msg::Marker message to be filled with landmark information.
     */
    void landmarkViz(std::vector<std::vector<double>> &points, visualization_msgs::msg::Marker &marker_msg_)
    {

        // Initialize the Marker message
        marker_msg_.header.frame_id = "map"; // Set the frame ID
        marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
        marker_msg_.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the points
        marker_msg_.scale.x = 0.05; // Point size
        marker_msg_.scale.y = 0.05;

        // Set the color (green in RGBA format)
        marker_msg_.color.r = 0.0;
        marker_msg_.color.g = 1.0;
        marker_msg_.color.b = 0.0;
        marker_msg_.color.a = 1.0;
        for (auto point : points)
        {
            geometry_msgs::msg::Point point1;
            point1.x = point[0];
            point1.y = point[1];
            marker_msg_.points.push_back(point1);
        }
    }

    /**
     * @brief Visualizes landmarks using a marker message.
     * This function generates a visualization marker message for landmarks defined by a set of pose points.
     *
     * @param points A vector containing geometry_msgs::msg::Pose representing the landmarks' positions.
     * @param marker_msg_ Reference to a visualization_msgs::msg::Marker message to be filled with landmark information.
     */
    void FrontierSelectionNode::landmarkViz(std::vector<geometry_msgs::msg::Pose> &points, visualization_msgs::msg::Marker &marker_msg_)
    {

        // Initialize the Marker message
        marker_msg_.header.frame_id = "map"; // Set the frame ID
        marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
        marker_msg_.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the points
        marker_msg_.scale.x = costmap_->getResolution(); // Point size
        marker_msg_.scale.y = costmap_->getResolution();

        // Set the color (green in RGBA format)
        marker_msg_.color.r = 0.0;
        marker_msg_.color.g = 1.0;
        marker_msg_.color.b = 0.0;
        marker_msg_.color.a = 1.0;
        for (auto point : points)
        {
            geometry_msgs::msg::Point point1 = point.position;
            marker_msg_.points.push_back(point1);
        }
    }

    /**
     * @brief Visualizes landmarks using a marker message.
     * This function generates a visualization marker message for landmarks defined by a set of pose points.
     *
     * @param points A vector containing geometry_msgs::msg::Pose representing the landmarks' positions.
     * @param marker_msg_ Reference to a visualization_msgs::msg::Marker message to be filled with landmark information.
     */
    void FrontierSelectionNode::landmarkViz(std::vector<Eigen::Vector3f> &points, visualization_msgs::msg::Marker &marker_msg_)
    {

        // Initialize the Marker message
        marker_msg_.header.frame_id = "map"; // Set the frame ID
        marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
        marker_msg_.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the points
        marker_msg_.scale.x = 0.1; // Point size
        marker_msg_.scale.y = 0.1;

        // Set the color (green in RGBA format)
        marker_msg_.color.r = 0.0;
        marker_msg_.color.g = 1.0;
        marker_msg_.color.b = 0.0;
        marker_msg_.color.a = 1.0;
        for (auto point : points)
        {
            geometry_msgs::msg::Point point1;
            point1.x = point.x();
            point1.y = point.y();
            point1.z = point.z();
            marker_msg_.points.push_back(point1);
        }
    }

    void FrontierSelectionNode::visualizeFrontier(const std::vector<frontier_msgs::msg::Frontier> &frontier_list, const std::vector<std::vector<double>> &every_frontier, std::string globalFrameID)
    {
        // pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);

        // pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> all_frontier_cloud_viz;
        pcl::PointXYZI all_frontier_point_viz(50);

        for (const auto &frontier : frontier_list)
        {
            // load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.goal_point.x;
            frontier_point_viz.y = frontier.goal_point.y;
            frontier_cloud_viz.push_back(frontier_point_viz);
        }

        for (const auto &frontier : every_frontier)
        {
            // load frontier into visualization poitncloud
            all_frontier_point_viz.x = frontier[0];
            all_frontier_point_viz.y = frontier[1];
            all_frontier_cloud_viz.push_back(all_frontier_point_viz);
        }

        // publish visualization point cloud
        sensor_msgs::msg::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz, frontier_viz_output);
        frontier_viz_output.header.frame_id = globalFrameID;
        frontier_viz_output.header.stamp = rclcpp::Clock().now();
        frontier_cloud_pub_->publish(frontier_viz_output);

        // publish visualization point cloud (all frontiers)
        sensor_msgs::msg::PointCloud2 all_frontier_viz_output;
        pcl::toROSMsg(all_frontier_cloud_viz, all_frontier_viz_output);
        all_frontier_viz_output.header.frame_id = globalFrameID;
        all_frontier_viz_output.header.stamp = rclcpp::Clock().now();
        all_frontier_cloud_pub_->publish(all_frontier_viz_output);
    }

    void FrontierSelectionNode::exportMapCoverage(std::vector<double> polygon_xy_min_max, std::chrono::_V2::system_clock::time_point startTime)
    {
        RCLCPP_INFO_STREAM(logger_, "Point2: " << polygon_xy_min_max[2]);
        RCLCPP_INFO_STREAM(logger_, "Point0: " << polygon_xy_min_max[0]);

        RCLCPP_INFO_STREAM(logger_, "Point1: " << polygon_xy_min_max[1]);
        RCLCPP_INFO_STREAM(logger_, "Point3: " << polygon_xy_min_max[3]);

        RCLCPP_INFO_STREAM(logger_, "Point0: " << polygon_xy_min_max[0]);
        RCLCPP_INFO_STREAM(logger_, "Point2: " << polygon_xy_min_max[2]);
        RCLCPP_INFO_STREAM(logger_, "Costmap is " << costmap_);
        int x_unknown = abs(polygon_xy_min_max[2] - polygon_xy_min_max[0]) / costmap_->getResolution();
        int y_unknown = abs(polygon_xy_min_max[3] - polygon_xy_min_max[1]) / costmap_->getResolution();
        int unknown = x_unknown * y_unknown;
        // int unknown = std::pow((polygon_xy_min_max[2] - polygon_xy_min_max[0]) / costmap_->getResolution(), 2);
        for (double y = polygon_xy_min_max[1]; y < polygon_xy_min_max[3]; y += costmap_->getResolution())
        {
            for (double x = polygon_xy_min_max[0]; x < polygon_xy_min_max[2]; x += costmap_->getResolution())
            {
                // Convert world coordinates to costmap grid coordinates
                unsigned int mx, my;
                if (costmap_->worldToMap(x, y, mx, my))
                {
                    // Access the costmap value at grid coordinates (mx, my)
                    unsigned char cost = costmap_->getCost(mx, my);
                    if ((int)cost != 255)
                    {
                        unknown--;
                    }
                }
            }
        }
        std::ofstream file;
        file.open(static_cast<std::string>(logger_.get_name()).substr(1, 7) + "_" + std::to_string(counter_value_) + "_" + mode_ + "_frontier_map_data_coverage.csv", std::ios::app);
        if (!file.is_open())
        {
            RCLCPP_ERROR(logger_, "Failed to open the CSV file for writing.");
            return;
        }
        file << std::fixed;
        file << std::setprecision(2);
        file << frontier_selection_node_->get_clock()->now().seconds() << "," << unknown << std::endl;
        file.close();
    }

    std::vector<frontier_msgs::msg::Frontier> findDuplicates(const std::vector<frontier_msgs::msg::Frontier> &vec)
    {
        std::vector<frontier_msgs::msg::Frontier> duplicates;

        // Iterate through the vector
        for (size_t i = 0; i < vec.size(); ++i)
        {
            // Compare the current element with all subsequent elements
            for (size_t j = i + 1; j < vec.size(); ++j)
            {
                if (vec[i] == vec[j])
                {
                    // If a duplicate is found, add it to the duplicates vector
                    duplicates.push_back(vec[i]);
                    break; // Break the inner loop to avoid adding the same duplicate multiple times
                }
            }
        }

        return duplicates;
    }

    SelectionResult FrontierSelectionNode::selectFrontierOurs(std::vector<frontier_msgs::msg::Frontier> &frontier_list, std::vector<double> polygon_xy_min_max,
                                                              geometry_msgs::msg::Point start_point_w, std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, nav2_costmap_2d::Costmap2D *exploration_costmap)
    {
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("FrontierSelectionNode::selectFrontierOurs", logger_.get_name()));
        exploration_costmap_ = exploration_costmap;
        // Vector to store frontiers with meta data and the u1 utility values
        std::vector<FrontierWithMetaData> frontier_meta_data_vector;
        std::vector<std::pair<FrontierWithMetaData, double>> frontier_with_u1_utility;
        std::unordered_map<FrontierWithMetaData, double, FrontierWithMetaData::Hash> frontier_costs;       
        frontier_msgs::msg::Frontier selected_frontier;
        geometry_msgs::msg::Quaternion selected_orientation;
        double theta_s_star = 0;         // optimal sensor arrival orientation
        const double radius = 2.0;       // max depth of camera fov
        const double delta_theta = 0.15; // spatial density
        const double camera_fov = 1.04;  // camera fov
        auto startTime = std::chrono::high_resolution_clock::now();
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found from frontier search.");
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        if (polygon_xy_min_max.size() <= 0)
        {
            RCLCPP_ERROR(logger_, "Frontier cannot be selected, no polygon.");
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        // Variables to track minimum traversable distance and maximum arrival information per frontier
        double min_traversable_distance = std::numeric_limits<double>::max();
        int max_arrival_info_per_frontier = 0.0;
        // Iterate through each frontier
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Frontier list size is (loop): " + std::to_string(frontier_list.size()), logger_.get_name()));
        auto frontier_list_duplicates = findDuplicates(frontier_list);
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("Duplicates size is: " + std::to_string(frontier_list_duplicates.size()), logger_.get_name()));
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("Blacklist size is: " + std::to_string(frontier_blacklist_.size()), logger_.get_name()));
        for (auto& frontier : frontier_list)
        {
            // Continue to next frontier if frontier is in blacklist
            bool frontier_exists_in_blacklist_ = false;
            if(frontier_blacklist_.count(frontier) > 0)
                frontier_exists_in_blacklist_ = true;

            // Preliminary checks and path planning for each frontier
            auto startTimePlan = std::chrono::high_resolution_clock::now();
            double length_to_frontier;
            if(use_planning_)
            {
                auto plan_of_frontier = getPlanForFrontier(start_point_w, frontier, map_data, false);
                // assume each pose is resolution * ((1 + root2) / 2) distance apart 
                length_to_frontier = static_cast<double>(plan_of_frontier.first.path.poses.size() * (costmap_->getResolution() * 1.207));
                // Continue to next frontier if path length is zero
                if (length_to_frontier == 0 || plan_of_frontier.second == false)
                {
                    RCLCPP_WARN_STREAM(logger_, COLOR_STR("Path length is zero or false", logger_.get_name()));
                    FrontierWithMetaData f_info_blacklisted(frontier, 0, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                    frontier_costs[f_info_blacklisted] = std::numeric_limits<double>::max();
                    continue;
                }
                // frontier_plan_pub_->publish(plan_of_frontier.first.path);
            }
            else
            {
                length_to_frontier = sqrt(pow(start_point_w.x - frontier.goal_point.x, 2) + pow(start_point_w.y - frontier.goal_point.y, 2));
                // length_to_frontier = frontier.min_distance;
            }
            auto endTimePlan = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> durationPlan = (endTimePlan - startTimePlan);
            RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("Time taken to Plan is: " + std::to_string(durationPlan.count()), logger_.get_name()));


            // Proceed to the function only if frontier is above the detection radius.
            if (length_to_frontier > frontierDetectRadius_ && frontier_exists_in_blacklist_ == false)
            {
                auto startTime = std::chrono::high_resolution_clock::now();
                double sx, sy; // sensor x, sensor y, sensor orientation
                double wx, wy;
                unsigned int min_length = 0.0;
                int resolution_cut_factor = 1;
                unsigned int max_length = radius / (costmap_->getResolution());
                sx = frontier.goal_point.x;
                sy = frontier.goal_point.y;
                std::vector<int> information_along_ray; // stores the information along each ray in 2PI.
                std::vector<geometry_msgs::msg::Pose> vizpoints;
                // Iterate through each angle in 2PI with delta_theta resolution
                for (double theta = 0; theta <= (2 * M_PI); theta += delta_theta)
                {
                    std::vector<nav2_costmap_2d::MapLocation> traced_cells;
                    RayTracedCells cell_gatherer(*costmap_, traced_cells);

                    wx = sx + (radius * cos(theta));
                    wy = sy + (radius * sin(theta));

                    // Check if wx and wy are outside the polygon. If they are, bring it to the edge of the polygon.
                    // This is to prevent raytracing beyond the edge of the boundary polygon.
                    wx = std::max(polygon_xy_min_max[0], std::max(costmap_->getOriginX(), std::min(polygon_xy_min_max[2], std::min(costmap_->getOriginX() + costmap_->getSizeInMetersX(), wx))));
                    wy = std::max(polygon_xy_min_max[1], std::max(costmap_->getOriginY(), std::min(polygon_xy_min_max[3], std::min(costmap_->getOriginY() + costmap_->getSizeInMetersY(), wy))));

                    // Calculate map coordinates
                    unsigned int x1, y1;
                    unsigned int x0, y0;
                    if (!costmap_->worldToMap(wx, wy, x1, y1) || !costmap_->worldToMap(sx, sy, x0, y0))
                    {
                        RCLCPP_ERROR(logger_, "Not world to map");
                        break;
                    }

                    // Calculate distance and adjust starting point to min_length distance
                    int dx_full = x1 - x0;
                    int dy_full = y1 - y0;
                    double dist = std::hypot(dx_full, dy_full);
                    if (dist < min_length)
                    {
                        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Distance to ray trace is lesser than minimum distance. Proceeding to next frontier.", logger_.get_name()));
                        SelectionResult selection_result;
                        selection_result.frontier = selected_frontier;
                        selection_result.orientation = selected_orientation;
                        selection_result.success = false;
                        selection_result.frontier_costs = frontier_costs;
                        return selection_result;
                    }
                    unsigned int min_x0, min_y0;
                    if (dist > 0.0)
                    {
                        // Adjust starting point and offset to start from min_length distance
                        min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
                        min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
                    }
                    else
                    {
                        min_x0 = x0;
                        min_y0 = y0;
                    }
                    unsigned int offset = min_y0 * costmap_->getSizeInCellsX() + min_x0;

                    int dx = x1 - min_x0;
                    int dy = y1 - min_y0;

                    unsigned int abs_dx = abs(dx);
                    unsigned int abs_dy = abs(dy);

                    int offset_dx = sign(dx);
                    int offset_dy = sign(dy) * costmap_->getSizeInCellsX();

                    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
                    // Calculate the maximum number of steps based on resolution_cut_factor
                    // if x is dominant
                    if (abs_dx >= abs_dy)
                    {
                        int error_y = abs_dx / 2;

                        FrontierSelectionNode::bresenham2D(
                            cell_gatherer, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx), resolution_cut_factor);
                    }
                    else
                    {
                        // otherwise y is dominant
                        int error_x = abs_dy / 2;
                        FrontierSelectionNode::bresenham2D(
                            cell_gatherer, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy), resolution_cut_factor);
                    }

                    auto info_addition = cell_gatherer.getCells();
                    information_along_ray.push_back(info_addition.size());
                    // loop for visualization
                    // for (size_t counter_info = 0; counter_info < info_addition.size(); counter_info++)
                    // {
                    //     double wmx, wmy;
                    //     costmap_->mapToWorld(info_addition[counter_info].x, info_addition[counter_info].y, wmx, wmy);
                    //     geometry_msgs::msg::Pose pnts;
                    //     pnts.position.x = wmx;
                    //     pnts.position.y = wmy;
                    //     vizpoints.push_back(pnts);
                    // }
                } // theta end

                std::vector<int> kernel(static_cast<int>(camera_fov / delta_theta), 1); // initialize a kernal vector of size 6 and all elements = 1
                int n = information_along_ray.size();                                   // number of rays computed in 2PI
                RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("n is: " + std::to_string(n), logger_.get_name()));
                int k = kernel.size();
                RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("k is: " + std::to_string(k), logger_.get_name()));
                std::vector<int> result(n - k + 1, 0);
                for (int i = 0; i < n - k + 1; ++i)
                {
                    for (int j = 0; j < k; ++j)
                    {
                        result[i] += information_along_ray[i + j] * kernel[j];
                    }
                }
                int maxIndex = 0;
                int maxValue = result[0];
                for (int i = 1; i < result.size(); ++i)
                {
                    if (result[i] > maxValue)
                    {
                        maxValue = result[i];
                        maxIndex = i;
                    }
                }

                // Calculate frontier information and add to metadata vector
                // camera_fov / 2 is added because until here the maxIndex is only the starting index.
                FrontierWithMetaData f_info(frontier, maxValue, ((maxIndex * delta_theta) + (camera_fov / 2)), length_to_frontier);
                frontier_meta_data_vector.push_back(f_info);
                max_arrival_info_per_frontier = std::max(max_arrival_info_per_frontier, maxValue);
                min_traversable_distance = std::min(min_traversable_distance, length_to_frontier);
                auto endTime = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration = (endTime - startTime);
                RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("Time taken to raytrace is: " + std::to_string(duration.count()), logger_.get_name()));
                RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("Total unknown cells is: " + std::to_string(std::accumulate(information_along_ray.begin(), information_along_ray.end(), 0)), logger_.get_name()));
                
                // visualize raytraced points
                // visualization_msgs::msg::Marker marker_msg_raytraced_;
                // landmarkViz(vizpoints, marker_msg_raytraced_);
                // landmark_publisher_->publish(marker_msg_raytraced_);
            }
            else
            {
                RCLCPP_ERROR_STREAM(logger_, COLOR_STR("Adding max cost: " + std::to_string(frontier.unique_id) + " ," + std::to_string(frontier.min_distance > frontierDetectRadius_) + " " + std::to_string(frontier_exists_in_blacklist_ == false), logger_.get_name()));
                FrontierWithMetaData f_info_blacklisted(frontier, 0, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                frontier_costs[f_info_blacklisted] = std::numeric_limits<double>::max();
            }
            
        } // frontier end
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(before) Frontier u1 with utility size is: " + std::to_string(frontier_with_u1_utility.size()), logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(before) Frontier costs size u1 is: " + std::to_string(frontier_costs.size()), logger_.get_name()));

        // U1 Utility
        double max_u1_utility = 0;
        for (auto frontier_with_properties : frontier_meta_data_vector)
        {
            // the distance term is inverted because we need to choose the closest frontier with tradeoff.
            auto utility = (alpha_ * (static_cast<double>(frontier_with_properties.information_) / static_cast<double>(max_arrival_info_per_frontier))) +
                           ((1.0 - alpha_) * (static_cast<double>(min_traversable_distance) / frontier_with_properties.path_length_));

            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 information:" << frontier_with_properties.information_);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 distance:" << min_traversable_distance);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 max info:" << max_arrival_info_per_frontier);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 max distance:" << frontier_with_properties.path_length_);
            // It is added with Beta multiplied. If the frontier lies in the N best then this value will be replaced with information on path.
            // If it does not lie in the N best then the information on path is treated as 0 and hence it is appropriate to multiply with beta.
            if(frontier_costs.count(frontier_with_properties) == 1)
                throw std::runtime_error("Something is wrong. Duplicate frontiers?");
            frontier_costs[frontier_with_properties] = (beta_ * utility) == 0 ? std::numeric_limits<double>::max() : 1 / (beta_ * utility);
            frontier_with_u1_utility.push_back(std::make_pair(frontier_with_properties, beta_ * utility));
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1:" << 1 / (beta_ * utility));
            if (beta_ * utility > max_u1_utility)
            {
                max_u1_utility = beta_ * utility;
                theta_s_star = frontier_with_properties.theta_s_star_;
                selected_frontier = frontier_with_properties.frontier_;
                selected_orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta_s_star);

                RCLCPP_DEBUG_STREAM(logger_, "Max u1 utility: " << max_u1_utility);
                RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("Min u1 cost: " + std::to_string(1 / max_u1_utility), logger_.get_name()));
                RCLCPP_DEBUG_STREAM(logger_, "Max information: " << max_arrival_info_per_frontier);
                RCLCPP_DEBUG_STREAM(logger_, "Min distance " << min_traversable_distance);
            }
        }
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(after) Frontier u1 with utility size is: " + std::to_string(frontier_with_u1_utility.size()), logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(after) Frontier costs size after u1 is: " + std::to_string(frontier_costs.size()), logger_.get_name()));

        RCLCPP_DEBUG_STREAM(logger_, "Alpha_: " << alpha_);
        RCLCPP_DEBUG_STREAM(logger_, "Beta_: " << beta_);
        geometry_msgs::msg::PoseStamped vizpose;
        vizpose.header.frame_id = "map";
        vizpose.pose.position.x = selected_frontier.goal_point.x;
        vizpose.pose.position.y = selected_frontier.goal_point.y;
        vizpose.pose.orientation = selected_orientation;
        viz_pose_publisher_->publish(vizpose);
        // FrontierU1ComparatorCost frontier_u1_comp;
        // // sort the frontier list based on the utility value
        // std::sort(frontier_with_u1_utility.begin(), frontier_with_u1_utility.end(), frontier_u1_comp);

        // // U2 UTILITY processed for N best.
        frontier_msgs::msg::Frontier::SharedPtr frontier_selected_post_u2;
        double theta_s_star_post_u2;
        // // std::map of frontier information mapped to the new utility
        // std::map<frontier_exploration::FrontierWithMetaData, double> frontier_with_path_information;

        // if (static_cast<int>(frontier_with_u1_utility.size()) > 0)
        // {
        //     RCLCPP_DEBUG_STREAM(logger_, "The value of N_best_for_u2 is: " << N_best_for_u2_);
        //     double max_u2_utility = 0;
        //     double maximum_path_information = 0;
        //     if (N_best_for_u2_ == -1)
        //     { // if -1 then the path information for all the frontiers is computed.
        //         N_best_for_u2_ = static_cast<int>(frontier_with_u1_utility.size());
        //     }
        //     RCLCPP_DEBUG_STREAM(logger_, "The value of loop min is: " << static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_);
        //     for (int m = static_cast<int>(frontier_with_u1_utility.size()) - 1; m >= static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_; m--)
        //     {
        //         if (m == 0)
        //         {
        //             break;
        //         }
        //         auto plan_result = getPlanForFrontier(start_point_w, frontier_with_u1_utility[m].first.frontier_, map_data, true);
        //         if (plan_result.first.information_total > maximum_path_information)
        //         {
        //             maximum_path_information = plan_result.first.information_total;
        //         }
        //         frontier_with_path_information[frontier_with_u1_utility[m].first] = plan_result.first.information_total;
        //     }
        //     for (int m = static_cast<int>(frontier_with_u1_utility.size()) - 1; m >= static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_; m--)
        //     {
        //         if (m == 0)
        //         {
        //             break;
        //         }
        //         double current_utility = ((beta_ * frontier_with_u1_utility[m].second) + ((1 - beta_) * (frontier_with_path_information[frontier_with_u1_utility[m].first] / maximum_path_information)));
        //         frontier_costs[frontier_with_u1_utility[m].first] = current_utility;
        //         if (current_utility > max_u2_utility)
        //         {
        //             RCLCPP_DEBUG_STREAM(logger_, "Current U2 utility: " << current_utility);
        //             RCLCPP_DEBUG_STREAM(logger_, "U1 utility for current one: " << frontier_with_u1_utility[m].second);
        //             max_u2_utility = current_utility;
        //             theta_s_star_post_u2 = frontier_with_u1_utility[m].first.theta_s_star_;
        //             frontier_selected_post_u2 = std::make_shared<frontier_msgs::msg::Frontier>(frontier_with_u1_utility[m].first.frontier_);
        //         }
        //     }
        // }
        // else
        // {
        //     RCLCPP_WARN_STREAM(logger_, COLOR_STR("The number of frontiers after U1 compute is zero.", logger_.get_name()));
        //     SelectionResult selection_result;
        //     selection_result.frontier = selected_frontier;
        //     selection_result.orientation = selected_orientation;
        //     selection_result.success = false;
        //     selection_result.frontier_costs = frontier_costs;
        //     return selection_result;
        // }
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = (endTime - startTime);
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("Time taken to search is: " + std::to_string(duration.count()), logger_.get_name()));

        if(frontier_costs.size() != frontier_list.size())
        {
            throw std::runtime_error("The returned size is not the same as input size.");
        }

        // To handle the case where all the max frontiers have zero information.
        if (frontier_selected_post_u2)
        {
            selected_orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta_s_star_post_u2);
            SelectionResult selection_result;
            selection_result.frontier = *frontier_selected_post_u2;
            selection_result.orientation = selected_orientation;
            selection_result.success = true;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }
        else
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("The selected frontier was not updated after U2 computation.", logger_.get_name()));
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("Returning for input list size: " + std::to_string(frontier_list.size()), logger_.get_name()));
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("Returning frontier costs size: " + std::to_string(frontier_costs.size()), logger_.get_name()));
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = true;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierClosest(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        // create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::max();

        // initialize with false, becomes true if frontier is selected.
        bool frontierSelectionFlag = false;
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found");
            frontierSelectionFlag = false;
            return std::make_pair(selected, false);
        }

        // Iterate through each frontier in the list
        for (auto &frontier : frontier_list)
        {
            // check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if (frontier.min_distance > frontierDetectRadius_)
            {
                bool frontier_exists_in_blacklist_ = false;
                // check if its blacklisted
                if(frontier_blacklist_.count(frontier) > 0)
                    frontier_exists_in_blacklist_ = true;
                if (frontier.min_distance < selected.min_distance && frontier_exists_in_blacklist_ == false)
                {
                    selected = frontier;
                    frontierSelectionFlag = true;
                }
            }
        }

        // If no frontier is selected, return
        if (frontierSelectionFlag == false)
        {
            RCLCPP_ERROR_STREAM(logger_, "No clustered frontiers are outside the minimum detection radius. Radius is: " << frontierDetectRadius_);
            return std::make_pair(selected, frontierSelectionFlag);
        }
        // Add the selected frontier to the blacklist to prevent re-selection
        return std::make_pair(selected, frontierSelectionFlag);
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierRandom(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        // create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::max();

        bool frontierSelectionFlag = false;
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found");
            frontierSelectionFlag = false;
            return std::make_pair(selected, false);
        }

        // Create a vector to store eligible frontiers
        std::vector<frontier_msgs::msg::Frontier> frontier_list_imp;
        // Iterate through each frontier in the list
        for (auto &frontier : frontier_list)
        {
            // check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if (frontier.min_distance > frontierDetectRadius_)
            {
                bool frontier_exists_in_blacklist_ = false;
                if(frontier_blacklist_.count(frontier) > 0)
                    frontier_exists_in_blacklist_ = true;
                if (frontier_exists_in_blacklist_ == false)
                {
                    frontier_list_imp.push_back(frontier);
                }
            }
        }
        if (frontier_list_imp.size() > 0)
        {
            // Seed the random number generator
            std::random_device rd;
            std::mt19937 gen(rd());

            // Generate a random floating-point number between 0 and 1
            std::uniform_real_distribution<> dist(0.0, 1.0);
            double randomFraction = dist(gen);
            int randomIndex = static_cast<int>(randomFraction * (frontier_list_imp.size() - 1));
            selected = frontier_list_imp[randomIndex];
            frontierSelectionFlag = true;
        }
        else
        {
            frontierSelectionFlag = false;
        }

        if (frontierSelectionFlag == false)
        {
            RCLCPP_ERROR_STREAM(logger_, "No clustered frontiers are outside the minimum detection radius. Radius is: " << frontierDetectRadius_);
            return std::make_pair(selected, frontierSelectionFlag);
        }

        // Add the selected frontier to the blacklist
        return std::make_pair(selected, frontierSelectionFlag);
    }

    std::pair<PathWithInfo, bool> FrontierSelectionNode::getPlanForFrontier(geometry_msgs::msg::Point start_point_w, frontier_msgs::msg::Frontier goal_point_w,
                                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information)
    {
        double information_for_path = 0;
        PathWithInfo struct_obj;
        visualization_msgs::msg::Marker marker_msg_;
        // Initialize the Marker message
        marker_msg_.header.frame_id = "map"; // Set the frame ID
        marker_msg_.ns = "two_triangles_namespace";
        marker_msg_.id = 0;
        marker_msg_.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_msg_.action = visualization_msgs::msg::Marker::ADD;
        marker_msg_.scale.x = 0.02; // Line width
        marker_msg_.color.r = 1.0;  // Red
        marker_msg_.color.a = 1.0;  // Fully opaque

        nav_msgs::msg::Path plan;
        plan.header.frame_id = "map";
        std::unique_ptr<frontier_exploration::NavFn> planner_;
        planner_ = std::make_unique<frontier_exploration::NavFn>(exploration_costmap_->getSizeInCellsX(), exploration_costmap_->getSizeInCellsY());
        planner_->setNavArr(exploration_costmap_->getSizeInCellsX(), exploration_costmap_->getSizeInCellsY());
        planner_->setCostmap(exploration_costmap_->getCharMap(), true, planner_allow_unknown_);

        // start point
        unsigned int mx, my;
        if (!exploration_costmap_->worldToMap(start_point_w.x, start_point_w.y, mx, my))
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("Cannot create a plan: the robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?", logger_.get_name()));
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }
        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        // goal point
        if (!exploration_costmap_->worldToMap(goal_point_w.goal_point.x, goal_point_w.goal_point.y, mx, my))
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("The goal sent to the planner is off the global costmap Planning will always fail to this goal.", logger_.get_name()));
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }

        int map_goal[2];
        map_goal[0] = mx;
        map_goal[1] = my;

        // Take note this is computed backwards. Copied what was done in nav2 navfn planner.
        planner_->setStart(map_goal);
        planner_->setGoal(map_start);

        if (!planner_->calcNavFnAstar())
        {
            RCLCPP_ERROR(logger_, "Plan not Found.");
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }

        const int &max_cycles = (exploration_costmap_->getSizeInCellsX() >= exploration_costmap_->getSizeInCellsY()) ? (exploration_costmap_->getSizeInCellsX() * 4) : (exploration_costmap_->getSizeInCellsY() * 4);
        int path_len = planner_->calcPath(max_cycles);
        if (path_len == 0)
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("Path length is zero", logger_.get_name()));
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }

        auto cost = planner_->getLastPathCost();
        // extract the plan
        float *x = planner_->getPathX();
        float *y = planner_->getPathY();
        int len = planner_->getPathLen();
        int path_cut_count = 0; // This variable is used to sample the poses in the path.
        double number_of_wayp = 0;
        RCLCPP_DEBUG_STREAM(logger_, "Compute information? << " << compute_information);
        for (int i = len - 1; i >= 0; --i)
        {
            // convert the plan to world coordinates
            double world_x, world_y;
            exploration_costmap_->mapToWorld(x[i], y[i], world_x, world_y);
            geometry_msgs::msg::PoseStamped pose_from;
            pose_from.pose.position.x = world_x;
            pose_from.pose.position.y = world_y;
            pose_from.pose.position.z = 0.0;
            plan.poses.push_back(pose_from);
            path_cut_count++;
            // If the path_cut_count variable is above a certain metric. The following block is executed.
            // This is done so as to prevent excessive information computation.
            if (path_cut_count > static_cast<int>(1.5 / exploration_costmap_->getResolution()) && compute_information == true)
            {
                number_of_wayp++;
                visualization_msgs::msg::Marker marker_msg_points_;
                double world_x2, world_y2;
                exploration_costmap_->mapToWorld(x[std::max(i - 10, 0)], y[std::max(i - 10, 0)], world_x2, world_y2);
                geometry_msgs::msg::PoseStamped pose_to;
                pose_to.pose.position.x = world_x2;
                pose_to.pose.position.y = world_y2;
                pose_to.pose.position.z = 0.0;
                // Calculate orientation of the pose and generate frustum vertices
                geometry_msgs::msg::Pose oriented_pose;
                frontier_exploration_planning::getRelativePoseGivenTwoPoints(pose_from.pose.position, pose_to.pose.position, oriented_pose);
                auto vertices = frontier_exploration_utils::getVerticesOfFrustum2D(oriented_pose, 2.0, 1.089);
                // Add frustum vertices to the marker message
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[0]));
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[1]));
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[1]));
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[2]));
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[2]));
                marker_msg_.points.push_back(frontier_exploration_utils::getPointFromVector(vertices[0]));
                path_cut_count = 0;

                // Getting poses within 4.5 m radius.
                geometry_msgs::msg::Pose request_pose;
                request_pose.position.x = oriented_pose.position.x;
                request_pose.position.y = oriented_pose.position.y;
                auto nodes_in_radius = frontier_exploration_planning::getNodesInRadius(map_data->data.graph.poses, map_data->data.graph.poses_id, 4.5, request_pose, logger_);

                auto Q = Eigen::Matrix3f::Identity() * 0.01f;
                auto info_pcl = frontier_exploration_information_affine::computeInformationForPose(oriented_pose,
                                                                                                   nodes_in_radius.first, nodes_in_radius.second, map_data->data, 2.0, 1.089, 0.5, Q, true, logger_, costmap_, true);
                if (info_pcl.first > 0)
                {
                    landmarkViz(info_pcl.second, marker_msg_points_);
                    landmark_publisher_->publish(marker_msg_points_);
                    information_for_path += info_pcl.first;
                }
            }
        }
        fov_marker_publisher_->publish(marker_msg_);
        frontier_plan_pub_->publish(plan);
        if (plan.poses.empty())
        {
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }
        struct_obj.path = plan;
        if (number_of_wayp == 0)
        {
            struct_obj.information_total = information_for_path;
        }
        if (number_of_wayp != 0)
        {
            // This is done so as to normalize the information on the path.
            struct_obj.information_total = information_for_path / number_of_wayp;
        }
        return std::make_pair(struct_obj, true);
    }

    void FrontierSelectionNode::setFrontierBlacklist(std::vector<frontier_msgs::msg::Frontier>& blacklist)
    {
        std::lock_guard<std::mutex> lock(blacklist_mutex_);
        for (auto frontier : blacklist)
        {
            frontier_blacklist_[frontier] = true;
        }
        for (auto frontier : frontier_blacklist_)
        {
            RCLCPP_ERROR_STREAM(logger_, COLOR_STR("Blacklist: " + std::to_string(frontier.first.goal_point.x), logger_.get_name()));
        }
    }

    void FrontierSelectionNode::bresenham2D(
        RayTracedCells at, unsigned int abs_da, unsigned int abs_db, int error_b,
        int offset_a,
        int offset_b, unsigned int offset,
        unsigned int max_length,
        int resolution_cut_factor)
    {
        auto max_offset = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            if (i % resolution_cut_factor == 0)
                at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }
}
