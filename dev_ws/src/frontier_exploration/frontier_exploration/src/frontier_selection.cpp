#include <frontier_exploration/frontier_selection.hpp>
#include <frontier_exploration/util.hpp>

namespace frontier_exploration {

    FrontierSelectionNode::FrontierSelectionNode(rclcpp_lifecycle::LifecycleNode::SharedPtr node, nav2_costmap_2d::Costmap2D* costmap) {
        client_node_ = rclcpp::Node::make_shared("frontier_selection_node");

        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        frontier_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", custom_qos);
        frontier_cloud_pub->on_activate();

        all_frontier_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("all_frontiers", custom_qos);
        all_frontier_cloud_pub->on_activate();

        plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);
        plan_pub_->on_activate();

        // Create a publisher for the Marker message
        marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("path_fovs", 10);
        marker_publisher_->on_activate();

        landmark_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("landmark_marker", 10);
        landmark_publisher_->on_activate();

        viz_pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("u1_pose", 10);
        viz_pose_publisher_->on_activate();

        client_node_->declare_parameter("exploration_mode", "random");
        client_node_->get_parameter("exploration_mode", mode_);

        client_node_->declare_parameter("counter", 1);
        client_node_->get_parameter("counter", counter_value_);

        client_node_->declare_parameter("alpha", 0.35);
        client_node_->get_parameter("alpha", alpha_);

        client_node_->declare_parameter("beta", 0.50);
        client_node_->get_parameter("beta", beta_);

        client_node_->declare_parameter("frontier_detect_radius", 0.80);
        client_node_->get_parameter("frontier_detect_radius", frontierDetectRadius_);

        client_node_->declare_parameter("planner_allow_unknown", false);
        client_node_->get_parameter("planner_allow_unknown", planner_allow_unknown_);

        client_node_->declare_parameter("N_best_for_u2", 6);
        client_node_->get_parameter("N_best_for_u2", N_best_for_u2_);

        get_nodes_in_radius_client_ = client_node_->create_client<rtabmap_msgs::srv::GetNodesInRadius>("get_nodes_in_radius");
        // map_subscription_ = node->create_subscription<octomap_msgs::msg::Octomap>("octomap_full", 
        // 10, std::bind(&FrontierSelectionNode::mapCallback, this, std::placeholders::_1));

        // occ_map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&FrontierSelectionNode::occupancyMapCallback, this, std::placeholders::_1));
        costmap_ = costmap;
    }

    void landmarkViz(std::vector<std::vector<double>>& points, visualization_msgs::msg::Marker& marker_msg_) {

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
        for(auto point : points) {
            geometry_msgs::msg::Point point1;
            point1.x = point[0];
            point1.y = point[1];
            marker_msg_.points.push_back(point1);
        }
    }

    void landmarkViz(std::vector<geometry_msgs::msg::Pose>& points, visualization_msgs::msg::Marker& marker_msg_) {

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
        for(auto point : points) {
            geometry_msgs::msg::Point point1 = point.position;
            marker_msg_.points.push_back(point1);
        }
    }

    // @brief order of polygon points is : minx, miny, maxx, maxy
    SelectionResult FrontierSelectionNode::selectFrontierCountUnknowns(const std::vector<frontier_msgs::msg::Frontier>& frontier_list, std::vector<double> polygon_xy_min_max,
                                         std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res, geometry_msgs::msg::Point start_point_w, std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data, nav2_costmap_2d::Costmap2D* traversability_costmap) {
        traversability_costmap_ = traversability_costmap;
        std::map<frontier_exploration::FrontierWithMetaData, double> frontier_costs;
        frontier_msgs::msg::Frontier selected_frontier;
        geometry_msgs::msg::Quaternion selected_orientation;
        double theta_s_star = 0; // optimal sensor arrival orientation
        const double radius = 2.0; // max depth of camera fov
        const double delta_theta = 0.15; // spatial density
        const double camera_fov = 1.04; // camera fov
        auto startTime = std::chrono::high_resolution_clock::now();
        if(frontier_list.size() == 0) {
            RCLCPP_ERROR(logger_, "No frontiers found from frontier search.");
            res->success = false;
            res->next_frontier.pose.position = selected_frontier.centroid;
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        if(polygon_xy_min_max.size() <= 0) {
            RCLCPP_ERROR(logger_, "Frontier cannot be selected");
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        std::vector<FrontierWithMetaData> frontier_meta_data_vector;
        std::vector<std::pair<FrontierWithMetaData, double>> frontier_with_u1_utility;
        int min_traversable_distance = std::numeric_limits<int>::max();
        int max_arrival_info_per_frontier = 0.0;
        for (auto frontier : frontier_list) {
            // PRELIMINARY CHECKS
            auto plan_of_frontier = getPlanForFrontier(start_point_w, frontier, map_data, false);
            plan_pub_->publish(plan_of_frontier.first.path);
            int length_to_frontier = plan_of_frontier.first.path.poses.size();
            if(length_to_frontier == 0) {
                continue;
            }
            bool frontier_exists_ = false;
            for (auto blacklisted_frontier_ : frontier_blacklist_) {
                if(blacklisted_frontier_.initial == frontier.initial) {
                    frontier_exists_ = true;
                    break;
                }
            }
            if(frontier.min_distance > frontierDetectRadius_ && frontier_exists_ == false) {
                double sx, sy; // sensor x, sensor y, sensor orientation
                double wx, wy;
                unsigned int min_length = 0.0;
                int resolution_cut_factor = 5;
                unsigned int max_length = radius / (costmap_->getResolution());
                sx = frontier.initial.x;
                sy = frontier.initial.y;
                std::vector<int> information_along_ray; // stores the information along each ray in 2PI.

                for(double theta = 0; theta <= (2 * M_PI); theta+=delta_theta) {
                    std::vector<nav2_costmap_2d::MapLocation> traced_cells;
                    RayTracedCells cell_gatherer(*costmap_, traced_cells);

                    wx = sx + (radius*cos(theta));
                    wy = sy + (radius*sin(theta));

                    // Check if wx and wy are outside the polygon. If they are, bring it to the edge of the polygon.
                    // This is to prevent raytracing beyond the edge of the boundary polygon.
                    wx = std::max(polygon_xy_min_max[0], std::max(costmap_->getOriginX(), std::min(polygon_xy_min_max[2], std::min(costmap_->getOriginX() + costmap_->getSizeInMetersX(), wx))));
                    wy = std::max(polygon_xy_min_max[1], std::max(costmap_->getOriginY(), std::min(polygon_xy_min_max[3], std::min(costmap_->getOriginY() + costmap_->getSizeInMetersY(), wy))));
                    unsigned int x1,y1;
                    unsigned int x0, y0;
                    if(!costmap_->worldToMap(wx,wy,x1,y1) || !costmap_->worldToMap(sx,sy,x0,y0)) {
                        RCLCPP_ERROR(logger_, "Not world to map");
                        break;
                    }

                    int dx_full = x1 - x0;
                    int dy_full = y1 - y0;
                    double dist = std::hypot(dx_full, dy_full);
                    if (dist < min_length) {
                        SelectionResult selection_result;
                        selection_result.frontier = selected_frontier;
                        selection_result.orientation = selected_orientation;
                        selection_result.success = false;
                        selection_result.frontier_costs = frontier_costs;
                        return selection_result;
                    }
                    unsigned int min_x0, min_y0;
                    if (dist > 0.0) {
                        // Adjust starting point and offset to start from min_length distance
                        min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
                        min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
                    } else {
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
                    if (abs_dx >= abs_dy) {
                        int error_y = abs_dx / 2;

                        FrontierSelectionNode::bresenham2D(
                            cell_gatherer, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx), resolution_cut_factor);
                    }
                    else {
                        // otherwise y is dominant
                        int error_x = abs_dy / 2;
                        FrontierSelectionNode::bresenham2D(
                            cell_gatherer, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy), resolution_cut_factor);
                    }
                    auto info_addition = cell_gatherer.getCells();
                    information_along_ray.push_back(info_addition.size()); 
                    std::vector<geometry_msgs::msg::Pose> vizpoints;
                    for (size_t z = 0; z<info_addition.size(); z++) {
                        double wmx, wmy;
                        costmap_->mapToWorld(info_addition[z].x, info_addition[z].y, wmx, wmy);
                        geometry_msgs::msg::Pose pnts;
                        pnts.position.x = wmx;
                        pnts.position.y = wmy;
                        vizpoints.push_back(pnts);
                    }
                } // theta end
                
                std::vector<int> kernel(static_cast<int>(camera_fov / delta_theta), 1); // initialize a kernal vector of size 6 and all elements = 1
                int n = information_along_ray.size(); // number of rays computed in 2PI
                int k = kernel.size();
                std::vector<int> result(n - k + 1, 0);
                for (int i = 0; i < n - k + 1; ++i) {
                    for (int j = 0; j < k; ++j) {
                        result[i] += information_along_ray[i + j] * kernel[j];
                    }
                }
                int maxIndex = 0;
                int maxValue = result[0];
                for (int i = 1; i < result.size(); ++i) {
                    if (result[i] > maxValue) {
                        maxValue = result[i];
                        maxIndex = i;
                    }
                }
                // camera_fov / 2 is added because until here the maxIndex is only the starting index.
                FrontierWithMetaData f_info(frontier, maxValue, ((maxIndex * delta_theta) + (camera_fov / 2)), length_to_frontier);
                frontier_meta_data_vector.push_back(f_info);
                max_arrival_info_per_frontier = std::max(max_arrival_info_per_frontier, maxValue);
                min_traversable_distance = std::min(min_traversable_distance, length_to_frontier);
            }
        } // frontier end


        // U1 Utility
        double max_u1_utility = 0;
        for (auto frontier_with_properties : frontier_meta_data_vector) {
            // the distance term is inverted because we need to choose the closest frontier with tradeoff.
            auto utility = (alpha_ * (static_cast<double>(frontier_with_properties.information_) / static_cast<double>(max_arrival_info_per_frontier))) + 
                            ((1.0-alpha_) * (static_cast<double>(min_traversable_distance) / frontier_with_properties.path_length_));
            
            // It is added with Beta multiplied. If the frontier lies in the N best then this value will be replaced with information on path.
            // If it does not lie in the N best then the information on path is treated as 0 and hence it is appropriate to multiply with beta.
            frontier_costs[frontier_with_properties] = beta_ * utility;
            frontier_with_u1_utility.push_back(std::make_pair(frontier_with_properties, utility));
            if (utility > max_u1_utility) {
                max_u1_utility = utility;
                theta_s_star = frontier_with_properties.theta_s_star_;
                selected_frontier = frontier_with_properties.frontier_;
                selected_orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta_s_star);

                RCLCPP_DEBUG_STREAM(logger_, "Max u1 utility: " << max_u1_utility);
                RCLCPP_DEBUG_STREAM(logger_, "Max information: " << max_arrival_info_per_frontier);
                RCLCPP_DEBUG_STREAM(logger_, "Min distance " << min_traversable_distance);
            }
        }

        geometry_msgs::msg::PoseStamped vizpose;
        vizpose.header.frame_id = "map";
        vizpose.pose.position.x = selected_frontier.initial.x;
        vizpose.pose.position.y = selected_frontier.initial.y;
        vizpose.pose.orientation = selected_orientation;
        viz_pose_publisher_->publish(vizpose);


        FrontierU1Comparator frontier_u1_comp;
        // sort the frontier list based on the utility value
        std::sort(frontier_with_u1_utility.begin(), frontier_with_u1_utility.end(), frontier_u1_comp);


        // U2 UTILITY
        frontier_msgs::msg::Frontier::SharedPtr frontier_selected_post_u2;
        double theta_s_star_post_u2;
        // std::map of frontier information mapped to the new utility
        std::map<frontier_exploration::FrontierWithMetaData, double> frontier_with_path_information;

        if(frontier_with_u1_utility.size() > 0) {
            double max_u2_utility = 0;
            double maximum_path_information = 0;
            if(N_best_for_u2_ == -1) { // if -1 then the path information for all the frontiers is computed.
                N_best_for_u2_ = frontier_with_u1_utility.size();
            }
            for (size_t m = frontier_with_u1_utility.size() - 1; m >=frontier_with_u1_utility.size() - N_best_for_u2_; m--) {
                if(m == 0) {
                    break;
                }
                auto plan_result = getPlanForFrontier(start_point_w, frontier_with_u1_utility[m].first.frontier_, map_data, true);
                if(plan_result.first.information_total > maximum_path_information) {
                    maximum_path_information = plan_result.first.information_total;
                }
                frontier_with_path_information[frontier_with_u1_utility[m].first]=plan_result.first.information_total;
            }
            for (size_t m = frontier_with_u1_utility.size() - 1; m >=frontier_with_u1_utility.size() - N_best_for_u2_; m--) {
                if(m == 0) {
                    break;
                }
                double current_utility = ((beta_ * frontier_with_u1_utility[m].second) + ((1-beta_) * (frontier_with_path_information[frontier_with_u1_utility[m].first]/maximum_path_information)));
                frontier_costs[frontier_with_u1_utility[m].first] = current_utility;
                if(current_utility > max_u2_utility) {
                    max_u2_utility = current_utility;
                    theta_s_star_post_u2 = frontier_with_u1_utility[m].first.theta_s_star_;
                    frontier_selected_post_u2 = std::make_shared<frontier_msgs::msg::Frontier>(frontier_with_u1_utility[m].first.frontier_);
                }
            }
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = (endTime - startTime).count() / 1.0;
        RCLCPP_INFO_STREAM(logger_, "Time taken to search is: " << duration);
        // To handle the case where all the max frontiers have zero information.
        if(frontier_selected_post_u2) {
            selected_orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta_s_star_post_u2);
            frontier_blacklist_.push_back(*frontier_selected_post_u2);
            SelectionResult selection_result;
            selection_result.frontier = *frontier_selected_post_u2;
            selection_result.orientation = selected_orientation;
            selection_result.success = true;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }
        else {
            RCLCPP_WARN(logger_, "The selected frontier was not updated after U2 computation.");
            frontier_blacklist_.push_back(selected_frontier);
            SelectionResult selection_result;
            selection_result.frontier = selected_frontier;
            selection_result.orientation = selected_orientation;
            selection_result.success = true;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierClosest(const std::vector<frontier_msgs::msg::Frontier>& frontier_list, const std::vector<std::vector<double>>& every_frontier, std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res, std::string globalFrameID) {
        //create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        if(frontier_list.size() == 0){
            RCLCPP_ERROR(logger_, "No frontiers found, exploration complete");
            res->success = false;
            res->next_frontier.pose.position = selected.centroid;
            return std::make_pair(selected, false);
        }


        // FRONTIER SELECTION - CHANGE TO NEW FUNCTION IF YOU WISH TO ADD INTELLIGENCE.
        bool frontierSelectionFlag = false;
        auto count_index = 0;
        for (const auto& frontier : frontier_list) {
            count_index ++;

            //check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if(frontier.min_distance > frontierDetectRadius_){
                bool frontier_exists_ = false;
                for (auto blacklisted_frontier_ : frontier_blacklist_) {
                    if(blacklisted_frontier_.initial == frontier.initial) {
                        frontier_exists_ = true;
                        break;
                    }
                }
                if (frontier.min_distance < selected.min_distance && frontier_exists_ == false){
                    selected = frontier;
                    frontierSelectionFlag = true;
                }
            }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10000));

        if(frontierSelectionFlag == false){
            res->success = false;
            return std::make_pair(selected, frontierSelectionFlag);
        }
        frontier_blacklist_.push_back(selected);

        return std::make_pair(selected, frontierSelectionFlag);
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierRandom(const std::vector<frontier_msgs::msg::Frontier>& frontier_list, const std::vector<std::vector<double>>& every_frontier, std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res, std::string globalFrameID) {
        //create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        if(frontier_list.size() == 0){
            RCLCPP_ERROR(logger_, "No frontiers found, exploration complete");
            res->success = false;
            res->next_frontier.pose.position = selected.centroid;
            return std::make_pair(selected, false);
        }


        // FRONTIER SELECTION - CHANGE TO NEW FUNCTION IF YOU WISH TO ADD INTELLIGENCE.
        bool frontierSelectionFlag = false;
        auto count_index = 0;
        std::vector<frontier_msgs::msg::Frontier> frontier_list_imp;
        for (const auto& frontier : frontier_list) {
            count_index ++;

            //check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if(frontier.min_distance > frontierDetectRadius_){
                bool frontier_exists_ = false;
                for (auto blacklisted_frontier_ : frontier_blacklist_) {
                    if(blacklisted_frontier_.initial == frontier.initial) {
                        frontier_exists_ = true;
                        break;
                    }
                }
                if(frontier_exists_ == false) {
                    frontier_list_imp.push_back(frontier);
                }
            }
        }
        if(frontier_list_imp.size() > 0) {
                // Seed the random number generator
            std::random_device rd;
            std::mt19937 gen(rd());

            // Generate a random floating-point number between 0 and 1
            std::uniform_real_distribution<> dist(0.0, 1.0);
            double randomFraction = dist(gen);
            int randomIndex = static_cast<int>(randomFraction * (frontier_list_imp.size()-1));
            selected = frontier_list_imp[randomIndex];
            frontierSelectionFlag = true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10000));
        if(frontierSelectionFlag == false){
            res->success = false;
            return std::make_pair(selected, frontierSelectionFlag);
        }

        frontier_blacklist_.push_back(selected);
        return std::make_pair(selected, frontierSelectionFlag);
    }

    std::pair<PathWithInfo, bool> FrontierSelectionNode::getPlanForFrontier(geometry_msgs::msg::Point start_point_w, frontier_msgs::msg::Frontier goal_point_w, 
    std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data, bool compute_information) {
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
        marker_msg_.color.r = 1.0; // Red
        marker_msg_.color.a = 1.0; // Fully opaque

        nav_msgs::msg::Path plan;
        plan.header.frame_id = "map";
        std::unique_ptr<frontier_exploration::NavFn> planner_;
        planner_ = std::make_unique<frontier_exploration::NavFn>(traversability_costmap_->getSizeInCellsX(), traversability_costmap_->getSizeInCellsY());
        planner_->setNavArr(traversability_costmap_->getSizeInCellsX(), traversability_costmap_->getSizeInCellsY());
        planner_->setCostmap(traversability_costmap_->getCharMap(), true, planner_allow_unknown_);

        // start point
        unsigned int mx, my;
        if (!traversability_costmap_->worldToMap(start_point_w.x, start_point_w.y, mx, my)) {
            RCLCPP_WARN(
            logger_,
            "Cannot create a plan: the robot's start position is off the global"
            " costmap. Planning will always fail, are you sure"
            " the robot has been properly localized?");
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }
        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        // goal point
        if (!traversability_costmap_->worldToMap(goal_point_w.initial.x, goal_point_w.initial.y, mx, my)) {
            RCLCPP_WARN(
            logger_,
            "The goal sent to the planner is off the global costmap."
            " Planning will always fail to this goal.");
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }

        int map_goal[2];
        map_goal[0] = mx;
        map_goal[1] = my;

        planner_->setStart(map_goal); // Take note this is computed backwards. Copied what was done in nav2 navfn planner.
        planner_->setGoal(map_start);

        if(planner_->calcNavFnAstar()) {
            // RCLCPP_INFO(logger_, "Plan Found.");
        }
        if(!planner_->calcNavFnAstar()) {
            RCLCPP_ERROR(logger_, "Plan not Found.");
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }

        const int & max_cycles = (traversability_costmap_->getSizeInCellsX() >= traversability_costmap_->getSizeInCellsY()) ?
            (traversability_costmap_->getSizeInCellsX() * 4) : (traversability_costmap_->getSizeInCellsY() * 4);
        int path_len = planner_->calcPath(max_cycles);
        if (path_len == 0) {
            RCLCPP_ERROR(logger_, "Path length is zero");
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }
        auto cost = planner_->getLastPathCost();
        // extract the plan
        float * x = planner_->getPathX();
        float * y = planner_->getPathY();
        int len = planner_->getPathLen();
        int path_cut_count = 0;
        int fov_cut = 10;
        double number_of_wayp = 0;
        for (int i = len - 1; i >= 0; --i) {
            // convert the plan to world coordinates
            double world_x, world_y;
            traversability_costmap_->mapToWorld(x[i], y[i], world_x, world_y);
            geometry_msgs::msg::PoseStamped pose_from;
            pose_from.pose.position.x = world_x;
            pose_from.pose.position.y = world_y;
            pose_from.pose.position.z = 0.0;
            plan.poses.push_back(pose_from);
            path_cut_count ++;
            if(path_cut_count > static_cast<int>(1.5 / traversability_costmap_->getResolution()) && compute_information == true) {
                number_of_wayp++;
                visualization_msgs::msg::Marker marker_msg_points_;
                double world_x2, world_y2;
                traversability_costmap_->mapToWorld(x[std::max(i-10, 0)], y[std::max(i-10, 0)], world_x2, world_y2);
                geometry_msgs::msg::PoseStamped pose_to;
                pose_to.pose.position.x = world_x2;
                pose_to.pose.position.y = world_y2;
                pose_to.pose.position.z = 0.0;
                auto oriented_pose = frontier_exploration_planning::getRelativePoseGivenTwoPoints(pose_from.pose.position, pose_to.pose.position);
                auto vertices = frontier_exploration_utils::getVerticesOfFrustum2D(oriented_pose, 2.0, 1.089);
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

                // auto startTime = std::chrono::high_resolution_clock::now();
                auto nodes_in_radius = frontier_exploration_planning::getNodesInRadius(map_data->data.graph.poses, map_data->data.graph.poses_id, 4.5, request_pose, logger_);
                // auto endTime = std::chrono::high_resolution_clock::now();
                // auto duration = (endTime - startTime).count() / 1.0;
                // RCLCPP_INFO_STREAM(logger_, "Time taken to find nodes in radius is: " << duration);

                auto Q = Eigen::Matrix3d::Identity() * 0.01;
                auto info_pcl = frontier_exploration_information::computeInformationForPose(oriented_pose, 
                nodes_in_radius.first, nodes_in_radius.second, map_data->data, 2.0, 1.089, 0.5, Q, true, logger_, costmap_);
                if(info_pcl.first > 0) {
                    landmarkViz(info_pcl.second, marker_msg_points_);
                    landmark_publisher_->publish(marker_msg_points_);
                    // RCLCPP_INFO_STREAM(logger_, "The information is: " << info_pcl.first);
                    information_for_path += info_pcl.first;
                }
            }
        }
        marker_publisher_->publish(marker_msg_);
        plan_pub_->publish(plan);
        if(plan.poses.empty()) {
            struct_obj.path = plan;
            struct_obj.information_total = -1;
            return std::make_pair(struct_obj, false);
        }
        struct_obj.path = plan;
        if(number_of_wayp == 0)
            struct_obj.information_total = information_for_path;
        if(number_of_wayp != 0) {
            // RCLCPP_ERROR_STREAM(logger_, "Information is divided by: " << number_of_wayp);
            struct_obj.information_total = information_for_path / number_of_wayp;
        }
        return std::make_pair(struct_obj, true);
    }


    void FrontierSelectionNode::visualizeFrontier(const std::vector<frontier_msgs::msg::Frontier>& frontier_list, const std::vector<std::vector<double>>& every_frontier, std::string globalFrameID) {

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> all_frontier_cloud_viz;
        pcl::PointXYZI all_frontier_point_viz(50);

        for (const auto& frontier : frontier_list) {
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.initial.x;
            frontier_point_viz.y = frontier.initial.y;
            frontier_cloud_viz.push_back(frontier_point_viz);
        }

        for (const auto& frontier : every_frontier) {
            //load frontier into visualization poitncloud
            all_frontier_point_viz.x = frontier[0];
            all_frontier_point_viz.y = frontier[1];
            all_frontier_cloud_viz.push_back(all_frontier_point_viz);
        }

        //publish visualization point cloud
        sensor_msgs::msg::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = globalFrameID;
        frontier_viz_output.header.stamp = rclcpp::Clock().now();
        frontier_cloud_pub->publish(frontier_viz_output);

        //publish visualization point cloud (all frontiers)
        sensor_msgs::msg::PointCloud2 all_frontier_viz_output;
        pcl::toROSMsg(all_frontier_cloud_viz,all_frontier_viz_output);
        all_frontier_viz_output.header.frame_id = globalFrameID;
        all_frontier_viz_output.header.stamp = rclcpp::Clock().now();
        all_frontier_cloud_pub->publish(all_frontier_viz_output);

    }

    // @brief order of polygon points is : minx, miny, maxx, maxy
    void FrontierSelectionNode::exportMapCoverage(std::vector<double> polygon_xy_min_max, std::chrono::_V2::system_clock::time_point startTime) {
        int unknown = std::pow((polygon_xy_min_max[2] - polygon_xy_min_max[0]) / costmap_->getResolution(), 2);
        // auto currentTime = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsedTime = currentTime - startTime;
        // auto time = elapsedTime.count();        
        for (double y = polygon_xy_min_max[1]; y < polygon_xy_min_max[3]; y += costmap_->getResolution()) {
            for (double x = polygon_xy_min_max[0]; x < polygon_xy_min_max[2]; x += costmap_->getResolution()) {
                // Convert world coordinates to costmap grid coordinates
                unsigned int mx, my;
                if (costmap_->worldToMap(x, y, mx, my)) {                    
                    // Access the costmap value at grid coordinates (mx, my)
                    unsigned char cost = costmap_->getCost(mx, my);
                    if ((int)cost != 255) {
                        unknown--;
                    }
                }
            }
        }        
        std::ofstream file;
        file.open(std::to_string(counter_value_) + "_" + mode_ + "_frontier_map_data_coverage_.csv", std::ios::app);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open the CSV file for writing.");
            return;
        }
        file << std::fixed;
        file << std::setprecision(2);
        file << client_node_->get_clock()->now().seconds() << "," << unknown << std::endl;
        file.close();        
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierInformationOnPath(const std::vector<frontier_msgs::msg::Frontier>& frontier_list, 
        std::shared_ptr<frontier_msgs::srv::GetNextFrontier_Response> res, geometry_msgs::msg::Point start_pose, std::shared_ptr<rtabmap_msgs::srv::GetMap2_Response> map_data) {
        for(auto frontier: frontier_list) {
            getPlanForFrontier(start_pose, frontier, map_data, true);
        }
        frontier_msgs::msg::Frontier frntr;
        return std::make_pair(frntr, false);
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
        for (unsigned int i = 0; i < end; ++i) {
            if(i % resolution_cut_factor == 0)
                at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da) {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }
}
