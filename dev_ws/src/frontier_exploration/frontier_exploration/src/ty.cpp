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
        planner_->setCostmap(traversability_costmap_->getCharMap(), true, true);

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
            if(path_cut_count > 30 && compute_information == true) {
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
                nodes_in_radius.first, nodes_in_radius.second, map_data->data, 2.0, 1.089, 0.5, Q, true, logger_, traversability_costmap_);
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