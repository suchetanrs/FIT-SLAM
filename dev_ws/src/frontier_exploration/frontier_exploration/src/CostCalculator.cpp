#include "frontier_exploration/CostCalculator.hpp"
#include <frontier_exploration/util.hpp>

namespace frontier_exploration
{
    FrontierCostCalculator::FrontierCostCalculator(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
    {
        node_ = node;
        logger_ = node_->get_logger();
        exploration_costmap_ = costmap;
        rosVisualizer_ = std::make_shared<RosVisualizer>(node, exploration_costmap_);
        fov_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("path_fovs", 10);
        rosVisualizer_ = std::make_shared<RosVisualizer>(node, exploration_costmap_);
        min_traversable_distance = std::numeric_limits<double>::max();
        max_arrival_info_per_frontier = 0.0;
    }

    void FrontierCostCalculator::setArrivalInformationForFrontier(Frontier &frontier, std::vector<double> &polygon_xy_min_max)
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        auto startTime = std::chrono::high_resolution_clock::now();
        const double radius = 2.0;       // max depth of camera fov
        const double delta_theta = 0.15; // spatial density
        const double camera_fov = 1.04;  // camera fov

        double sx, sy; // sensor x, sensor y, sensor orientation
        double wx, wy;
        unsigned int max_length = radius / (exploration_costmap_->getResolution());
        sx = frontier.getGoalPoint().x;
        sy = frontier.getGoalPoint().y;
        std::vector<int> information_along_ray; // stores the information along each ray in 2PI.
        std::vector<geometry_msgs::msg::Pose> vizpoints;
        // Iterate through each angle in 2PI with delta_theta resolution
        for (double theta = 0; theta <= (2 * M_PI); theta += delta_theta)
        {
            std::vector<nav2_costmap_2d::MapLocation> traced_cells;
            RayTracedCells cell_gatherer(exploration_costmap_, traced_cells, 240, 254, 255, 255);

            wx = sx + (radius * cos(theta));
            wy = sy + (radius * sin(theta));

            // Check if wx and wy are outside the polygon. If they are, bring it to the edge of the polygon.
            // This is to prevent raytracing beyond the edge of the boundary polygon.
            wx = std::max(polygon_xy_min_max[0], std::max(exploration_costmap_->getOriginX(), std::min(polygon_xy_min_max[2], std::min(exploration_costmap_->getOriginX() + exploration_costmap_->getSizeInMetersX(), wx))));
            wy = std::max(polygon_xy_min_max[1], std::max(exploration_costmap_->getOriginY(), std::min(polygon_xy_min_max[3], std::min(exploration_costmap_->getOriginY() + exploration_costmap_->getSizeInMetersY(), wy))));

            if (!getTracedCells(sx, sy, wx, wy, cell_gatherer, max_length, exploration_costmap_))
            {
                frontier.setArrivalInformation(0.0);
                frontier.setGoalOrientation(0.0);
                return;
            }

            auto info_addition = cell_gatherer.getCells();
            information_along_ray.push_back(info_addition.size());
            // RCLCPP_INFO_STREAM(logger_, "Has hit obstacle? " << cell_gatherer.hasHitObstacle());
            // loop for visualization
            for (size_t counter_info = 0; counter_info < info_addition.size(); counter_info++)
            {
                double wmx, wmy;
                exploration_costmap_->mapToWorld(info_addition[counter_info].x, info_addition[counter_info].y, wmx, wmy);
                geometry_msgs::msg::Pose pnts;
                pnts.position.x = wmx;
                pnts.position.y = wmy;
                vizpoints.push_back(pnts);
            }
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
        // RCLCPP_INFO_STREAM(logger_, COLOR_STR("Total unknown cells is: " + std::to_string(std::accumulate(information_along_ray.begin(), information_along_ray.end(), 0)), logger_.get_name()));

        // visualize raytraced points
        rosVisualizer_->observableCellsViz(vizpoints);
        frontier.setArrivalInformation(maxValue);
        frontier.setGoalOrientation((maxIndex * delta_theta) + (camera_fov / 2));
        return;
    }

    void FrontierCostCalculator::setPlanForFrontier(geometry_msgs::msg::Point start_point_w, Frontier &goal_point_w,
                                                    std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        double information_for_path = 0;
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
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        // goal point
        if (!exploration_costmap_->worldToMap(goal_point_w.getGoalPoint().x, goal_point_w.getGoalPoint().y, mx, my))
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("The goal sent to the planner is off the global costmap Planning will always fail to this goal.", logger_.get_name()));
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
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
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }

        const int &max_cycles = (exploration_costmap_->getSizeInCellsX() >= exploration_costmap_->getSizeInCellsY()) ? (exploration_costmap_->getSizeInCellsX() * 4) : (exploration_costmap_->getSizeInCellsY() * 4);
        int path_len = planner_->calcPath(max_cycles);
        if (path_len == 0)
        {
            RCLCPP_WARN_STREAM(logger_, COLOR_STR("Path length is zero", logger_.get_name()));
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
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
                                                                                                   nodes_in_radius.first, nodes_in_radius.second, map_data->data, 2.0, 1.089, 0.5, Q, true, logger_, exploration_costmap_, true);
                if (info_pcl.first > 0)
                {
                    rosVisualizer_->landmarkViz(info_pcl.second);
                    information_for_path += info_pcl.first;
                }
            }
        }
        fov_marker_publisher_->publish(marker_msg_);
        if (plan.poses.size() == 0)
        {
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        goal_point_w.setPathLength(plan.poses.size());
        if (number_of_wayp == 0)
        {
            goal_point_w.setFisherInformation(information_for_path);
        }
        if (number_of_wayp != 0)
        {
            // This is done so as to normalize the information on the path.
            goal_point_w.setFisherInformation(information_for_path / number_of_wayp);
        }
        rosVisualizer_->frontierPlanViz(plan);
        return;
    }

    void FrontierCostCalculator::getPlanForFrontierRRT(geometry_msgs::msg::Point start_point_w, Frontier &goal_point_w,
                                                       std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        rrt_planner::RRTPlanner planner(exploration_costmap_, 1.0, 1000, logger_);
        std::vector<rrt_planner::Node> path = planner.plan(start_point_w.x, start_point_w.y, goal_point_w.getGoalPoint().x, goal_point_w.getGoalPoint().y);
    }

    void FrontierCostCalculator::setPlanForFrontierEuclidean(geometry_msgs::msg::Point start_point_w, Frontier &goal_point_w,
                                                             std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        auto length_to_frontier = sqrt(pow(start_point_w.x - goal_point_w.getGoalPoint().x, 2) + pow(start_point_w.y - goal_point_w.getGoalPoint().y, 2));
        goal_point_w.setPathLength(length_to_frontier);
        if (length_to_frontier == 0)
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
        goal_point_w.setFisherInformation(0.0);
        return;
    }

    double FrontierCostCalculator::getRandomVal()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        return dis(gen);
    }

    void FrontierCostCalculator::setRandomMetaData(Frontier &goal_point_w)
    {
        goal_point_w.setArrivalInformation(getRandomVal());
        goal_point_w.setGoalOrientation(getRandomVal());
        goal_point_w.setPathLength(getRandomVal());
        goal_point_w.setFisherInformation(getRandomVal());
    }

    void FrontierCostCalculator::setClosestFrontierMetaData(geometry_msgs::msg::Point start_point_w, Frontier &goal_point_w,
                                                            std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        goal_point_w.setArrivalInformation(1e-9);
        goal_point_w.setGoalOrientation(1e-9);
        setPlanForFrontierEuclidean(start_point_w, goal_point_w, map_data, compute_information, planner_allow_unknown_);
        goal_point_w.setFisherInformation(1e-9);
    }

    void FrontierCostCalculator::recomputeNormalizationFactors(Frontier &frontier)
    {
        min_traversable_distance = std::min(min_traversable_distance, frontier.getPathLength());
        max_arrival_info_per_frontier = std::max(max_arrival_info_per_frontier, frontier.getArrivalInformation());
    }
}