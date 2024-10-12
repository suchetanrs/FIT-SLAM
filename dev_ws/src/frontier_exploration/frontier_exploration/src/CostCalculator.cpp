#include "frontier_exploration/CostCalculator.hpp"
#include <frontier_exploration/util/util.hpp>

namespace frontier_exploration
{
    FrontierCostCalculator::FrontierCostCalculator(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    {
        node_ = node;
        exploration_costmap_ = explore_costmap_ros->getCostmap();
        logger_ = node_->get_logger();
        // rosVisualizerInstance = std::make_shared<RosVisualizer>(node, exploration_costmap_);
        min_traversable_distance = std::numeric_limits<double>::max();
        max_traversable_distance = 0.0;
        min_arrival_info_per_frontier = std::numeric_limits<double>::max();
        max_arrival_info_per_frontier = 0.0;

        fov_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("path_fovs", 10);
        robot_radius_ = explore_costmap_ros->getRobotRadius();
        max_arrival_info_gt_ = setMaxArrivalInformation();
        LOG_WARN("Max arrival cost GT: " << max_arrival_info_gt_);
        // wait for costmap to be current before computing the above. Hence hardcoded.
        max_arrival_info_gt_ = 85.0;
        LOG_WARN("Max arrival cost GT: " << max_arrival_info_gt_);
        min_arrival_info_gt_ = 0.70 * max_arrival_info_gt_;

    }

    void FrontierCostCalculator::setArrivalInformationForFrontier(Frontier &frontier, std::vector<double> &polygon_xy_min_max)
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));

        double sx, sy; // sensor x, sensor y, sensor orientation
        double wx, wy;
        unsigned int max_length = MAX_CAMERA_DEPTH / (exploration_costmap_->getResolution());
        sx = frontier.getGoalPoint().x;
        sy = frontier.getGoalPoint().y;
        std::vector<int> information_along_ray; // stores the information along each ray in 2PI.
        std::vector<geometry_msgs::msg::Pose> vizpoints;
        // Iterate through each angle in 2PI with DELTA_THETA resolution
        double maxHitObstacles = 2 * M_PI / DELTA_THETA;
        float hitObstacleCount = 0;
        for (double theta = 0; theta <= (2 * M_PI); theta += DELTA_THETA)
        {
            std::vector<nav2_costmap_2d::MapLocation> traced_cells;
            // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
            RayTracedCells cell_gatherer(exploration_costmap_, traced_cells, 240, 254, 255, 255);

            wx = sx + (MAX_CAMERA_DEPTH * cos(theta));
            wy = sy + (MAX_CAMERA_DEPTH * sin(theta));

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
            if (cell_gatherer.hasHitObstacle())
            {
                ++hitObstacleCount;
            }
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

        unsigned int sxm, sym;
        exploration_costmap_->worldToMap(sx, sy, sxm, sym);
        double footprintInLethalPenalty = isRobotFootprintInLethal(exploration_costmap_, sxm, sym, std::ceil(robot_radius_ / exploration_costmap_->getResolution()));
        if (1.0 - footprintInLethalPenalty == 0.0 && frontier.getSize() < 10.0)
        {
            frontier.setAchievability(false);
        }
        // std::cout << "maxHitObstacles" << maxHitObstacles << std::endl;
        // std::cout << "hitObstacleCount" << hitObstacleCount << std::endl;
        // std::cout << "footprintInLethalPenalty" << footprintInLethalPenalty << std::endl;

        std::vector<int> kernel(static_cast<int>(CAMERA_FOV / DELTA_THETA), 1); // initialize a kernal vector of size 6 and all elements = 1
        int n = information_along_ray.size();                                   // number of rays computed in 2PI
        int k = kernel.size();
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
        LOG_DEBUG("Total unknown cells is: " + std::to_string(std::accumulate(information_along_ray.begin(), information_along_ray.end(), 0)));

        // visualize raytraced points
        // RosVisualizer::getInstance()observableCellsViz(vizpoints);
        frontier.setArrivalInformation(maxValue);
        LOG_DEBUG("Arrival information is: " << frontier.getArrivalInformation());
        if(frontier.getArrivalInformation() < min_arrival_info_gt_)
        {
            frontier.setAchievability(false);
        }
        frontier.setGoalOrientation((maxIndex * DELTA_THETA) + (CAMERA_FOV / 2));
        return;
    }

    double FrontierCostCalculator::setMaxArrivalInformation()
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));

        double sx, sy; // sensor x, sensor y, sensor orientation
        double wx, wy;
        unsigned int max_length = MAX_CAMERA_DEPTH / (exploration_costmap_->getResolution());
        sx = 0.0;
        sy = 0.0;
        std::vector<int> information_along_ray; // stores the information along each ray in 2PI.
        std::vector<geometry_msgs::msg::Pose> vizpoints;
        // Iterate through each angle in 2PI with DELTA_THETA resolution
        double maxHitObstacles = 2 * M_PI / DELTA_THETA;
        for (double theta = 0; theta <= (2 * M_PI); theta += DELTA_THETA)
        {
            std::vector<nav2_costmap_2d::MapLocation> traced_cells;
            // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
            RayTracedCells cell_gatherer(exploration_costmap_, traced_cells, 260, 260, 0, 255);

            wx = sx + (MAX_CAMERA_DEPTH * cos(theta));
            wy = sy + (MAX_CAMERA_DEPTH * sin(theta));

            if (!getTracedCells(sx, sy, wx, wy, cell_gatherer, max_length, exploration_costmap_))
            {
                return 0;
            }

            auto info_addition = cell_gatherer.getCells();
            information_along_ray.push_back(info_addition.size());
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

        std::vector<int> kernel(static_cast<int>(CAMERA_FOV / DELTA_THETA), 1); // initialize a kernal vector of size 6 and all elements = 1
        int n = information_along_ray.size();                                   // number of rays computed in 2PI
        int k = kernel.size();
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
        return maxValue;
    }

    void FrontierCostCalculator::setPlanForFrontier(geometry_msgs::msg::Pose start_pose_w, Frontier &goal_point_w,
                                                    std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        if (goal_point_w.isAchievable() == false)
        {
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        auto start_point_w = start_pose_w.position;
        double information_for_path = 0;
        visualization_msgs::msg::Marker marker_msg_;

        // calculate heading difference
        auto robot_yaw = quatToEuler(start_pose_w.orientation)[2];
        if (robot_yaw < 0)
            robot_yaw = robot_yaw + (M_PI * 2);
        double goal_yaw = atan2(goal_point_w.getGoalPoint().y - start_pose_w.position.y, goal_point_w.getGoalPoint().x - start_pose_w.position.x);
        if (goal_yaw < 0)
            goal_yaw = goal_yaw + (M_PI * 2);
        double path_heading = abs(robot_yaw - goal_yaw);
        if (path_heading > M_PI)
            path_heading = (2 * M_PI) - path_heading;

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
            LOG_ERROR("Cannot create a plan: the robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        // goal point
        if (!exploration_costmap_->worldToMap(goal_point_w.getGoalPoint().x, goal_point_w.getGoalPoint().y, mx, my))
        {
            LOG_ERROR("The goal sent to the planner is off the global costmap Planning will always fail to this goal.");
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
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
            LOG_WARN("Plan not Found for frontier at x: " << goal_point_w.getGoalPoint().x << " y: " << goal_point_w.getGoalPoint().y);
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }

        const int &max_cycles = (exploration_costmap_->getSizeInCellsX() >= exploration_costmap_->getSizeInCellsY()) ? (exploration_costmap_->getSizeInCellsX() * 4) : (exploration_costmap_->getSizeInCellsY() * 4);
        int path_len = planner_->calcPath(max_cycles);
        if (path_len == 0)
        {
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
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
        double path_length_m = 0;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> previous_pose = nullptr;
        LOG_TRACE("Compute information? << " << compute_information);
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
            if (i != 0 && previous_pose != nullptr)
            {
                path_length_m += distanceBetweenPoints(pose_from.pose.position, previous_pose->pose.position);
            }
            previous_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_from);
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
                frontier_exploration::getRelativePoseGivenTwoPoints(pose_from.pose.position, pose_to.pose.position, oriented_pose);
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
                    // RosVisualizer::getInstance()landmarkViz(info_pcl.second);
                    information_for_path += info_pcl.first;
                }
            }
        }
        fov_marker_publisher_->publish(marker_msg_);
        if (plan.poses.size() == 0)
        {
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        goal_point_w.setAchievability(true);
        goal_point_w.setPathLength(plan.poses.size());
        goal_point_w.setPathLengthInM(path_length_m);
        goal_point_w.setPathHeading(path_heading);
        if (number_of_wayp == 0)
        {
            goal_point_w.setFisherInformation(information_for_path);
        }
        if (number_of_wayp != 0)
        {
            // This is done so as to normalize the information on the path.
            goal_point_w.setFisherInformation(information_for_path / number_of_wayp);
        }
        // RosVisualizer::getInstance()frontierPlanViz(plan);
        return;
    }

    void FrontierCostCalculator::setPlanForFrontierRoadmap(geometry_msgs::msg::Pose start_pose_w, Frontier &goal_point_w,
                                                           std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(600));
        PROFILE_FUNCTION;
        if (goal_point_w.isAchievable() == false)
        {
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }
        auto path_length = FrontierRoadMap::getInstance().getPlan(start_pose_w.position.x, start_pose_w.position.y, true, goal_point_w.getGoalPoint().x, goal_point_w.getGoalPoint().y, true, false);
        // TODO: Set correct f_info
        if (path_length.path_exists == false)
        {
            goal_point_w.setAchievability(false);
            goal_point_w.setPathLength(std::numeric_limits<double>::max());
            goal_point_w.setPathLengthInM(std::numeric_limits<double>::max());
            goal_point_w.setPathHeading(std::numeric_limits<double>::max());
            goal_point_w.setFisherInformation(0);
            return;
        }

        goal_point_w.setAchievability(true);
        // calculate heading difference
        auto robot_yaw = quatToEuler(start_pose_w.orientation)[2];
        if (robot_yaw < 0)
            robot_yaw = robot_yaw + (M_PI * 2);
        double goal_yaw = atan2(goal_point_w.getGoalPoint().y - start_pose_w.position.y, goal_point_w.getGoalPoint().x - start_pose_w.position.x);
        if (goal_yaw < 0)
            goal_yaw = goal_yaw + (M_PI * 2);
        double path_heading = abs(robot_yaw - goal_yaw);
        if (path_heading > M_PI)
            path_heading = (2 * M_PI) - path_heading;
        
        goal_point_w.setPathHeading(path_heading);
        goal_point_w.setPathLength(path_length.path_length_m);
        goal_point_w.setPathLengthInM(path_length.path_length_m);
        goal_point_w.setFisherInformation(0.0);
        return;
    }

    void FrontierCostCalculator::updateRoadmapData(geometry_msgs::msg::Pose &start_pose_w, std::vector<Frontier> &frontiers)
    {
        // PROFILE_FUNCTION;
    }

    void FrontierCostCalculator::setPlanForFrontierEuclidean(geometry_msgs::msg::Point start_point_w, Frontier &goal_point_w,
                                                             std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data, bool compute_information, bool planner_allow_unknown_)
    {
        auto length_to_frontier = sqrt(pow(start_point_w.x - goal_point_w.getGoalPoint().x, 2) + pow(start_point_w.y - goal_point_w.getGoalPoint().y, 2));
        goal_point_w.setPathLength(length_to_frontier);
        if (length_to_frontier == 0)
            goal_point_w.setAchievability(false);
        goal_point_w.setPathLength(std::numeric_limits<double>::max());
        goal_point_w.setPathHeading(std::numeric_limits<double>::max());
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
        if (!frontier.isAchievable())
            return;
        min_traversable_distance = std::min(min_traversable_distance, frontier.getPathLength());
        max_traversable_distance = std::max(max_traversable_distance, frontier.getPathLength());
        min_arrival_info_per_frontier = std::min(min_arrival_info_per_frontier, frontier.getArrivalInformation());
        max_arrival_info_per_frontier = std::max(max_arrival_info_per_frontier, frontier.getArrivalInformation());
    }
}