// node_graph.cpp

#include "frontier_exploration/planners/FrontierRoadmap.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

std::unique_ptr<frontier_exploration::FrontierRoadMap> frontier_exploration::FrontierRoadMap::frontierRoadmapPtr = nullptr;
std::mutex frontier_exploration::FrontierRoadMap::instanceMutex_;

namespace frontier_exploration
{
    FrontierRoadMap::FrontierRoadMap(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
        : costmap_(explore_costmap_ros->getCostmap()),
          explore_costmap_ros_(explore_costmap_ros)
    {
        max_connection_length_ = RADIUS_TO_DECIDE_EDGES * 1.5;
        node_ = rclcpp::Node::make_shared("frontier_roadmap_node");
        marker_pub_roadmap_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_roadmap", 10);
        marker_pub_roadmap_edges_ = node_->create_publisher<visualization_msgs::msg::Marker>("frontier_roadmap_edges", 10);
        marker_pub_roadmap_vertices_ = node_->create_publisher<visualization_msgs::msg::Marker>("frontier_roadmap_nodes", 10);
        marker_pub_plan_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_roadmap_plan", 10);
        frontier_nav2_plan_ = node_->create_publisher<nav_msgs::msg::Path>("frontier_roadmap_nav2_plan", 10);
        astar_planner_ = std::make_shared<FrontierRoadmapAStar>();

        node_->declare_parameter("max_frontier_distance", rclcpp::ParameterValue(5.12));
        node_->get_parameter("max_frontier_distance", max_frontier_distance_);
        LOG_INFO("Max frontier distance: " << max_frontier_distance_);

        // Subscriber to handle clicked points
        clicked_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&FrontierRoadMap::clickedPointCallback, this, std::placeholders::_1));
        std::thread t1([this]()
                       { rclcpp::spin(node_); });
        t1.detach();
        // rosVisualizerInstance = std::make_shared<RosVisualizer>(node_, costmap_);
    }

    void FrontierRoadMap::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        clicked_points_.push_back(msg->point);

        // If we have two points, calculate the plan
        if (clicked_points_.size() == 2)
        {
            auto p1 = clicked_points_[0];
            auto p2 = clicked_points_[1];

            LOG_INFO("Calculating plan from " << p1.x << ", " << p1.y << " to " << p2.x << p2.y);

            getPlan(p1.x, p1.y, true, p2.x, p2.y, true, true);

            // Reset the clicked points for next input
            clicked_points_.clear();
        }
    }

    std::pair<int, int> FrontierRoadMap::getGridCell(double x, double y)
    {
        int cell_x = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
        int cell_y = static_cast<int>(std::floor(y / GRID_CELL_SIZE));
        return std::make_pair(cell_x, cell_y);
    }

    void FrontierRoadMap::populateNodes(const std::vector<Frontier> &frontiers, bool populateClosest, double min_distance_between_to_add)
    {
        // PROFILE_FUNCTION;
        for (auto &new_frontier : frontiers)
        {
            bool isNew = true;
            auto new_point = new_frontier.getGoalPoint();
            auto grid_cell = getGridCell(new_point.x, new_point.y);
            spatial_hash_map_mutex_.lock();
            // make a cell in the spatial hash map if it does not exist already
            if (spatial_hash_map_.count(grid_cell) == 0)
                spatial_hash_map_[grid_cell] = {};
            spatial_hash_map_mutex_.unlock();
            if (!populateClosest)
            {
                // LOG_TRACE(new_frontier);
                spatial_hash_map_mutex_.lock();
                for (auto &point : spatial_hash_map_[grid_cell])
                {
                    // LOG_TRACE("Existing frontier");
                    // LOG_TRACE(point);
                    if (distanceBetweenFrontiers(new_frontier, point) < 0.1)
                        isNew = false;
                }
                if (isNew)
                    spatial_hash_map_[grid_cell].push_back(new_frontier);
                spatial_hash_map_mutex_.unlock();
                continue;
            }
            // LOG_INFO("New Point x is: " << new_point.x);
            // LOG_INFO("New Point y is: " << new_point.y);
            // LOG_INFO("Grid cell x is: " << grid_cell.first);
            // LOG_INFO("Grid cell y is: " << grid_cell.second);
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
                    // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
                    spatial_hash_map_mutex_.lock();
                    if (spatial_hash_map_.count(neighbor_cell) > 0)
                    {
                        for (const auto &existing_frontier : spatial_hash_map_[neighbor_cell])
                        {
                            // LOG_INFO("Checking frontier at x: " << existing_frontier.getGoalPoint().x << " and y: " << existing_frontier.getGoalPoint().y);
                            if (distanceBetweenFrontiers(new_frontier, existing_frontier) < min_distance_between_to_add)
                            {
                                isNew = false;
                                // LOG_INFO("The frontier at x: " << new_frontier.getGoalPoint().x << " and y: "
                                //                                                                << new_frontier.getGoalPoint().y << " is very close (spatial hash). Discarding..");
                                spatial_hash_map_mutex_.unlock();
                                break;
                            }
                        }
                    }
                    spatial_hash_map_mutex_.unlock();
                    if (!isNew)
                        break;
                }
                if (!isNew)
                    break;
            }
            if (isNew)
            {
                spatial_hash_map_mutex_.lock();
                spatial_hash_map_[grid_cell].push_back(new_frontier);
                if (spatial_hash_map_[grid_cell].size() > 20)
                {
                    throw std::runtime_error("The size is too big");
                }
                spatial_hash_map_mutex_.unlock();
            }
        }
    }

    void FrontierRoadMap::addNodes(const std::vector<Frontier> &frontiers, bool populateClosest)
    {
        eventLoggerInstance.startEvent("addNodes", 2);
        LOG_INFO("Going to add these many frontiers to the spatial hash map:" << frontiers.size());
        eventLoggerInstance.startEvent("populateNodes", 2);
        populateNodes(frontiers, populateClosest, MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES);
        eventLoggerInstance.endEvent("populateNodes", 2);
        eventLoggerInstance.endEvent("addNodes", 2);
    }

    void FrontierRoadMap::addRobotPoseAsNode(geometry_msgs::msg::Pose &start_pose_w, bool populateClosest)
    {
        LOG_HIGHLIGHT("ADDING ROBOT POSE ...");
        Frontier start;
        start.setGoalPoint(start_pose_w.position.x, start_pose_w.position.y);
        start.setUID(generateUID(start));
        std::vector<Frontier> frontier_vec;
        frontier_vec.push_back(start);
        populateNodes(frontier_vec, populateClosest, MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES);
        trailing_robot_poses_.push_back(start_pose_w);
        if(trailing_robot_poses_.size() > 10)
            trailing_robot_poses_.pop_front();
        RosVisualizer::getInstance().visualizeTrailingPoses(trailing_robot_poses_);
    }

    void FrontierRoadMap::constructNewEdges(const std::vector<Frontier> &frontiers)
    {
        LOG_INFO("Reconstructing new frontier edges");
        // Add each point as a child of the parent if no obstacle is present
        spatial_hash_map_mutex_.lock();
        for (const auto &point : frontiers)
        {
            // Ensure the point is added if not already present
            if (roadmap_.find(point) == roadmap_.end())
            {
                roadmap_mutex_.lock();
                roadmap_[point] = {};
                roadmap_mutex_.unlock();
            }
            std::vector<Frontier> closestNodes;
            getNodesWithinRadius(point, closestNodes, RADIUS_TO_DECIDE_EDGES);
            int numChildren = 0;
            for (auto &closestNode : closestNodes)
            {
                if (closestNode == point)
                    continue;
                if (isConnectable(closestNode, point))
                {
                    roadmap_mutex_.lock();
                    // initialize the closest node memory to add children.
                    if (roadmap_.find(closestNode) == roadmap_.end())
                        roadmap_[closestNode] = {};
                    // add if only it does not already exist.
                    auto addition1 = roadmap_[point];
                    auto addition2 = roadmap_[closestNode];
                    if (std::find(addition1.begin(), addition1.end(), closestNode) == addition1.end())
                        roadmap_[point].push_back(closestNode);
                    if (std::find(addition2.begin(), addition2.end(), point) == addition2.end())
                        roadmap_[closestNode].push_back(point);
                    roadmap_mutex_.unlock();
                    numChildren++;
                }
                else
                {
                    // LOG_INFO("Not connectable");
                }
            }
            LOG_TRACE("Clearing roadmap and constructing new edges for: " << point << " with " << numChildren << " children");
        }
        // roadmap_mutex_.lock();
        // roadmap_.clear();
        // roadmap_mutex_.unlock();
        spatial_hash_map_mutex_.unlock();
    }

    void FrontierRoadMap::constructNewEdgeRobotPose(const geometry_msgs::msg::Pose &rPose)
    {
        LOG_INFO("Reconstructing new robot pose edges");
        Frontier rPoseF;
        rPoseF.setGoalPoint(rPose.position.x, rPose.position.y);
        rPoseF.setUID(generateUID(rPoseF));
        std::vector<Frontier> toAdd;
        toAdd.push_back(rPoseF);
        constructNewEdges(toAdd);
    }

    void FrontierRoadMap::reConstructGraph()
    {
        LOG_INFO("Reconstructing entire graph within radius: " << max_frontier_distance_);
        geometry_msgs::msg::PoseStamped robotPose;
        explore_costmap_ros_->getRobotPose(robotPose);
        // Add each point as a child of the parent if no obstacle is present
        spatial_hash_map_mutex_.lock();
        roadmap_.clear();
        for (const auto &pair : spatial_hash_map_)
        {
            if (sqrt(pow(robotPose.pose.position.x - pair.first.first, 2) + pow(robotPose.pose.position.y - pair.first.second, 2)) > max_frontier_distance_)
            {
                LOG_DEBUG("Skipping x: " << pair.first.first << ", y: " << pair.first.second << " from roadmap");
                continue;
            }
            for (const auto &point : pair.second)
            {
                roadmap_mutex_.lock();
                roadmap_[point] = {};
                roadmap_mutex_.unlock();
                // }
                std::vector<Frontier> closestNodes;
                getNodesWithinRadius(point, closestNodes, RADIUS_TO_DECIDE_EDGES);
                // Ensure the point is added if not already present
                // if (roadmap_.find(point) == roadmap_.end())
                // {

                for (auto &closestNode : closestNodes)
                {
                    if (closestNode == point)
                        continue;
                    if (isConnectable(closestNode, point))
                    {
                        roadmap_mutex_.lock();
                        auto addition1 = roadmap_[point];
                        if (std::find(addition1.begin(), addition1.end(), closestNode) == addition1.end())
                            roadmap_[point].push_back(closestNode);
                        roadmap_mutex_.unlock();
                    }
                    else
                    {
                        // LOG_INFO("Not connectable");
                    }
                }
            }
        }
        // roadmap_mutex_.lock();
        // roadmap_.clear();
        // roadmap_mutex_.unlock();
        spatial_hash_map_mutex_.unlock();
    }

    void FrontierRoadMap::getNodesWithinRadius(const Frontier &interestNode, std::vector<Frontier> &closestNodeVector, const double radius)
    {
        // PROFILE_FUNCTION;
        // Get the central grid cell of the interest node
        auto interest_point = interestNode.getGoalPoint();
        auto center_cell = getGridCell(interest_point.x, interest_point.y);

        // Calculate the number of grid cells to check in each direction (radius in cells)
        int cell_radius = static_cast<int>(std::ceil(radius / GRID_CELL_SIZE));
        for (int dx = -cell_radius; dx <= cell_radius; ++dx)
        {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy)
            {
                auto neighbor_cell = std::make_pair(center_cell.first + dx, center_cell.second + dy);
                // The spatial hash map is not protected with the mutex here because the function itself is protected at the caller line.
                if (spatial_hash_map_.count(neighbor_cell) > 0)
                {
                    for (const auto &existing_frontier : spatial_hash_map_[neighbor_cell])
                    {
                        if (distanceBetweenFrontiers(interestNode, existing_frontier) < radius)
                        {
                            closestNodeVector.push_back(existing_frontier);
                        }
                    }
                }
            }
        }
    }

    void FrontierRoadMap::getNodesWithinRadius(const geometry_msgs::msg::Point &interestPoint, std::vector<Frontier> &closestNodeVector, const double radius)
    {
        // PROFILE_FUNCTION;
        // Get the central grid cell of the interest node
        auto center_cell = getGridCell(interestPoint.x, interestPoint.y);

        // Calculate the number of grid cells to check in each direction (radius in cells)
        int cell_radius = static_cast<int>(std::ceil(radius / GRID_CELL_SIZE));
        for (int dx = -cell_radius; dx <= cell_radius; ++dx)
        {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy)
            {
                auto neighbor_cell = std::make_pair(center_cell.first + dx, center_cell.second + dy);
                // The spatial hash map is not protected with the mutex here because the function itself is protected at the caller line.
                if (spatial_hash_map_.count(neighbor_cell) > 0)
                {
                    for (const auto &existing_frontier : spatial_hash_map_[neighbor_cell])
                    {
                        if (distanceBetweenPoints(interestPoint, existing_frontier.getGoalPoint()) < radius)
                        {
                            closestNodeVector.push_back(existing_frontier);
                        }
                    }
                }
            }
        }
    }

    void FrontierRoadMap::getClosestNodeInHashmap(const Frontier &interestNode, Frontier &closestNode)
    {
        auto grid_cell = getGridCell(interestNode.getGoalPoint().x, interestNode.getGoalPoint().y);
        double min_distance = std::numeric_limits<double>::max();
        bool foundClosestNode = false;
        int searchRadiusMultiplier = 1;
        while (!foundClosestNode)
        {
            int searchRadius = GRID_CELL_SIZE * searchRadiusMultiplier;
            if (searchRadius > 10)
                LOG_CRITICAL("Cannot find closest node within 10m. This is not ok.")
            for (int dx = -searchRadius; dx <= searchRadius; ++dx)
            {
                for (int dy = -searchRadius; dy <= searchRadius; ++dy)
                {
                    auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
                    // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
                    spatial_hash_map_mutex_.lock();
                    if (spatial_hash_map_.count(neighbor_cell) > 0)
                    {
                        for (const auto &existing_frontier : spatial_hash_map_[neighbor_cell])
                        {
                            double distance = distanceBetweenFrontiers(interestNode, existing_frontier);
                            if (distance < min_distance)
                            {
                                foundClosestNode = true;
                                min_distance = distance;
                                closestNode = existing_frontier;
                            }
                        }
                    }
                    spatial_hash_map_mutex_.unlock();
                }
            }
            ++searchRadiusMultiplier;
        }
    }

    void FrontierRoadMap::getClosestNodeInRoadMap(const Frontier &interestNode, Frontier &closestNode)
    {
        auto grid_cell = getGridCell(interestNode.getGoalPoint().x, interestNode.getGoalPoint().y);
        double min_distance = std::numeric_limits<double>::max();
        bool foundClosestNode = false;
        int searchRadiusMultiplier = 1;
        while (!foundClosestNode)
        {
            int searchRadius = GRID_CELL_SIZE * searchRadiusMultiplier;
            if (searchRadius > 10)
                LOG_CRITICAL("Cannot find closest node within 10m. This is not ok.")
            for (int dx = -searchRadius; dx <= searchRadius; ++dx)
            {
                for (int dy = -searchRadius; dy <= searchRadius; ++dy)
                {
                    auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
                    // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
                    spatial_hash_map_mutex_.lock();
                    if (spatial_hash_map_.count(neighbor_cell) > 0)
                    {
                        for (const auto &existing_frontier : spatial_hash_map_[neighbor_cell])
                        {
                            roadmap_mutex_.lock();
                            if (roadmap_.count(existing_frontier) > 0)
                            {
                                double distance = distanceBetweenFrontiers(interestNode, existing_frontier);
                                if (distance < min_distance)
                                {
                                    foundClosestNode = true;
                                    min_distance = distance;
                                    closestNode = existing_frontier;
                                }
                            }
                            roadmap_mutex_.unlock();
                        }
                    }
                    spatial_hash_map_mutex_.unlock();
                }
            }
            ++searchRadiusMultiplier;
        }
    }

    RoadmapPlanResult FrontierRoadMap::getPlan(double xs, double ys, bool useClosestToStart, double xe, double ye, bool useClosestToEnd, bool publish_plan)
    {
        RoadmapPlanResult planResult;
        if (xs == xe && ys == ye)
        {
            planResult.path_exists = true;
            planResult.path_length_m = 0.0;
            planResult.path = std::vector<std::shared_ptr<Node>>();
            return planResult;
        }
        Frontier start;
        start.setGoalPoint(xs, ys);
        start.setUID(generateUID(start));
        Frontier start_closest;
        {
            if (!useClosestToStart)
            {
                if (roadmap_.count(start) == 0)
                {
                    std::vector<Frontier> frontier_vec;
                    frontier_vec.push_back(start);
                    addNodes(frontier_vec, false);
                    start_closest = start;
                }
                else
                {
                    LOG_INFO("Start node: " << xs << ", " << ys << " already in the roadmap");
                }
            }
            else
            {
                getClosestNodeInRoadMap(start, start_closest);
            }
        }

        Frontier goal;
        goal.setGoalPoint(xe, ye);
        goal.setUID(generateUID(goal));
        Frontier goal_closest;
        {
            if (!useClosestToEnd)
            {
                if (roadmap_.count(goal) == 0)
                {
                    std::vector<Frontier> frontier_vec;
                    frontier_vec.push_back(goal);
                    addNodes(frontier_vec, false);
                    goal_closest = goal;
                }
                else
                {
                    LOG_INFO("End node: " << xs << ", " << ys << " already in the roadmap");
                }
            }
            else
            {
                getClosestNodeInRoadMap(goal, goal_closest);
            }
        }

        roadmap_mutex_.lock();
        LOG_DEBUG("Getting path from: " << start << " to " << goal);
        LOG_DEBUG("In turn calculating from:" << start_closest << " to " << goal_closest);
        auto aStarResult = astar_planner_->getPlan(start_closest, goal_closest, roadmap_);
        planResult.path = aStarResult.first;
        planResult.path_length_m = aStarResult.second;
        planResult.path_exists = true;
        if (planResult.path.size() == 0)
        {
            LOG_WARN("Could not compute path from: " << xs << ", " << ys << " to " << xe << ", " << ye);
            planResult.path_exists = false;
            roadmap_mutex_.unlock();
            return planResult;
        }
        // LOG_INFO("Plan size: %d", plan.size());
        // rclcpp::sleep_for(std::chrono::seconds(1));
        roadmap_mutex_.unlock();
        LOG_TRACE("Path information:");
        for (auto &fullPoint : planResult.path)
        {
            LOG_TRACE(fullPoint->frontier << " , ");
        }
        LOG_TRACE("Path length in m: " << planResult.path_length_m);
        if(publish_plan)
            publishPlan(planResult.path, 1.0, 0.0, 0.0);
        return planResult;
    };

    RoadmapPlanResult FrontierRoadMap::getPlan(Frontier &startNode, bool useClosestToStart, Frontier &endNode, bool useClosestToEnd)
    {
        return getPlan(startNode.getGoalPoint().x, startNode.getGoalPoint().y, useClosestToStart, endNode.getGoalPoint().x, endNode.getGoalPoint().y, useClosestToEnd, false);
        // Frontier start;
        // Frontier goal;
        // start.setGoalPoint(xs, ys);
        // goal.setGoalPoint(xe, ye);

        // Frontier start_closest;
        // Frontier goal_closest;
        // getClosestNodeInHashmap(start, start_closest);
        // getClosestNodeInHashmap(goal, goal_closest);

        // roadmap_mutex_.lock();
        // auto plan = astar_planner_->getPlan(goal_closest, start_closest, roadmap_);
        // LOG_INFO("Plan size: %d", plan.size());
        // roadmap_mutex_.unlock();
        // publishPlan(plan);
    }

    std::vector<Frontier> FrontierRoadMap::refinePath(RoadmapPlanResult& planResult)
    {
        std::vector<Frontier> resultingPath;

        // Debug: Check the size of the path
        std::cout << "Starting path refinement. PlanResult path size: " << planResult.path.size() << std::endl;
        
        if (planResult.path.empty()) {
            std::cerr << "Error: planResult.path is empty!" << std::endl;
            return resultingPath;
        }

        resultingPath.push_back(planResult.path[0]->frontier);
        std::cout << "Added first frontier to resultingPath." << std::endl;

        for (int kk = 0; kk < planResult.path.size() - 1;)
        {
            Frontier latestFrontier = planResult.path[kk]->frontier;
            std::cout << "Processing frontier at index " << kk << std::endl;

            int next = kk + 1;
            
            while (next < planResult.path.size())
            {
                std::cout << "Checking connection from index " << kk << " to " << next << std::endl;
                if (isConnectable(planResult.path[kk]->frontier, planResult.path[next]->frontier))
                {
                    std::cout << "Frontiers at index " << kk << " and " << next << " are connectable." << std::endl;
                    latestFrontier = planResult.path[next]->frontier;
                    next++;
                }
                else
                {
                    std::cout << "Frontiers at index " << kk << " and " << next << " are not connectable. Breaking out of the loop." << std::endl;
                    break;
                }
            }

            // Debug: Check if the frontier changed after connection checks
            if (!(latestFrontier == planResult.path[kk]->frontier))
            {
                std::cout << "Adding frontier from index " << next - 1 << " to resultingPath." << std::endl;
                resultingPath.push_back(latestFrontier);
            }
            else
            {
                std::cout << "No connectable frontier found from index " << kk << std::endl;
                return resultingPath;
            }

            kk = next - 1;  // Move to the last connected index
            std::cout << "Moving kk to " << kk << std::endl;
        }

        std::cout << "Path refinement complete. Resulting path size: " << resultingPath.size() << std::endl;
        return resultingPath;
    }

    bool FrontierRoadMap::isConnectable(const Frontier &f1, const Frontier &f2)
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
        std::vector<nav2_costmap_2d::MapLocation> traced_cells;
        RayTracedCells cell_gatherer(costmap_, traced_cells, 253, 254, 0, 255);
        unsigned int max_length = max_connection_length_ / costmap_->getResolution();
        if (!getTracedCells(f1.getGoalPoint().x, f1.getGoalPoint().y, f2.getGoalPoint().x, f2.getGoalPoint().y, cell_gatherer, max_length, costmap_))
        {
            return false;
        }
        if (cell_gatherer.hasHitObstacle())
        {
            return false;
        }
        // RosVisualizer::getInstance()observableCellsViz(cell_gatherer.getCells());
        // LOG_INFO("ray trace cell size" << cell_gatherer.getCells().size());

        return true;
    }

    void FrontierRoadMap::publishRoadMap()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Marker for nodes
        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = rclcpp::Clock().now();
        node_marker.ns = "nodes";
        node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::msg::Marker::ADD;
        node_marker.pose.orientation.w = 1.0;
        node_marker.scale.x = 0.2; // Size of each sphere
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.a = 0.30; // 1.0 - Fully opaque
        node_marker.color.r = 0.0;
        node_marker.color.g = 1.0;
        node_marker.color.b = 0.0;

        // Marker for edges
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = rclcpp::Clock().now();
        edge_marker.ns = "edges";
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.02; // Thickness of the lines
        edge_marker.color.a = 0.18; // 1.0 - Fully opaque
        edge_marker.color.r = 0.3;  // Red channel
        edge_marker.color.g = 0.7;  // Green channel
        edge_marker.color.b = 1.0;  // Blue channel

        LOG_TRACE("Roadmap points: ")
        // Populate the markers
        for (const auto &pair : roadmap_)
        {
            LOG_TRACE("parent:")
            Frontier parent = pair.first;
            LOG_TRACE(parent);
            geometry_msgs::msg::Point parent_point = parent.getGoalPoint();
            parent_point.z = 0.15;
            node_marker.points.push_back(parent_point);

            for (const auto &child : pair.second)
            {
                LOG_TRACE("Children:")
                LOG_TRACE(child);
                geometry_msgs::msg::Point child_point = child.getGoalPoint();
                child_point.z = 0.15;
                node_marker.points.push_back(child_point);

                edge_marker.points.push_back(parent_point);
                edge_marker.points.push_back(child_point);
            }
            LOG_TRACE("=======")
        }
        LOG_TRACE("***********Roadmap points end: ")

        // Add the markers to the marker array
        marker_pub_roadmap_vertices_->publish(node_marker);
        marker_array.markers.push_back(node_marker);
        marker_pub_roadmap_edges_->publish(edge_marker);
        marker_array.markers.push_back(edge_marker);

        // Publish the markers
        marker_pub_roadmap_->publish(marker_array);
    }

    void FrontierRoadMap::publishPlan(const std::vector<std::shared_ptr<Node>> &plan, float r, float g, float b)
    {
        if (plan.size() == 0)
            return;
        // Create a MarkerArray to hold the markers for the plan
        visualization_msgs::msg::MarkerArray marker_array;

        // Marker for nodes
        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = rclcpp::Clock().now();
        node_marker.ns = "plan_nodes";
        node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::msg::Marker::ADD;
        node_marker.pose.orientation.w = 1.0;
        node_marker.scale.x = 0.2; // Size of each sphere
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.a = 1.0; // Fully opaque
        node_marker.color.r = r;
        node_marker.color.g = g;
        node_marker.color.b = b; // Blue color for plan nodes

        // Marker for edges
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = rclcpp::Clock().now();
        edge_marker.ns = "plan_edges";
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.15; // Thickness of the lines
        edge_marker.color.a = 1.0;  // Fully opaque
        edge_marker.color.r = r;
        edge_marker.color.g = g;
        edge_marker.color.b = b;

        // Populate the markers
        for (const auto &node : plan)
        {
            geometry_msgs::msg::Point point = node->frontier.getGoalPoint();
            node_marker.points.push_back(point);
        }

        for (size_t i = 0; i < plan.size() - 1; ++i)
        {
            // LOG_TRACE("New start point in plan: " << plan[i]->frontier.getGoalPoint().x << ", " << plan[i]->frontier.getGoalPoint().y);
            // LOG_TRACE("New end point in plan: " << plan[i + 1]->frontier.getGoalPoint().x << ", " << plan[i + 1]->frontier.getGoalPoint().y);
            geometry_msgs::msg::Point start_point = plan[i]->frontier.getGoalPoint();
            geometry_msgs::msg::Point end_point = plan[i + 1]->frontier.getGoalPoint();

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);
        }

        // Add the markers to the marker array
        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(edge_marker);

        // Publish the markers
        marker_pub_plan_->publish(marker_array);
    }

    void FrontierRoadMap::publishPlan(const std::vector<Frontier> &plan, std::string planType)
    {
        if (plan.size() == 0)
            return;
        // Create a MarkerArray to hold the markers for the plan
        visualization_msgs::msg::MarkerArray marker_array;
        nav_msgs::msg::Path frontier_nav2_plan_msg;
        frontier_nav2_plan_msg.header.frame_id = "map";
        frontier_nav2_plan_msg.header.stamp = rclcpp::Clock().now();

        // Marker for nodes
        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = rclcpp::Clock().now();
        node_marker.ns = "plan_nodes";
        node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::msg::Marker::ADD;
        node_marker.pose.orientation.w = 1.0;
        node_marker.scale.x = 0.2; // Size of each sphere
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.a = 1.0; // Fully opaque
        node_marker.color.r = 0.0;
        node_marker.color.g = 0.0;
        node_marker.color.b = 1.0; // Blue color for plan nodes

        // Marker for edges
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = rclcpp::Clock().now();
        edge_marker.ns = "plan_edges";
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.scale.x = 0.15; // Thickness of the lines
        edge_marker.color.a = 1.0;  // Fully opaque
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 0.0;

        // Populate the markers
        for (const auto &node : plan)
        {
            geometry_msgs::msg::Point point = node.getGoalPoint();
            node_marker.points.push_back(point);
            geometry_msgs::msg::PoseStamped path_pose;
            path_pose.pose.position = point;
            frontier_nav2_plan_msg.poses.push_back(path_pose);
        }

        for (size_t i = 0; i < plan.size() - 1; ++i)
        {
            geometry_msgs::msg::Point start_point = plan[i].getGoalPoint();
            geometry_msgs::msg::Point end_point = plan[i + 1].getGoalPoint();

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);
        }

        // Add the markers to the marker array
        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(edge_marker);

        // Publish the markers
        if(planType == "fullTSP")
            marker_pub_plan_->publish(marker_array);
        else if(planType == "refinedPath")
            frontier_nav2_plan_->publish(frontier_nav2_plan_msg);
        else
            throw std::runtime_error("Unknown plan type");
    }

    bool FrontierRoadMap::isPointBlacklisted(const Frontier& point)
    {
        for (auto& blacklist : blacklisted_frontiers_)
        {
            if(distanceBetweenFrontiers(point, blacklist) <= 3.0)
            {
                LOG_DEBUG("Skipping " << point << " from roadmap. In blacklist.");
                return true;
            }
        }
        return false;
    }
}