// node_graph.cpp

#include "frontier_exploration/planners/FrontierRoadmap.hpp"

namespace frontier_exploration
{
    FrontierRoadMap::FrontierRoadMap(nav2_costmap_2d::Costmap2D *costmap)
        : costmap_(costmap)
    {
        max_connection_length_ = 5.0;
        node_ = rclcpp::Node::make_shared("FRM");
        marker_pub_roadmap_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_roadmap", 10);
        marker_pub_plan_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_roadmap_plan", 10);
        astar_planner_ = std::make_shared<FrontierRoadmapAStar>();

        // Subscriber to handle clicked points
        clicked_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&FrontierRoadMap::clickedPointCallback, this, std::placeholders::_1));
        std::thread t1([this]() {
            rclcpp::spin(node_);
        });
        t1.detach();
        rosViz_ = std::make_shared<RosVisualizer>(node_, costmap_);
    }

    void FrontierRoadMap::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        clicked_points_.push_back(msg->point);

        // If we have two points, calculate the plan
        if (clicked_points_.size() == 2)
        {
            auto p1 = clicked_points_[0];
            auto p2 = clicked_points_[1];

            RCLCPP_INFO(node_->get_logger(), "Calculating plan from (%.2f, %.2f) to (%.2f, %.2f)",
                        p1.x, p1.y, p2.x, p2.y);

            getPlan(p1.x, p1.y, p2.x, p2.y);

            // Reset the clicked points for next input
            clicked_points_.clear();
        }
    }

    void FrontierRoadMap::addNodes(const std::vector<Frontier> &frontiers)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Addition frontier list size:" << frontiers.size());

        populateNodes(frontiers);

        // Add each point as a child of the parent if no obstacle is present
        for (const auto &point : graph_nodes_)
        {
            std::vector<Frontier> closestNodes;
            getNodesWithinRadius(point, closestNodes, 2.8);
            // Ensure the point is added if not already present
            // if (roadmap_.find(point) == roadmap_.end())
            // {
                roadmap_[point] = {};
            // }

            for (auto& closestNode : closestNodes)
            {
                if (isConnectable(closestNode, point))
                {
                    // assigned_children_[point] = 1;
                    roadmap_[point].push_back(closestNode);
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Not connectable");
                }
            }
        }
        publishRoadMap();
    }

    void FrontierRoadMap::getPlan(double xs, double ys, double xe, double ye)
    {
        Frontier start;
        Frontier goal;
        start.setGoalPoint(xs, ys);
        goal.setGoalPoint(xe, ye);
        
        Frontier start_closest;
        Frontier goal_closest;
        getClosestNode(start, start_closest);
        getClosestNode(goal, goal_closest);
        
        auto plan = astar_planner_->getPlan(goal_closest, start_closest, roadmap_);
        publishPlan(plan);
        RCLCPP_WARN(node_->get_logger(), "Plan size: %d", plan.size());
    }

    bool FrontierRoadMap::isConnectable(const Frontier &f1, const Frontier &f2)
    {
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
        // if (distanceBetweenFrontiers(f1, f2) > max_connection_length_)
        //     return false;
        std::vector<nav2_costmap_2d::MapLocation> traced_cells;
        RayTracedCells cell_gatherer(costmap_, traced_cells, 254, 254, 0, 255);
        unsigned int max_length = max_connection_length_ / costmap_->getResolution();
        if (!getTracedCells(f1.getGoalPoint().x, f1.getGoalPoint().y, f2.getGoalPoint().x, f2.getGoalPoint().y, cell_gatherer, max_length, costmap_))
        {
            return false;
        }
        auto allCells = cell_gatherer.getCells();
        if (cell_gatherer.hasHitObstacle())
        {
            return false;
        }
        rosViz_->observableCellsViz(cell_gatherer.getCells());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "ray trace cell size" << cell_gatherer.getCells().size());

        return true;
    }

    const std::unordered_map<Frontier, std::vector<Frontier>, FrontierHash> &FrontierRoadMap::getRoadMap() const
    {
        return roadmap_;
    }

    void FrontierRoadMap::populateNodes(const std::vector<Frontier> &frontiers)
    {
        for (auto &new_frontier : frontiers)
        {
            bool isNew = true;
            for (auto &existing_frontier : graph_nodes_)
            {
                if (distanceBetweenFrontiers(new_frontier, existing_frontier) < 1.0)
                {
                    isNew = false;
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "The frontier at x: " << new_frontier.getGoalPoint().x << " and y: "
                                                                                  << new_frontier.getGoalPoint().y << " is very close. Discarding..");
                    break;
                }
            }
            if (isNew)
                graph_nodes_.push_back(new_frontier);
        }
    }

    void FrontierRoadMap::getNodesWithinRadius(const Frontier &interestNode, std::vector<Frontier> &closestNodeVector, double radius)
    {
        for (auto &existing_frontier : graph_nodes_)
        {
            if (distanceBetweenFrontiers(interestNode, existing_frontier) < radius)
            {
                closestNodeVector.push_back(existing_frontier);
            }
        }
    }

    void FrontierRoadMap::getClosestNode(const Frontier &interestNode, Frontier &closestNode)
    {
        double min_distance = std::numeric_limits<double>::max();
        for (auto &existing_frontier : graph_nodes_)
        {
            double distance = distanceBetweenFrontiers(interestNode, existing_frontier);
            if (distance < min_distance)
            {
                min_distance = distance;
                closestNode = existing_frontier;
            }
        }
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
        node_marker.color.a = 1.0; // Fully opaque
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
        edge_marker.color.a = 1.0;  // Fully opaque
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 0.3;
        edge_marker.color.b = 0.5;

        // Populate the markers
        for (const auto &pair : roadmap_)
        {
            Frontier parent = pair.first;
            geometry_msgs::msg::Point parent_point = parent.getGoalPoint();
            node_marker.points.push_back(parent_point);

            for (const auto &child : pair.second)
            {
                geometry_msgs::msg::Point child_point = child.getGoalPoint();
                node_marker.points.push_back(child_point);

                edge_marker.points.push_back(parent_point);
                edge_marker.points.push_back(child_point);
            }
        }

        // Add the markers to the marker array
        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(edge_marker);

        // Publish the markers
        marker_pub_roadmap_->publish(marker_array);
    }

    void FrontierRoadMap::publishPlan(const std::vector<Node>& plan)
    {
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
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 1.0;

        // Populate the markers
        for (const auto& node : plan)
        {
            geometry_msgs::msg::Point point = node.frontier.getGoalPoint();
            node_marker.points.push_back(point);
        }

        for (size_t i = 0; i < plan.size() - 1; ++i)
        {
            geometry_msgs::msg::Point start_point = plan[i].frontier.getGoalPoint();
            geometry_msgs::msg::Point end_point = plan[i + 1].frontier.getGoalPoint();

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);
        }

        // Add the markers to the marker array
        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(edge_marker);

        // Publish the markers
        marker_pub_plan_->publish(marker_array);
    }
}