#include <vector>
#include <algorithm>
#include <limits>
#include "frontier_exploration/FullPathOptimizer.hpp"
namespace frontier_exploration
{
    FullPathOptimizer::FullPathOptimizer(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
        : node_(node)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("top_frontiers", 10);
        local_search_area_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("local_search_area", 10);
    }

    std::tuple<std::vector<Frontier>, std::vector<Frontier>> FullPathOptimizer::getFilteredFrontiers(std::vector<Frontier> &frontier_list, size_t n)
    {
        // Filter frontiers with arrival information > 70
        std::vector<Frontier> filtered_frontiers;
        for (const auto &frontier : frontier_list)
        {
            if (frontier.isAchievable())
            {
                filtered_frontiers.push_back(frontier);
            }
        }

        // Sort the filtered frontiers based on path length (closest first)
        std::sort(filtered_frontiers.begin(), filtered_frontiers.end(), [](const Frontier &a, const Frontier &b)
                  { return a.getPathLength() < b.getPathLength(); });

        // If there are fewer than n frontiers, return all of them
        if (filtered_frontiers.size() <= n)
        {
            return std::make_tuple(filtered_frontiers, std::vector<Frontier>());
        }

        // Otherwise, return the first n frontiers
        return std::make_tuple(std::vector<Frontier>(filtered_frontiers.begin(), filtered_frontiers.begin() + n),
                               std::vector<Frontier>(filtered_frontiers.begin() + n, filtered_frontiers.end()));
    }

    void FullPathOptimizer::addToMarkerArray(visualization_msgs::msg::MarkerArray &marker_array, std::vector<Frontier> &frontier_list,
                                             std::string ns, float r, float g, float b, int id)
    {
        // Initialize min and max values for x and y
        double min_x = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_y = -std::numeric_limits<double>::max();

        // Iterate through the filtered frontiers to find min and max x, y values
        for (const auto &frontier : frontier_list)
        {
            geometry_msgs::msg::Point point = frontier.getGoalPoint();
            if (point.x <= min_x)
                min_x = point.x;
            if (point.x >= max_x)
                max_x = point.x;
            if (point.y <= min_y)
                min_y = point.y;
            if (point.y >= max_y)
                max_y = point.y;
        }

        // Create a Marker for the polygon
        visualization_msgs::msg::Marker polygon_marker;
        polygon_marker.header.frame_id = "map";
        polygon_marker.header.stamp = rclcpp::Clock().now();
        polygon_marker.ns = ns;
        polygon_marker.id = id;
        polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        polygon_marker.action = visualization_msgs::msg::Marker::ADD;
        polygon_marker.scale.x = 0.25; // Line width
        polygon_marker.color.a = 1.0;  // Alpha
        polygon_marker.color.r = r;
        polygon_marker.color.g = g;
        polygon_marker.color.b = b;

        // Define the corners of the bounding box
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = min_x - CLUSTER_PADDING;
        p1.y = min_y - CLUSTER_PADDING;
        p2.x = max_x + CLUSTER_PADDING;
        p2.y = min_y - CLUSTER_PADDING;
        p3.x = max_x + CLUSTER_PADDING;
        p3.y = max_y + CLUSTER_PADDING;
        p4.x = min_x - CLUSTER_PADDING;
        p4.y = max_y + CLUSTER_PADDING;

        // Add the points to the marker
        polygon_marker.points.push_back(p1);
        polygon_marker.points.push_back(p2);
        polygon_marker.points.push_back(p3);
        polygon_marker.points.push_back(p4);
        polygon_marker.points.push_back(p1); // Close the polygon

        // Add the marker to the MarkerArray
        marker_array.markers.push_back(polygon_marker);

    }

    void FullPathOptimizer::publishLocalSearchArea(std::vector<Frontier> &frontier_list, size_t n)
    {
        auto [filtered_frontiers, outer_frontiers] = getFilteredFrontiers(frontier_list, NUM_FRONTIERS_IN_LOCAL_AREA);


        // Create a MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;
        addToMarkerArray(marker_array, filtered_frontiers, "local_search", 0.5, 1.0, 0.5, 0);
        int id = 0;

        // GLOBAL SEARCH
        while (outer_frontiers.size() > 0)
        {
            auto initial_frontier = outer_frontiers[0];
            std::vector<Frontier> newCluster;
            for (auto it = outer_frontiers.begin(); it != outer_frontiers.end();) {
                if (distanceBetweenFrontiers(*it, initial_frontier) < DISTANCE_THRESHOLD_GLOBAL_CLUSTER) {
                    newCluster.push_back(*it);
                    std::cout << "New frontier in cluster" << std::endl;
                    it = outer_frontiers.erase(it);
                } else {
                    ++it;
                }
            }
            std::cout << "*=*=*=*=*=" << std::endl;
            addToMarkerArray(marker_array, newCluster, "global_search", 1.0, 0.5, 0.3, id);
            ++id;
        }
        // Publish the MarkerArray
        local_search_area_publisher_->publish(marker_array);
    }

    std::vector<Frontier> FullPathOptimizer::getTopThreeFrontiers(std::vector<Frontier> &response)
    {
        // Sort pairs by cost (ascending)
        std::sort(response.begin(), response.end(),
                  [](const Frontier &a, const Frontier &b)
                  {
                      return a.getWeightedCost() < b.getWeightedCost();
                  });

        // Retrieve the top 3 frontiers
        std::vector<Frontier> top_frontiers;
        for (size_t i = 0; i < response.size(); ++i)
        {
            if (i > 2)
                break;
            if (response[i].getWeightedCost() < std::numeric_limits<double>::max())
                top_frontiers.push_back(response[i]);
        }
        if (top_frontiers.size() >= 3)
        {
            frontier_pair_distances_[std::make_pair(0, 1)] = distanceBetweenFrontiers(top_frontiers[0], top_frontiers[1]);
            frontier_pair_distances_[std::make_pair(1, 0)] = distanceBetweenFrontiers(top_frontiers[0], top_frontiers[1]);
            frontier_pair_distances_[std::make_pair(0, 2)] = distanceBetweenFrontiers(top_frontiers[0], top_frontiers[2]);
            frontier_pair_distances_[std::make_pair(2, 0)] = distanceBetweenFrontiers(top_frontiers[0], top_frontiers[2]);
            frontier_pair_distances_[std::make_pair(1, 2)] = distanceBetweenFrontiers(top_frontiers[1], top_frontiers[2]);
            frontier_pair_distances_[std::make_pair(2, 1)] = distanceBetweenFrontiers(top_frontiers[1], top_frontiers[2]);
            frontier_pair_distances_[std::make_pair(-1, 0)] = top_frontiers[0].getPathLength();
            frontier_pair_distances_[std::make_pair(-1, 1)] = top_frontiers[1].getPathLength();
            frontier_pair_distances_[std::make_pair(-1, 2)] = top_frontiers[2].getPathLength();
        }
        auto optimalOrder = findOptimalOrder();
        publishTopFrontiersAsMarkers(top_frontiers, optimalOrder);

        return top_frontiers;
    }

    std::vector<int> FullPathOptimizer::findOptimalOrder()
    {
        std::vector<int> frontiers = {0, 1, 2}; // Order: robot, frontier0, frontier1, frontier2

        // Function to calculate total distance for a given order
        auto calculateTotalDistance = [&](const std::vector<int> &order)
        {
            double totalDistance = 0.0;
            for (int i = 0; i < order.size() - 1; ++i)
            {
                totalDistance += frontier_pair_distances_[std::make_pair(order[i], order[i + 1])];
            }
            return totalDistance;
        };

        // Generate all permutations
        std::vector<std::vector<int>> permutations;
        do
        {
            std::vector<int> frontiers_with_robot = frontiers;
            frontiers_with_robot.insert(frontiers_with_robot.begin(), -1);
            permutations.push_back(frontiers_with_robot);
        } while (std::next_permutation(frontiers.begin(), frontiers.end()));

        // Find the permutation with the minimum total distance
        double minDistance = std::numeric_limits<double>::max();
        std::vector<int> optimalOrder;
        for (const auto &order : permutations)
        {
            std::cout << "Order: ";
            for (auto z : order)
                std::cout << z << ", ";
            std::cout << std::endl;
            double distance = calculateTotalDistance(order);
            if (distance < minDistance)
            {
                minDistance = distance;
                optimalOrder = order;
            }
        }

        return optimalOrder;
    }

    void FullPathOptimizer::publishTopFrontiersAsMarkers(std::vector<Frontier> &top_frontiers, std::vector<int> &optimalOrder)
    {
        if (top_frontiers.size() != optimalOrder.size() - 1)
            return;
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < top_frontiers.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = node_->now();
            marker.ns = "frontier";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = top_frontiers[i].getGoalPoint().x;
            marker.pose.position.y = top_frontiers[i].getGoalPoint().y;
            marker.pose.position.z = 0.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.a = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }
        std::cout << "Optimal order: ";
        for (auto z : optimalOrder)
            std::cout << z << ", ";
        std::cout << std::endl;

        // Add lines for connections based on optimal order
        for (size_t i = 0; i < optimalOrder.size() - 1; ++i)
        {
            std::cout << "i = " << i << std::endl;
            int fromIndex = optimalOrder[i];
            int toIndex = optimalOrder[i + 1];

            if (fromIndex == -1)
            {
                // Line between two frontiers
                visualization_msgs::msg::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = node_->now();
                line_marker.ns = "frontier_connection";
                line_marker.id = i;
                line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::msg::Marker::ADD;

                geometry_msgs::msg::PoseStamped robotPose;
                explore_costmap_ros_->getRobotPose(robotPose);
                line_marker.points.push_back(robotPose.pose.position);
                line_marker.points.push_back(top_frontiers[toIndex].getGoalPoint());

                line_marker.scale.x = 0.50; // Adjust line thickness

                line_marker.color.a = 0.3;
                line_marker.color.r = 0.2;
                line_marker.color.g = 1.0;
                line_marker.color.b = 1.0; // Blue color for lines

                marker_array.markers.push_back(line_marker);
            }
            else
            {
                // Line between two frontiers
                visualization_msgs::msg::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = node_->now();
                line_marker.ns = "frontier_connection";
                line_marker.id = i;
                line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::msg::Marker::ADD;

                line_marker.points.push_back(top_frontiers[fromIndex].getGoalPoint());
                line_marker.points.push_back(top_frontiers[toIndex].getGoalPoint());

                line_marker.scale.x = 0.50; // Adjust line thickness

                line_marker.color.a = 0.3;
                line_marker.color.r = 0.2;
                line_marker.color.g = 1.0;
                line_marker.color.b = 1.0; // Blue color for lines

                marker_array.markers.push_back(line_marker);
            }
        }

        marker_publisher_->publish(marker_array);
    }
};