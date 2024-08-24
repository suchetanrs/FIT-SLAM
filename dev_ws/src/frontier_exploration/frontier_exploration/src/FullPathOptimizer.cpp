#include <vector>
#include <algorithm>
#include <limits>
#include "frontier_exploration/FullPathOptimizer.hpp"
namespace frontier_exploration
{
    FullPathOptimizer::FullPathOptimizer(rclcpp::Node::SharedPtr node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<FrontierRoadMap> roadmap_ptr)
        : node_(node)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("top_frontiers", 10);
        local_search_area_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("local_search_area", 10);
        roadmap_ptr_ = roadmap_ptr;
    }

    // seperates global and local frontiers.
    void FullPathOptimizer::getFilteredFrontiers(std::vector<Frontier> &frontier_list, size_t n, SortedFrontiers &sortedFrontiers)
    {
        double closest_global_frontier_length = std::numeric_limits<double>::max();
        // get achievable frontiers
        for (const auto &frontier : frontier_list)
        {
            if (frontier.isAchievable())
            {
                std::cout << "Path length in m" << frontier.getPathLengthInM() << std::endl;
                if (frontier.getPathLengthInM() <= LOCAL_FRONTIER_SEARCH_RADIUS)
                {
                    std::cout << "Local Frontiers:" << std::endl;
                    sortedFrontiers.local_frontiers.push_back(frontier);
                    std::cout << frontier << std::endl;
                }
                else if (frontier.getPathLengthInM() > LOCAL_FRONTIER_SEARCH_RADIUS)
                {
                    sortedFrontiers.global_frontiers.push_back(frontier);
                    std::cout << "Global Frontiers:" << std::endl;
                    std::cout << frontier << std::endl;
                    if (frontier.getPathLengthInM() < closest_global_frontier_length)
                    {
                        std::cout << "ADDING CLOSEST FRONTIER" << std::endl;
                        closest_global_frontier_length = frontier.getPathLengthInM();
                        sortedFrontiers.closest_global_frontier = frontier;
                    }
                }
            }
        }
        if (sortedFrontiers.global_frontiers.size() > 0)
        {
            std::cout << "Closest global Frontier:" << std::endl;
            std::cout << sortedFrontiers.closest_global_frontier << std::endl;
        }

        // // Sort the filtered frontiers based on path length (closest first)
        // std::sort(filtered_frontiers.begin(), filtered_frontiers.end(), [](const Frontier &a, const Frontier &b)
        //           { return a.getPathLength() < b.getPathLength(); });

        // // If there are fewer than n frontiers, return all of them
        // if (filtered_frontiers.size() <= n)
        // {
        //     return std::make_tuple(filtered_frontiers, std::vector<Frontier>());
        // }

        // Otherwise, return the first n frontiers
        // return std::make_tuple(std::vector<Frontier>(filtered_frontiers.begin(), filtered_frontiers.begin() + n),
        //                        std::vector<Frontier>(filtered_frontiers.begin() + n, filtered_frontiers.end()));
    }

    void FullPathOptimizer::addToMarkerArrayLinePolygon(visualization_msgs::msg::MarkerArray &marker_array, std::vector<Frontier> &frontier_list,
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

    void FullPathOptimizer::addToMarkerArraySolidPolygon(visualization_msgs::msg::MarkerArray &marker_array, geometry_msgs::msg::Point center, double radius, std::string ns, float r, float g, float b, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = rclcpp::Clock().now();
        marker.header.frame_id = "map"; // Set the frame to map
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // Change to CYLINDER
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = center;
        marker.scale.x = radius * 2; // Ensure this is suitable for a cylinder
        marker.scale.y = radius * 2;
        marker.scale.z = 1.0;
        marker.color.a = 0.15; // Semi-transparent
        marker.color.r = r;    // Red color
        marker.color.g = g;
        marker.color.b = b;
        marker_array.markers.push_back(marker);
    }

    double FullPathOptimizer::calculatePathLength(std::vector<Frontier> &path)
    {
        double totalLength = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            // std::cout << "Inside loop i =" << i << "limit " << path.size() << std::endl;
            // rclcpp::sleep_for(std::chrono::milliseconds(400));
            auto it = frontier_pair_distances_.find(FrontierPair(path[i], path[i + 1]));
            auto it2 = frontier_pair_distances_.find(FrontierPair(path[i + 1], path[i]));
            if (it != frontier_pair_distances_.end())
                totalLength += it->second.path_length_m;
            else if (it2 != frontier_pair_distances_.end())
                totalLength += it2->second.path_length_m;
            else
            {
                auto current_length = roadmap_ptr_->getPlan(path[i], true, path[i + 1], true);
                if (current_length.path_exists == true)
                {
                    totalLength += current_length.path_length_m;
                    frontier_pair_distances_[FrontierPair(path[i], path[i + 1])] = current_length;
                }
                else if (current_length.path_exists == false)
                {
                    // set it to a large value since path could not be found.
                    totalLength += LOCAL_FRONTIER_SEARCH_RADIUS * 100;
                    frontier_pair_distances_[FrontierPair(path[i], path[i + 1])] = current_length;
                }
            }
        }
        return totalLength;
    }

    void FullPathOptimizer::publishLocalSearchArea(std::vector<Frontier> &frontier_list, size_t n, geometry_msgs::msg::PoseStamped &robotP)
    {
        SortedFrontiers sortedFrontiers;
        getFilteredFrontiers(frontier_list, NUM_FRONTIERS_IN_LOCAL_AREA, sortedFrontiers);
        std::cout << "Local frontier list size: " << sortedFrontiers.local_frontiers.size() << std::endl;
        std::cout << "Global frontier list size: " << sortedFrontiers.global_frontiers.size() << std::endl;

        // Create a MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;
        geometry_msgs::msg::PoseStamped robotPose;
        explore_costmap_ros_->getRobotPose(robotPose);
        addToMarkerArraySolidPolygon(marker_array, robotPose.pose.position, LOCAL_FRONTIER_SEARCH_RADIUS, "local_search", 0.5, 1.0, 0.5, 0);
        // addToMarkerArrayLinePolygon(marker_array, sortedFrontiers.local_frontiers, "local_search", 0.5, 1.0, 0.5, 0);
        int id = 0;

        if (sortedFrontiers.global_frontiers.size() > 0)
            addToMarkerArraySolidPolygon(marker_array, sortedFrontiers.closest_global_frontier.getGoalPoint(), 1.0, "global_search", 1.0, 0.5, 0.3, 0);

        local_search_area_publisher_->publish(marker_array);

        double minLength = std::numeric_limits<double>::max();
        if (sortedFrontiers.local_frontiers.size() == 0)
            return;
        std::vector<Frontier> bestPath;
        Frontier robotPoseFrontier;
        robotPoseFrontier.setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
        robotPoseFrontier.setUID(generateUID(robotPoseFrontier));
        robotPoseFrontier.setPathLength(0.0);
        robotPoseFrontier.setPathLengthInM(0.0);

        frontier_pair_distances_.clear();
        do
        {
            // std::cout << "Inserting closest frontier to the beginning." << std::endl;
            if (sortedFrontiers.global_frontiers.size() > 0)
                sortedFrontiers.local_frontiers.insert(sortedFrontiers.local_frontiers.begin(), sortedFrontiers.closest_global_frontier);

            // std::cout << "Inserting robot Pose frontier to the end." << std::endl;
            sortedFrontiers.local_frontiers.push_back(robotPoseFrontier);
            double currentLength = calculatePathLength(sortedFrontiers.local_frontiers);
            if (currentLength < minLength)
            {
                minLength = currentLength;
                bestPath = sortedFrontiers.local_frontiers;
            }
            sortedFrontiers.local_frontiers.pop_back();

            if (sortedFrontiers.global_frontiers.size() > 0)
                sortedFrontiers.local_frontiers.erase(sortedFrontiers.local_frontiers.begin());

        } while (std::next_permutation(sortedFrontiers.local_frontiers.begin(), sortedFrontiers.local_frontiers.end()));
        std::vector<std::shared_ptr<Node>> bestPathViz;
        for (int o = 0; o < bestPath.size() - 1; o++)
        {
            bestPathViz.insert(bestPathViz.end(), frontier_pair_distances_[FrontierPair(bestPath[o], bestPath[o + 1])].path.begin(), frontier_pair_distances_[FrontierPair(bestPath[o], bestPath[o + 1])].path.end());
        }
        roadmap_ptr_->publishPlan(bestPathViz, 1.0, 0.0, 0.0);
        // // GLOBAL SEARCH
        // while (sortedFrontiers.global_frontiers.size() > 0)
        // {
        //     auto initial_frontier = sortedFrontiers.global_frontiers[0];
        //     std::vector<Frontier> newCluster;
        //     for (auto it = sortedFrontiers.global_frontiers.begin(); it != sortedFrontiers.global_frontiers.end();) {
        //         if (distanceBetweenFrontiers(*it, initial_frontier) < DISTANCE_THRESHOLD_GLOBAL_CLUSTER) {
        //             newCluster.push_back(*it);
        //             std::cout << "New frontier in cluster" << std::endl;
        //             it = sortedFrontiers.global_frontiers.erase(it);
        //         } else {
        //             ++it;
        //         }
        //     }
        //     std::cout << "*=*=*=*=*=" << std::endl;
        //     addToMarkerArrayLinePolygon(marker_array, newCluster, "global_search", 1.0, 0.5, 0.3, id);
        //     ++id;
        // }
        // Publish the MarkerArray
    }
};