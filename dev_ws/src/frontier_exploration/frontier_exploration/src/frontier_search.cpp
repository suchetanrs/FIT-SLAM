#include <frontier_exploration/costmap_tools.hpp>
#include <frontier_exploration/frontier_search.hpp>
namespace frontier_exploration
{

    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

    FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D &costmap, int min_frontier_cluster_size, int max_frontier_cluster_size) : costmap_(costmap), min_frontier_cluster_size_(min_frontier_cluster_size), max_frontier_cluster_size_(max_frontier_cluster_size)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("frontier_search"), "FrontierSearch::FrontierSearch");
    }

    std::vector<frontier_msgs::msg::Frontier> FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
    {
        //  frontier_list to store the detected frontiers.
        std::vector<frontier_msgs::msg::Frontier> frontier_list;

        // Sanity check that robot is inside costmap bounds before searching
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my))
        {
            RCLCPP_ERROR(rclcpp::get_logger("frontier_search"), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        map_ = costmap_.getCharMap();
        size_x_ = costmap_.getSizeInCellsX();
        size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_, false);
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        // initialize breadth first search queue
        std::queue<unsigned int> bfs;

        // find closest clear cell to start search
        unsigned int clear, pos = costmap_.getIndex(mx, my);
        if (nearestCell(clear, pos, FREE_SPACE, costmap_))
        {
            bfs.push(clear);
        }
        else
        {
            bfs.push(pos);
            RCLCPP_WARN(rclcpp::get_logger("frontier_search"), "Could not find nearby clear cell to start search, pushing current position of robot to start search");
        }
        visited_flag[bfs.front()] = true;

        while (!bfs.empty())
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned nbr : nhood4(idx, costmap_))
            {
                // RCLCPP_INFO(rclcpp::get_logger("frontier_search"), "New nhood4");
                // add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                if (map_[nbr] <= map_[idx] && !visited_flag[nbr])
                {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                }
                else if (isNewFrontierCell(nbr, frontier_flag))
                {
                    frontier_flag[nbr] = true;
                    std::vector<frontier_msgs::msg::Frontier> new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                    for (auto curr_frontier : new_frontier)
                    {
                        if (curr_frontier.size > min_frontier_cluster_size_)
                        {
                            // std::cout << "PUSHING NEW FRONTIER TO LIST: UID: " << curr_frontier.unique_id << std::endl;
                            // std::cout << "Size: " << curr_frontier.size << std::endl;
                            // std::cout << "Minimum Distance: " << curr_frontier.min_distance << std::endl;
                            // std::cout << "Goal Point: (" << curr_frontier.goal_point.x << ", " << curr_frontier.goal_point.y << ")" << std::endl;
                            frontier_list.push_back(curr_frontier);
                        }
                    }
                }
            }
        }

        return frontier_list;
    }

    std::vector<frontier_msgs::msg::Frontier> FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool> &frontier_flag)
    {
        frontier_msgs::msg::Frontier output;
        // initialize frontier structure
        output.goal_point.x = 0.0;
        output.goal_point.y = 0.0;
        output.size = 1;
        output.min_distance = std::numeric_limits<double>::infinity();
        std::vector<frontier_msgs::msg::Frontier> calculated_frontiers;
        // record initial contact point for frontier
        unsigned int ix, iy;
        costmap_.indexToCells(initial_cell, ix, iy);
        #ifdef FRONTIER_POINT_INITIAL
            costmap_.mapToWorld(ix, iy, output.goal_point.x, output.goal_point.y);
        #endif

        // push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_.indexToCells(reference, rx, ry);
        costmap_.mapToWorld(rx, ry, reference_x, reference_y);
        int current_cluster_size = 0;

        while (!bfs.empty())
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr : nhood8(idx, costmap_))
            {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontier_flag))
                {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    // add to every frontier list
                    std::vector<double> coord_val;
                    coord_val.push_back(wx);
                    coord_val.push_back(wy);

                    every_frontier_list.push_back(coord_val);

                    // update frontier size
                    output.size++;

                    #ifdef FRONTIER_POINT_CENTROID
                    // update centroid of frontier
                    output.goal_point.x += wx;
                    output.goal_point.y += wy;
                    #endif

                    // determine frontier's distance from robot, going by closest gridcell to robot
                    double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) + pow((double(reference_y) - double(wy)), 2.0));
                    if (distance < output.min_distance)
                    {
                        output.min_distance = distance;
                        #ifdef FRONTIER_POINT_MIDDLE
                        output.goal_point.x = wx;
                        output.goal_point.y = wy;
                        #endif
                    }

                    // add to queue for breadth first search
                    bfs.push(nbr);

                    if (output.size > max_frontier_cluster_size_)
                    {
                        // push to output vector
                        #ifdef FRONTIER_POINT_CENTROID
                        output.goal_point.x /= output.size;
                        output.goal_point.y /= output.size;
                        #endif
                        output.unique_id = generateUID(output);
                        // std::cout << "1PUSHING NEW FRONTIER TO LIST: UID: " << output.unique_id << std::endl;
                        // std::cout << "1Size: " << output.size << std::endl;
                        // std::cout << "1Minimum Distance: " << output.min_distance << std::endl;
                        // std::cout << "1Initial Point: (" << output.goal_point.x << ", " << output.goal_point.y << ", " << output.goal_point.z << ")" << std::endl;
                        // std::cout << "**************" << std::endl;
                        calculated_frontiers.push_back(output);

                        // reset frontier structure
                        #ifdef FRONTIER_POINT_CENTROID
                        output.goal_point.x = 0.0;
                        output.goal_point.y = 0.0;
                        #endif
                        #ifdef FRONTIER_POINT_MIDDLE
                        output.goal_point.x = 0.0;
                        output.goal_point.y = 0.0;
                        #endif
                        #ifdef FRONTIER_POINT_INITIAL
                        output.goal_point.x = wx;
                        output.goal_point.y = wy;
                        #endif
                        output.size = 1;
                        output.min_distance = std::numeric_limits<double>::infinity();
                    }
                }
            }
        }

        // average out frontier centroid
        #ifdef FRONTIER_POINT_CENTROID
        output.goal_point.x /= output.size;
        output.goal_point.y /= output.size;
        #endif
        output.unique_id = generateUID(output);
        // std::cout << "1PUSHING NEW FRONTIER TO LIST: UID: " << output.unique_id << std::endl;
        // std::cout << "1Size: " << output.size << std::endl;
        // std::cout << "1Minimum Distance: " << output.min_distance << std::endl;
        // std::cout << "1Initial Point: (" << output.goal_point.x << ", " << output.goal_point.y << ", " << output.goal_point.z << ")" << std::endl;
        // std::cout << "**************====================" << std::endl;
        calculated_frontiers.push_back(output);
        return calculated_frontiers;
    }

    bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag)
    {

        // check that cell is unknown and not already marked as frontier
        if (map_[idx] != NO_INFORMATION || frontier_flag[idx])
        {
            return false;
        }

        // frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for (unsigned int nbr : nhood4(idx, costmap_))
        {
            if (map_[nbr] == FREE_SPACE)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<std::vector<double>> FrontierSearch::getAllFrontiers()
    {
        return every_frontier_list;
    }

}