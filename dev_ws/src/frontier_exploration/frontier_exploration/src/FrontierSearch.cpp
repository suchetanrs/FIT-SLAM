#include <frontier_exploration/FrontierSearch.hpp>
namespace frontier_exploration
{

    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

    FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D &costmap)
        : costmap_(costmap)
    {
        min_frontier_cluster_size_ = parameterInstance.getValue<double>("frontierSearch/min_frontier_cluster_size");
        max_frontier_cluster_size_ = parameterInstance.getValue<double>("frontierSearch/max_frontier_cluster_size");
        max_frontier_distance_ = parameterInstance.getValue<double>("frontierSearch/max_frontier_distance");
        original_search_distance_ = parameterInstance.getValue<double>("frontierSearch/max_frontier_distance");
        LOG_DEBUG("FrontierSearch::FrontierSearch");
        LOG_INFO("MAX SEARCH DISTANCE: " << max_frontier_distance_);
        lethal_threshold_ = parameterInstance.getValue<int>("frontierSearch/lethal_threshold");
    }

    std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
    {
        //  frontier_list to store the detected frontiers.
        std::vector<Frontier> frontier_list;

        // Sanity check that robot is inside costmap bounds before searching
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my))
        {
            LOG_CRITICAL("Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

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
        if (nearestFreeCell(clear, pos, lethal_threshold_, costmap_))
        {
            bfs.push(clear);
        }
        else
        {
            bfs.push(pos);
            LOG_WARN("Could not find nearby clear cell to start search, pushing current position of robot to start search");
        }
        visited_flag[bfs.front()] = true;

        while (rclcpp::ok() && !bfs.empty())
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned nbr : nhood4(idx, costmap_))
            {
                // add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                if (isFree(map_[nbr]) && !visited_flag[nbr])
                {
                    visited_flag[nbr] = true;
                    unsigned int nbr_mx, nbr_my;
                    double nbr_wx, nbr_wy;
                    costmap_.indexToCells(nbr, nbr_mx, nbr_my);
                    costmap_.mapToWorld(nbr_mx, nbr_my, nbr_wx, nbr_wy);
                    if (distanceBetweenPoints(position, nbr_wx, nbr_wy) < max_frontier_distance_ + (max_frontier_cluster_size_ * costmap_.getResolution() * 1.414))
                        bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                }
                else if (isNewFrontierCell(nbr, frontier_flag))
                {
                    frontier_flag[nbr] = true;
                    std::vector<Frontier> new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                    for (auto curr_frontier : new_frontier)
                    {
                        if (curr_frontier.getSize() > min_frontier_cluster_size_)
                        {
                            frontier_list.push_back(curr_frontier);
                            LOG_TRACE("PUSHING NEW FRONTIER TO LIST: UID: " << curr_frontier.getUID());
                            LOG_TRACE("Size: " << curr_frontier.getSize());
                            LOG_TRACE("Goal Point: (" << curr_frontier.getGoalPoint().x << ", " << curr_frontier.getGoalPoint().y << ")");
                        }
                    }
                }
            }
        }
        return frontier_list;
    }

    std::vector<Frontier> FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool> &frontier_flag)
    {
        int currentFrontierSize = 1;
        std::vector<Frontier> calculated_frontiers;
        std::vector<std::pair<double, double>> frontier_cell_indices; // used to find the median value in case that is needed to be assigned to frontier.
        // record initial contact point for frontier
        unsigned int ix, iy;
        costmap_.indexToCells(initial_cell, ix, iy);
        double wix, wiy;
        costmap_.mapToWorld(ix, iy, wix, wiy);
        double currentGoalX = wix, currentGoalY = wiy;
        every_frontier_list.push_back({wix, wiy});
        frontier_cell_indices.push_back(std::make_pair(wix, wiy));

        // push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_.indexToCells(reference, rx, ry);
        costmap_.mapToWorld(rx, ry, reference_x, reference_y);
        int current_cluster_size = 0;

        while (rclcpp::ok() && !bfs.empty())
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
                    frontier_cell_indices.push_back(std::make_pair(wx, wy));

                    // update frontier size
                    currentFrontierSize = currentFrontierSize + 1;

                    // add to queue for breadth first search
                    bfs.push(nbr);

                    if (currentFrontierSize > max_frontier_cluster_size_)
                    {
                        Frontier output;
#ifdef FRONTIER_POINT_MEDIAN
                        LOG_DEBUG("*************");
                        LOG_DEBUG("Getting centroid")
                        auto cluster_centroid = getCentroidOfCells(frontier_cell_indices, (costmap_.getResolution() * 1.414 * 2));
                        SortByMedianFunctor sortFunctor(cluster_centroid);
                        std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
                        auto goal_point = frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
                        output.setGoalPoint(goal_point.first, goal_point.second);
                        output.setSize(currentFrontierSize);
                        LOG_DEBUG("Cluster size: " << frontier_cell_indices.size())
                        LOG_DEBUG("x, y goal: " << goal_point.first << " , " << goal_point.second)
                        LOG_DEBUG("Cluster components: ");
                        for (auto i : frontier_cell_indices)
                            LOG_DEBUG_N("x: " << i.first << " y: " << i.second << " atan: " << atan2(i.second - cluster_centroid.second, i.first - cluster_centroid.first) << " ");
                        LOG_DEBUG("");
                        frontier_cell_indices.clear();
#endif
                        output.setUID(generateUID(output));
                        LOG_TRACE("1PUSHING NEW FRONTIER TO LIST: UID: " << output.getUID());
                        LOG_TRACE("1Size: " << output.getSize());
                        LOG_TRACE("1Initial Point: (" << output.getGoalPoint().x << ", " << output.getGoalPoint().y << ", " << output.getGoalPoint().z << ")");
                        LOG_TRACE("**************");
                        calculated_frontiers.push_back(output);

                        // reset variables
                        currentGoalX = wx;
                        currentGoalY = wy;
                        currentFrontierSize = 0;
                    }
                }
            }
        }
        if (currentFrontierSize > min_frontier_cluster_size_)
        {
            Frontier output;
#ifdef FRONTIER_POINT_MEDIAN
            auto cluster_centroid = getCentroidOfCells(frontier_cell_indices, (costmap_.getResolution() * 1.414 * 2));
            SortByMedianFunctor sortFunctor(cluster_centroid);
            std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
            auto goal_point = frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
            output.setGoalPoint(goal_point.first, goal_point.second);
            output.setSize(currentFrontierSize);
            LOG_DEBUG("Cluster size: " << frontier_cell_indices.size())
            LOG_DEBUG("x, y goal: " << goal_point.first << " , " << goal_point.second)
            LOG_DEBUG("Cluster components: ");
            for (auto i : frontier_cell_indices)
                LOG_DEBUG_N(", x: " << i.first << " y: " << i.second << " atan: " << atan2(i.second - cluster_centroid.second, i.first - cluster_centroid.first));
            LOG_DEBUG("");
            frontier_cell_indices.clear();
#endif
            output.setUID(generateUID(output));
            LOG_TRACE("2PUSHING NEW FRONTIER TO LIST: UID: " << output.getUID());
            LOG_TRACE("2Size: " << output.getSize());
            LOG_TRACE("2Initial Point: (" << output.getGoalPoint().x << ", " << output.getGoalPoint().y << ", " << output.getGoalPoint().z << ")");
            LOG_TRACE("2*************====================")
            calculated_frontiers.push_back(output);
        }
        return calculated_frontiers;
    }

    bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag)
    {
        // check that cell is unknown and not already marked as frontier
        if (!isUnknown(map_[idx]) || frontier_flag[idx])
        {
            return false;
        }

        bool has_one_free_neighbour = false;
        bool has_one_lethal_neighbour = false;

        // frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for (unsigned int nbr : nhood4(idx, costmap_))
        {
            if (isFree(map_[nbr]))
            {
                has_one_free_neighbour = true;
            }
            if (isLethal(map_[nbr]))
            {
                has_one_lethal_neighbour = true;
            }
        }
        if (has_one_lethal_neighbour)
            return false;
        else if (has_one_free_neighbour)
            return true;
        else
            return false;

        return false;
    }

    std::vector<std::vector<double>> FrontierSearch::getAllFrontiers()
    {
        return every_frontier_list;
    }

}