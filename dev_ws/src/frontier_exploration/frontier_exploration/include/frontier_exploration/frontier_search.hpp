#ifndef FRONTIER_SEARCH_HPP_
#define FRONTIER_SEARCH_HPP_

#include <list>
#include <vector>
#include <queue>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <frontier_msgs/msg/frontier.hpp>
#include <frontier_exploration/colorize.hpp>

namespace frontier_exploration
{
    /**
     * @brief Thread-safe implementation of a frontier-search task for an input costmap.
     */
    class FrontierSearch
    {

    public:
        /**
         * @brief Constructor for search task
         * @param costmap Reference to costmap data to search.
         */
        FrontierSearch(nav2_costmap_2d::Costmap2D &costmap, int min_frontier_cluster_size, int max_frontier_cluster_size);

        /**
         * @brief Runs search implementation, outward from the start position
         * @param position Initial position to search from
         * @return List of frontiers, if any
         */
        std::vector<frontier_msgs::msg::Frontier> searchFrom(geometry_msgs::msg::Point position);

        /**
         * @brief Getter function for all frontiers unclustered.
         * @return every_frontier variable value.
         */
        std::vector<std::vector<double>> getAllFrontiers();

    protected:
        /**
         * @brief Starting from an initial cell, build a frontier from valid adjacent cells
         * @param initial_cell Index of cell to start frontier building
         * @param reference Reference index to calculate position from
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        std::vector<frontier_msgs::msg::Frontier> buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool> &frontier_flag);

        /**
         * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
         * @param idx Index of candidate cell
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        bool isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

    private:
        nav2_costmap_2d::Costmap2D &costmap_;
        unsigned char *map_;
        unsigned int size_x_, size_y_;
        std::vector<std::vector<double>> every_frontier_list;
        int min_frontier_cluster_size_;
        int max_frontier_cluster_size_;
    };

}
#endif
