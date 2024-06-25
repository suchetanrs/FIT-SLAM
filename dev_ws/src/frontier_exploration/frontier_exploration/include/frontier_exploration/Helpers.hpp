// helpers.hpp

#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <vector>
#include <iostream>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "frontier_exploration/Frontier.hpp"

namespace frontier_exploration
{
    class RayTracedCells
    {
    public:
        /**
         * @brief Constructor for RayTracedCells.
         *
         * @param costmap The reference to the costmap.
         * @param cells The vector of map locations to store ray-traced cells.
         */
        RayTracedCells(
            nav2_costmap_2d::Costmap2D* costmap,
            std::vector<nav2_costmap_2d::MapLocation> &cells)
            : costmap_(costmap), cells_(cells)
        {
            hit_obstacle = false;
        }

        /**
         * @brief Function call operator to add unexplored cells to the list.
         * This operator adds cells that are currently unexplored to the list of cells.
         * i.e pushes the relevant cells back onto the list.
         * @param offset The offset of the cell to consider.
         */
        inline void operator()(unsigned int offset)
        {
            nav2_costmap_2d::MapLocation loc;
            costmap_->indexToCells(offset, loc.x, loc.y);
            bool presentflag = false;
            for (auto item : cells_)
            {
                if (item.x == loc.x && item.y == loc.y)
                    presentflag = true;
            }
            if (presentflag == false)
            {
                if ((int)costmap_->getCost(offset) == 255 && hit_obstacle == false)
                {
                    cells_.push_back(loc);
                }
                if ((int)costmap_->getCost(offset) > 240 && (int)costmap_->getCost(offset) != 255)
                {
                    hit_obstacle = true;
                }
            }
        }

        /**
         * @brief Getter function for the vector of cells.
         * @return std::vector<nav2_costmap_2d::MapLocation> The vector of map locations.
         */
        std::vector<nav2_costmap_2d::MapLocation> getCells()
        {
            return cells_;
        }

        bool hasHitObstacle()
        {
            return hit_obstacle;
        }

    private:
        nav2_costmap_2d::Costmap2D* costmap_;
        std::vector<nav2_costmap_2d::MapLocation> &cells_;
        bool hit_obstacle;
    };

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    void bresenham2D(RayTracedCells at, unsigned int abs_da, unsigned int abs_db, int error_b,
                     int offset_a,
                     int offset_b, unsigned int offset,
                     unsigned int max_length,
                     int resolution_cut_factor,
                     nav2_costmap_2d::Costmap2D *exploration_costmap_);

    bool getTracedCells(double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells &cell_gatherer, double max_length,
                        nav2_costmap_2d::Costmap2D *exploration_costmap_);

    double distanceBetweenFrontiers(const Frontier& f1, const Frontier& f2);
}

#endif // HELPERS_HPP