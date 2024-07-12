// helpers.cpp

#include "frontier_exploration/Helpers.hpp" // Include the corresponding header file

namespace frontier_exploration
{
    void bresenham2D(
        RayTracedCells& at, unsigned int abs_da, unsigned int abs_db, int error_b,
        int offset_a,
        int offset_b, unsigned int offset,
        unsigned int max_length,
        int resolution_cut_factor,
        nav2_costmap_2d::Costmap2D *exploration_costmap_)
    {
        auto max_offset = exploration_costmap_->getSizeInCellsX() * exploration_costmap_->getSizeInCellsY();
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            if (i % resolution_cut_factor == 0)
                at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }

    bool getTracedCells(double sx, double sy, double wx, double wy, RayTracedCells &cell_gatherer, double max_length,
                        nav2_costmap_2d::Costmap2D *exploration_costmap_)
    {
        unsigned int min_length = 0.0;
        int resolution_cut_factor = 1;
        // Calculate map coordinates
        unsigned int x1, y1;
        unsigned int x0, y0;
        if (!exploration_costmap_->worldToMap(wx, wy, x1, y1) || !exploration_costmap_->worldToMap(sx, sy, x0, y0))
        {
            std::cerr << "Not world to map" << std::endl;
            return false;
        }

        // Calculate distance and adjust starting point to min_length distance
        int dx_full = x1 - x0;
        int dy_full = y1 - y0;
        double dist = std::hypot(dx_full, dy_full);
        if (dist < min_length)
        {
            std::cerr << "Distance to ray trace is lesser than minimum distance. Proceeding to next frontier." << std::endl;
            return false;
        }
        unsigned int min_x0, min_y0;
        if (dist > 0.0)
        {
            // Adjust starting point and offset to start from min_length distance
            min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
            min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
        }
        else
        {
            min_x0 = x0;
            min_y0 = y0;
        }
        unsigned int offset = min_y0 * exploration_costmap_->getSizeInCellsX() + min_x0;

        int dx = x1 - min_x0;
        int dy = y1 - min_y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * exploration_costmap_->getSizeInCellsX();

        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
        // Calculate the maximum number of steps based on resolution_cut_factor
        // if x is dominant
        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;

            frontier_exploration::bresenham2D(
                cell_gatherer, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx), resolution_cut_factor, exploration_costmap_);
        }
        else
        {
            // otherwise y is dominant
            int error_x = abs_dy / 2;
            frontier_exploration::bresenham2D(
                cell_gatherer, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy), resolution_cut_factor, exploration_costmap_);
        }
        return true;
    }

    double distanceBetweenFrontiers(const Frontier& f1, const Frontier& f2)
    {
        return sqrt(pow(f1.getGoalPoint().x - f2.getGoalPoint().x, 2) + pow(f1.getGoalPoint().y - f2.getGoalPoint().y, 2));
    }

    double sqDistanceBetweenFrontiers(const Frontier& f1, const Frontier& f2)
    {
        return pow(f1.getGoalPoint().x - f2.getGoalPoint().x, 2) + pow(f1.getGoalPoint().y - f2.getGoalPoint().y, 2);
    }
}