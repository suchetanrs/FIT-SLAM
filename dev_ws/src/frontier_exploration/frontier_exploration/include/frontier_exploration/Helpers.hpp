// helpers.hpp

#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <vector>
#include <iostream>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/GeometryUtils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

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
         * @param all_min_max These values are taken <= and >= on both sides.
         */
        RayTracedCells(
            nav2_costmap_2d::Costmap2D* costmap,
            std::vector<nav2_costmap_2d::MapLocation> &cells,
            int obstacle_min, int obstacle_max,
            int trace_min, int trace_max)
            : costmap_(costmap), cells_(cells),
            obstacle_min_(obstacle_min), obstacle_max_(obstacle_max),
            trace_min_(trace_min), trace_max_(trace_max)
        {
            hit_obstacle = false;
        };

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
                if ((int)costmap_->getCost(offset) <= trace_max_ && (int)costmap_->getCost(offset) >= trace_min_ && !hit_obstacle)
                {
                    cells_.push_back(loc);
                }
                if ((int)costmap_->getCost(offset) >= obstacle_min_ && (int)costmap_->getCost(offset) <= obstacle_max_)
                {
                    hit_obstacle = true;
                }
            }
        };

        /**
         * @brief Getter function for the vector of cells.
         * @return std::vector<nav2_costmap_2d::MapLocation> The vector of map locations.
         */
        std::vector<nav2_costmap_2d::MapLocation> getCells()
        {
            return cells_;
        };

        bool hasHitObstacle()
        {
            return hit_obstacle;
        };

    private:
        nav2_costmap_2d::Costmap2D* costmap_;
        std::vector<nav2_costmap_2d::MapLocation> &cells_;
        bool hit_obstacle;
        int obstacle_min_, obstacle_max_;
        int trace_min_, trace_max_;
    };

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    void bresenham2D(RayTracedCells& at, unsigned int abs_da, unsigned int abs_db, int error_b,
                     int offset_a,
                     int offset_b, unsigned int offset,
                     unsigned int max_length,
                     int resolution_cut_factor,
                     nav2_costmap_2d::Costmap2D *exploration_costmap_);

    bool getTracedCells(double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells &cell_gatherer, double max_length,
                        nav2_costmap_2d::Costmap2D *exploration_costmap_);

// -------------------------- FISHER INFORMATION COMPUTATION RELATED --------------------------------
    Eigen::Matrix3f getSkewMatrix(const Eigen::Vector3f &v);

    Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose &pose);

    Eigen::Matrix<float, 3, 6> computeJacobianForPoint(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est);

    Eigen::Matrix<float, 6, 6> computeFIM(Eigen::Matrix<float, 3, 6> &jacobian, Eigen::Matrix3f &Q);

    float computeInformationOfPoint(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                     Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f &Q);

    float computeInformationFrontierPair(std::vector<geometry_msgs::msg::Point>& lndmrk_w, 
                                         geometry_msgs::msg::Pose& kf_pose_w, geometry_msgs::msg::Pose& est_pose_w, std::vector<Point2D>& FOVFrontierPair);
}

#endif // HELPERS_HPP