// helpers.hpp

#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <vector>
#include <iostream>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "frontier_exploration/Frontier.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"
#include <frontier_exploration/planners/planner.hpp>
#include <frontier_exploration/planners/theta_star.hpp>
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
            nav2_costmap_2d::Costmap2D *costmap,
            std::vector<nav2_costmap_2d::MapLocation> &cells,
            int obstacle_min, int obstacle_max,
            int trace_min, int trace_max)
            : costmap_(costmap), cells_(cells),
              obstacle_min_(obstacle_min), obstacle_max_(obstacle_max),
              trace_min_(trace_min), trace_max_(trace_max)
        {
            hit_obstacle = false;
            unknown_cells_ = 0;
            all_cells_count_ = 0;
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
                ++all_cells_count_;
                auto cost = (int)costmap_->getCost(offset);
                if (cost <= trace_max_ && cost >= trace_min_ && !hit_obstacle)
                {
                    cells_.push_back(loc);
                }
                if (cost >= obstacle_min_ && cost <= obstacle_max_)
                {
                    hit_obstacle = true;
                }
                if (cost == 255)
                {
                    unknown_cells_++;
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

        size_t getCellsSize()
        {
            return all_cells_count_;
        };

        bool hasHitObstacle()
        {
            return hit_obstacle;
        };

        size_t getNumUnknown()
        {
            return unknown_cells_;
        };

    private:
        nav2_costmap_2d::Costmap2D *costmap_;
        std::vector<nav2_costmap_2d::MapLocation> &cells_;
        bool hit_obstacle;
        int obstacle_min_, obstacle_max_;
        int trace_min_, trace_max_;
        int unknown_cells_ = 0;
        int all_cells_count_ = 0;
    };

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    void bresenham2D(RayTracedCells &at, unsigned int abs_da, unsigned int abs_db, int error_b,
                     int offset_a,
                     int offset_b, unsigned int offset,
                     unsigned int max_length,
                     int resolution_cut_factor,
                     nav2_costmap_2d::Costmap2D *exploration_costmap_);

    bool getTracedCells(double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells &cell_gatherer, double max_length,
                        nav2_costmap_2d::Costmap2D *exploration_costmap_);

    bool surroundingCellsMapped(geometry_msgs::msg::Point &checkPoint, nav2_costmap_2d::Costmap2D &exploration_costmap_);

    bool isRobotFootprintInLethal(const nav2_costmap_2d::Costmap2D *costmap, unsigned int center_x, unsigned int center_y, double radius_in_cells);

    bool verifyFrontierList(std::vector<Frontier> &frontier_list, const nav2_costmap_2d::Costmap2D *costmap);
    // -------------------------- COSTMAP TOOLS ---------------------------------------------------------

    std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D &costmap);

    std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D &costmap);

    std::vector<unsigned int> nhood20(unsigned int idx, const nav2_costmap_2d::Costmap2D &costmap);

    bool nearestFreeCell(unsigned int &result, unsigned int start, unsigned char val, const nav2_costmap_2d::Costmap2D &costmap);

    // -------------------------- FISHER INFORMATION COMPUTATION RELATED --------------------------------
    Eigen::Matrix3f getSkewMatrix(const Eigen::Vector3f &v);

    Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose &pose);

    Eigen::Matrix<float, 3, 6> computeJacobianForPointGlobal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est);

    Eigen::Matrix<float, 3, 6> computeJacobianForPointLocal(Eigen::Vector3f &p3d_c_eig);

    Eigen::Matrix<float, 3, 6> computeJacobianForPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est);

    Eigen::Matrix<float, 6, 6> computeFIM(Eigen::Matrix<float, 3, 6> &jacobian, Eigen::Matrix3f &Q);

    float computeInformationOfPointGlobal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                          Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f Q);

    float computeInformationOfPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                         Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f Q);

    float computeInformationOfPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Matrix3f Q);

    float computeInformationFrontierPair(std::vector<geometry_msgs::msg::Point> &lndmrk_w,
                                         geometry_msgs::msg::Pose &kf_pose_w, geometry_msgs::msg::Pose &est_pose_w, std::vector<Point2D> &FOVFrontierPair);

    bool computePathBetweenPoints(nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &start_point, const geometry_msgs::msg::Point &goal_point, bool planner_allow_unknown, nav2_costmap_2d::Costmap2D *exploration_costmap_);

    bool computePathBetweenPointsThetaStar(nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &start_point, const geometry_msgs::msg::Point &goal_point, bool planner_allow_unknown, nav2_costmap_2d::Costmap2D *exploration_costmap_);
}

#endif // HELPERS_HPP