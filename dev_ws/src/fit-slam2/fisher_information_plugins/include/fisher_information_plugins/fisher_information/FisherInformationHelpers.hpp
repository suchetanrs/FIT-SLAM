// helpers.hpp

#pragma once

#include <vector>
#include <iostream>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "roadmap_explorer/util/GeometryUtils.hpp"

namespace roadmap_explorer
{
    // --------------------------- FOV OVERLAP RELATED ----------------------------------------

    // Function to check if a point lies on the left side of a line segment
    inline bool onLeft(const Point2D &p, const Point2D &q, const Point2D &r)
    {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) > 0;
    }

    // Function to check if a point lies on the left side of a line segment
    inline bool onLeft(const float x, const float y, const Point2D &q, const Point2D &r)
    {
        return (q.x - x) * (r.y - y) - (q.y - y) * (r.x - x) > 0;
    }

    // Function to check if a point is inside a triangle
    inline bool isInside(const Point2D &p, const Point2D &a, const Point2D &b, const Point2D &c)
    {
        // Check if the point is on the left side of all three edges
        return onLeft(p, a, b) && onLeft(p, b, c) && onLeft(p, c, a);
    }

    // Function to check if a point is inside a triangle
    inline bool isInside(float x, float y, const Point2D &a, const Point2D &b, const Point2D &c)
    {
        // Check if the point is on the left side of all three edges
        return onLeft(x, y, a, b) && onLeft(x, y, b, c) && onLeft(x, y, c, a);
    }

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