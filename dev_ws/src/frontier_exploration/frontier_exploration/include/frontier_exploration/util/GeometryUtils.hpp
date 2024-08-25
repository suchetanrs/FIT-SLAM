#ifndef GEOMETRYUTILS_HPP
#define GEOMETRYUTILS_HPP

#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include "frontier_exploration/Frontier.hpp"

namespace frontier_exploration
{

    struct Point2D
    {
        double x;
        double y;
    };

    inline std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion &quat)
    {
        tf2::Quaternion tf2_quaternion(
            quat.x, quat.y, quat.z, quat.w);

        // Convert tf2 quaternion to Euler angles
        tf2::Matrix3x3 matrix(tf2_quaternion);
        std::vector<double> rpy = {0, 0, 0};
        matrix.getRPY(rpy[0], rpy[1], rpy[2]);
        return rpy;
    };

    inline geometry_msgs::msg::Quaternion yawToQuat(double yaw)
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw); // Set roll, pitch, and yaw. Here, roll and pitch are 0.

        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = quaternion.x();
        quat_msg.y = quaternion.y();
        quat_msg.z = quaternion.z();
        quat_msg.w = quaternion.w();

        return quat_msg;
    };

    inline double distanceBetweenFrontiers(const Frontier &f1, const Frontier &f2)
    {
        return sqrt(pow(f1.getGoalPoint().x - f2.getGoalPoint().x, 2) + pow(f1.getGoalPoint().y - f2.getGoalPoint().y, 2));
    };

    inline double distanceBetweenPoints(const geometry_msgs::msg::Point &f1, const geometry_msgs::msg::Point &f2)
    {
        return sqrt(pow(f1.x - f2.x, 2) + pow(f1.y - f2.y, 2));
    };

    inline double distanceBetweenPoints(const geometry_msgs::msg::Point &f1, const double x2, const double y2)
    {
        return sqrt(pow(f1.x - x2, 2) + pow(f1.y - y2, 2));
    };

    inline double sqDistanceBetweenFrontiers(const Frontier &f1, const Frontier &f2)
    {
        return pow(f1.getGoalPoint().x - f2.getGoalPoint().x, 2) + pow(f1.getGoalPoint().y - f2.getGoalPoint().y, 2);
    };

    inline void getFOVKeyframe(geometry_msgs::msg::Pose &pose, double max_depth, double hfov, std::vector<Point2D> &FOV)
    {
        auto alpha = quatToEuler(pose.orientation);

        Point2D point = {pose.position.x, pose.position.y};
        FOV.push_back(point);

        point = {pose.position.x + max_depth * cos(alpha[2] - hfov / 2), pose.position.y + max_depth * sin(alpha[2] - hfov / 2)};
        FOV.push_back(point);

        point = {pose.position.x + max_depth * cos(alpha[2] + hfov / 2), pose.position.y + max_depth * sin(alpha[2] + hfov / 2)};
        FOV.push_back(point);

        return;
    };

    inline double getFOVFrontierPair(const Frontier &frontier_from, const Frontier &frontier_to, double hfov, std::vector<Point2D> &FOV)
    {
        double delta_x = frontier_to.getGoalPoint().x - frontier_from.getGoalPoint().x;
        double delta_y = frontier_to.getGoalPoint().y - frontier_from.getGoalPoint().y;
        auto alpha = atan2(delta_y, delta_x);
        auto max_depth = distanceBetweenFrontiers(frontier_from, frontier_to);

        Point2D point = {frontier_from.getGoalPoint().x, frontier_from.getGoalPoint().y};
        FOV.push_back(point);

        point = {frontier_from.getGoalPoint().x + max_depth * cos(alpha - hfov / 2), frontier_from.getGoalPoint().y + max_depth * sin(alpha - hfov / 2)};
        FOV.push_back(point);

        point = {frontier_from.getGoalPoint().x + max_depth * cos(alpha + hfov / 2), frontier_from.getGoalPoint().y + max_depth * sin(alpha + hfov / 2)};
        FOV.push_back(point);

        return alpha;
    };

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

    // Function to check if two triangles overlap
    inline bool doFOVsOverlap(const Point2D &p1, const Point2D &p2, const Point2D &p3,
                              const Point2D &q1, const Point2D &q2, const Point2D &q3)
    {
        // Check if any vertex of one triangle is inside the other triangle
        if (isInside(p1, q1, q2, q3) || isInside(p2, q1, q2, q3) || isInside(p3, q1, q2, q3) ||
            isInside(q1, p1, p2, p3) || isInside(q2, p1, p2, p3) || isInside(q3, p1, p2, p3))
        {
            return true;
        }

        return false;
    }

    // Function to check if two triangles overlap
    inline bool doFOVsOverlap(const std::vector<Point2D> &t1, const std::vector<Point2D> &t2)
    {
        if (t1.size() != 3 || t2.size() != 3)
        {
            throw std::runtime_error("You are defining a triangle which does not have 3 vertices.");
        }
        // Check if any vertex of one triangle is inside the other triangle
        if (isInside(t1[0], t2[0], t2[1], t2[2]) || isInside(t1[1], t2[0], t2[1], t2[2]) || isInside(t1[2], t2[0], t2[1], t2[2]) ||
            isInside(t2[0], t1[0], t1[1], t1[2]) || isInside(t2[1], t1[0], t1[1], t1[2]) || isInside(t2[2], t1[0], t1[1], t1[2]))
        {
            return true;
        }

        return false;
    }

}
#endif