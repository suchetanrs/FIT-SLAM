// ==================================================================================================
//
// This file (util.hpp) is DEPRECATED and is NO LONGER USED anywhere in the codebase. 
//
// DO NOT USE THIS FILE. It is being kept for reference only and may be removed in the future.
//
// If you need any of the functionality previously provided here, you'll need to reimplement it 
// or find alternative solutions.
//
// ==================================================================================================

#ifndef FRONTIER_EXPLORATION_UTIL_HPP_
#define FRONTIER_EXPLORATION_UTIL_HPP_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <slam_msgs/msg/key_frame.hpp>
#include <slam_msgs/msg/map_data.hpp>

#include <nav2_util/geometry_utils.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>

namespace frontier_exploration_utils
{
    /**
     * @brief Checks if a 2D point is inside a triangle.
     * This function determines whether a given 2D point lies within a triangle
     * defined by its vertices.
     *
     * @param point The 2D point to check, represented as a vector of size 2.
     * @param triangle_vertices The vertices of the triangle, represented as a vector of size 3,
     *                          where each vertex is represented as a vector of size 2 containing x and y coordinates.
     *
     * @return true if the point is inside the triangle, false otherwise.
     */
    template<typename PointType>
    bool isPointInsideTriangle(PointType& point, std::vector<std::vector<double>>& triangle_vertices)
    {
        std::vector<double> v0 = {triangle_vertices[2][0] - triangle_vertices[0][0], triangle_vertices[2][1] - triangle_vertices[0][1]};
        std::vector<double> v1 = {triangle_vertices[1][0] - triangle_vertices[0][0], triangle_vertices[1][1] - triangle_vertices[0][1]};
        std::vector<double> v2 = {point[0] - triangle_vertices[0][0], point[1] - triangle_vertices[0][1]};

        double dot00 = v0[0] * v0[0] + v0[1] * v0[1];
        double dot01 = v0[0] * v1[0] + v0[1] * v1[1];
        double dot02 = v0[0] * v2[0] + v0[1] * v2[1];
        double dot11 = v1[0] * v1[0] + v1[1] * v1[1];
        double dot12 = v1[0] * v2[0] + v1[1] * v2[1];

        double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);
    }

    /**
     * @brief Converts a quaternion to Euler angles.
     * This function converts a quaternion representation of orientation to Euler angles (roll, pitch, and yaw).
     *
     * @param quat The quaternion representing the orientation.
     *
     * @return A vector containing Euler angles in radians: [roll, pitch, yaw].
     */
    // TODO: Verify if this quaternion has to be normalized.
    std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion &quat)
    {
        tf2::Quaternion tf2_quaternion(
            quat.x, quat.y, quat.z, quat.w);

        // Convert tf2 quaternion to Euler angles
        tf2::Matrix3x3 matrix(tf2_quaternion);
        std::vector<double> rpy = {0, 0, 0};
        matrix.getRPY(rpy[0], rpy[1], rpy[2]);
        return rpy;
    }

    /**
     * @brief Returns the vertices of a 2D frustum (triangle) given the pose, max_depth of observation, and horizontal field of view (hfov).
     *
     * This function calculates the vertices of a 2D frustum (triangle) based on the provided pose (position and orientation),
     * maximum depth of observation, and horizontal field of view (hfov).
     *
     * @param pose The pose of the camera, including position and orientation.
     * @param max_depth The maximum depth of observation.
     * @param hfov The horizontal field of view of the camera.
     *
     * @return A vector of vectors representing the vertices of the frustum in 2D space. Each vertex is represented as [x, y].
     */
    std::vector<std::vector<double>> getVerticesOfFrustum2D(geometry_msgs::msg::Pose &pose, double max_depth, double hfov)
    {
        std::vector<std::vector<double>> vertices;
        auto alpha = quatToEuler(pose.orientation);

        std::vector<double> point = {pose.position.x, pose.position.y};
        vertices.push_back(point);
        point.clear();

        point = {pose.position.x + max_depth * cos(alpha[2] - hfov / 2), pose.position.y + max_depth * sin(alpha[2] - hfov / 2)};
        vertices.push_back(point);
        point.clear();

        point = {pose.position.x + max_depth * cos(alpha[2] + hfov / 2), pose.position.y + max_depth * sin(alpha[2] + hfov / 2)};
        vertices.push_back(point);
        point.clear();

        return vertices;
    }

    /**
     * @brief Returns the midpoints of the three sides of a 2D triangle along with its vertices.
     *
     * This function calculates the midpoints of the three sides of a 2D triangle along with its vertices.
     *
     * @param pose The pose of the camera, including position and orientation.
     * @param max_depth The maximum depth of observation.
     * @param hfov The horizontal field of view of the camera.
     *
     * @return A vector of vectors representing the vertices of the triangle and the midpoints of its sides in 2D space. Each vertex is represented as [x, y].
     *
     * @note Modify this function if you want to check for more points other than just these 6.
     */
    std::vector<std::vector<double>> getVerticesToCheck(geometry_msgs::msg::Pose &pose, double max_depth, double hfov)
    {
        std::vector<std::vector<double>> vertices = getVerticesOfFrustum2D(pose, max_depth, hfov);

        std::vector<double> point;
        point.push_back((vertices[0][0] + vertices[1][0]) / 2);
        point.push_back((vertices[0][1] + vertices[1][1]) / 2);
        vertices.push_back(point);
        point.clear();

        point.push_back((vertices[1][0] + vertices[2][0]) / 2);
        point.push_back((vertices[1][1] + vertices[2][1]) / 2);
        vertices.push_back(point);
        point.clear();

        point.push_back((vertices[2][0] + vertices[0][0]) / 2);
        point.push_back((vertices[2][1] + vertices[0][1]) / 2);
        vertices.push_back(point);
        point.clear();

        return vertices;
    }

    /**
     * @brief Checks if the camera frustums of two poses have any overlap.
     *
     * This function determines if there is any overlap between the camera frustums of two poses.
     * The max_depth_error parameter is the value added to the max_depth of the 3D camera.
     * For example, if the error is 0.5, the camera field of view (FOV) triangle will have a height of max_depth + max_depth_error.
     *
     * @param curr_pose The pose of the current camera.
     * @param pose_to_check The pose of the camera to be checked for overlap.
     * @param max_depth The maximum depth of observation of the camera.
     * @param hfov The horizontal field of view (HFOV) of the camera.
     * @param max_depth_error The error value added to the max_depth of the camera.
     *
     * @return True if there is overlap between the camera frustums, otherwise false.
     */
    bool frustumOverlap(geometry_msgs::msg::Pose &curr_pose, geometry_msgs::msg::Pose &pose_to_check, double max_depth, double hfov, double max_depth_error)
    {
        auto vertices_pose = getVerticesOfFrustum2D(curr_pose, max_depth + max_depth_error, hfov);
        auto vertices_check = getVerticesToCheck(pose_to_check, max_depth + max_depth_error, hfov);
        for (auto pose : vertices_check)
        {
            if (isPointInsideTriangle(pose, vertices_pose))
            {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Converts a vector of double values to a geometry_msgs::msg::Point.
     *
     * This function takes a vector containing double values representing coordinates
     * and converts them into a geometry_msgs::msg::Point.
     *
     * @param vec The vector containing the x and y coordinates.
     * @return The geometry_msgs::msg::Point representation of the coordinates.
     */
    geometry_msgs::msg::Point getPointFromVector(std::vector<double>& vec)
    {
        geometry_msgs::msg::Point pnt;
        pnt.x = vec[0];
        pnt.y = vec[1];
        return pnt;
    }
}

// namespace frontier_exploration_information
// {

//     /**
//      * @brief Computes the skew-symmetric matrix for a 3D vector.
//      *
//      * This function calculates the skew-symmetric matrix representation of a 3D vector.
//      * The skew-symmetric matrix has the property that when multiplied by another vector,
//      * it produces the cross product of the original vector and the other vector.
//      *
//      * @param v The 3D vector.
//      * @return The skew-symmetric matrix corresponding to the input vector.
//      *         The last entry of the input vector is not considered for computation.
//      * @note The last element in v is 1. It is not used for computation.
//      */
//     Eigen::Matrix3d getSkewMatrix(const Eigen::Vector4d &v)
//     {
//         Eigen::Matrix3d skewMat;
//         skewMat << 0, -v(2), v(1),
//             v(2), 0, -v(0),
//             -v(1), v(0), 0;
//         return skewMat;
//     }

//     /**
//      * @brief Computes the 4x4 transformation matrix from a ROS Transform message.
//      *
//      * This function computes the 4x4 transformation matrix from the rotation and translation
//      * components of a ROS Transform message.
//      *
//      * @param transform The ROS Transform message containing rotation and translation information.
//      * @return The 4x4 transformation matrix computed from the input transform.
//      */
//     Eigen::Matrix4d computeTransformationMatrix(geometry_msgs::msg::Transform &transform)
//     {
//         Eigen::Quaterniond quaternion(
//             transform.rotation.w,
//             transform.rotation.x,
//             transform.rotation.y,
//             transform.rotation.z);
//         quaternion.normalize();
//         Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
//         Eigen::Matrix<double, 4, 1> translation_matrix;
//         translation_matrix << transform.translation.x, transform.translation.y, transform.translation.z, 1;
//         Eigen::Matrix<double, 4, 3> rt_mat;
//         rt_mat.topRows(3) = rotation_matrix;
//         rt_mat.bottomRows(1).setZero();

//         Eigen::Matrix4d transformation_matrix;

//         transformation_matrix << rt_mat, translation_matrix;
//         return transformation_matrix;
//     }

//     /**
//      * @brief Computes the 3x3 rotation matrix from a 4x4 transformation matrix.
//      *
//      * This function extracts the 3x3 rotation matrix from the upper-left 3x3 block
//      * of a 4x4 transformation matrix.
//      *
//      * @param transformation_matrix The 4x4 transformation matrix.
//      * @return The 3x3 rotation matrix extracted from the transformation matrix.
//      */
//     Eigen::Matrix3d computeRotationMatrix(const Eigen::Matrix4d &transformation_matrix)
//     {
//         return transformation_matrix.block<3, 3>(0, 0);
//     }

//     /**
//      * @brief Transforms a 3D point from camera frame to world frame using a transformation matrix.
//      *
//      * This function performs the transformation of a 3D point from the camera frame to the world frame
//      * using the provided transformation matrix T_w_c.
//      *
//      * @param p3d_c The 3D point in the camera frame represented as a 4D homogeneous vector.
//      * @param T_w_c The transformation matrix from camera frame to world frame.
//      * @return The transformed 3D point in the world frame represented as a 4D homogeneous vector.
//      */
//     Eigen::Vector4d transformPoint_C_W(Eigen::Vector4d &p3d_c, const Eigen::Matrix4d &T_w_c)
//     {
//         return T_w_c * p3d_c;
//     }

//     /**
//      * @brief Transforms a 3D point from camera frame to world frame using a transformation message.
//      *
//      * This function transforms a 3D point from the camera frame to the world frame using the provided
//      * transformation message T_w_c. The transformation message is converted into a transformation matrix
//      * internally for the transformation computation.
//      *
//      * @param p3d_c The 3D point in the camera frame represented as a 4D homogeneous vector.
//      * @param T_w_c The transformation from camera frame to world frame represented as a geometry_msgs::msg::Transform.
//      * @return The transformed 3D point in the world frame represented as a 4D homogeneous vector.
//      */
//     Eigen::Vector4d transformPoint_C_W(Eigen::Vector4d &p3d_c, geometry_msgs::msg::Transform &T_w_c)
//     {
//         return computeTransformationMatrix(T_w_c) * p3d_c;
//     }

//     /**
//      * @brief Computes the Jacobian matrix for a 3D point transformation.
//      *
//      * This function computes the Jacobian matrix for transforming a 3D point from camera frame to world frame
//      * with respect to the given camera-to-world transformation (T_w_c) and the estimated camera-to-world
//      * transformation (T_w_c_est). The Jacobian matrix represents the sensitivity of the transformed point
//      * to changes in the camera-to-world transformation.
//      *
//      * @param p3d_c4_measurement The 3D point in the camera frame represented as a 4D homogeneous vector.
//      * @param T_w_c The transformation from camera frame to world frame represented as a geometry_msgs::msg::Transform.
//      * @param T_w_c_est The estimated transformation from camera frame to world frame represented as a geometry_msgs::msg::Transform.
//      * @return The Jacobian matrix for the point transformation, represented as a 3x6 matrix.
//      */
//     Eigen::Matrix<double, 3, 6> computeJacobianForPoint(Eigen::Vector4d &p3d_c4_measurement, geometry_msgs::msg::Transform &T_w_c, geometry_msgs::msg::Transform &T_w_c_est)
//     {
//         Eigen::Matrix4d transformation_matrix_w_c = computeTransformationMatrix(T_w_c);
//         Eigen::Vector4d p3d_w4 = transformation_matrix_w_c * p3d_c4_measurement;

//         Eigen::Matrix4d transformation_matrix_w_c_est = computeTransformationMatrix(T_w_c_est);
//         Eigen::Vector4d p3d_c4 = transformation_matrix_w_c_est.inverse() * p3d_w4;

//         // df_dp
//         const double n = p3d_c4.head<3>().norm();
//         Eigen::Matrix3d df_dpc = (1 / n) * Eigen::Matrix3d::Identity() -
//                                  (1 / (n * n * n)) * p3d_c4.head<3>() * p3d_c4.head<3>().transpose();

//         // dp_dTwc
//         Eigen::Matrix<double, 3, 6> rightMat;
//         rightMat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
//         rightMat.block<3, 3>(0, 3) = (-1.0) * getSkewMatrix(p3d_w4);
//         Eigen::Matrix<double, 3, 3> leftMat;
//         leftMat = computeRotationMatrix(transformation_matrix_w_c_est).inverse();
//         Eigen::Matrix<double, 3, 6> dpc_dtwc = leftMat * rightMat;

//         Eigen::Matrix<double, 3, 6> jacobian = df_dpc * dpc_dtwc;
//         // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
//         return jacobian;
//     }

//     /**
//      * @brief Computes the Fisher Information Matrix (FIM) for a point.
//      *
//      * This function computes the Fisher Information Matrix (FIM) for a 3D point transformation
//      * based on the provided Jacobian matrix and the covariance matrix Q. The FIM quantifies
//      * the amount of information provided by the point transformation.
//      *
//      * @param jacobian The Jacobian matrix for the point transformation, represented as a 3x6 matrix.
//      * @param Q The covariance matrix, represented as a 3x3 Eigen::Matrix3d.
//      * @return The Fisher Information Matrix (FIM) for the point, represented as a 6x6 matrix.
//      */
//     Eigen::Matrix<double, 6, 6> computeFIM(Eigen::Matrix<double, 3, 6> &jacobian, Eigen::Matrix3d &Q)
//     {
//         return jacobian.transpose() * Q.inverse() * jacobian;
//     }

//     /**
//      * @brief Computes the total information of a point from its Fisher Information Matrix (FIM).
//      *
//      * This function computes the total information of a point transformation from its Fisher Information Matrix (FIM).
//      * The total information is calculated as the trace of the FIM.
//      *
//      * @param FIM The Fisher Information Matrix (FIM) for the point, represented as a 6x6 matrix.
//      * @return The total information of the point.
//      */
//     double computeInformationOfPoint(Eigen::Matrix<double, 6, 6> FIM)
//     {
//         return FIM.trace();
//     }

//     /**
//      * @brief Computes the total information of a point based on its measurements and transforms.
//      *
//      * This function computes the total information of a 3D point based on its measurements,
//      * the transform between the world and camera (T_w_c), the estimated transform between
//      * the world and camera (T_w_c_est), and the covariance matrix Q.
//      *
//      * @param p3d_c4_measurement The 4D homogeneous coordinates of the 3D point measurement in the camera frame.
//      * @param T_w_c The transform between the world and camera, represented as geometry_msgs::msg::Transform.
//      * @param T_w_c_est The estimated transform between the world and camera, represented as geometry_msgs::msg::Transform.
//      * @param Q The covariance matrix of the measurement, represented as a 3x3 Eigen::Matrix3d.
//      * @return The total information of the point.
//      */
//     double computeInformationOfPoint(Eigen::Vector4d &p3d_c4_measurement, geometry_msgs::msg::Transform &T_w_c,
//                                      geometry_msgs::msg::Transform &T_w_c_est, Eigen::Matrix3d &Q)
//     {
//         auto jac = computeJacobianForPoint(p3d_c4_measurement, T_w_c, T_w_c_est);
//         auto fim = computeFIM(jac, Q);
//         return computeInformationOfPoint(fim);
//     }

//     /**
//      * @brief Converts a pose to a transform.
//      *
//      * This function converts a geometry_msgs::msg::Pose to a geometry_msgs::msg::Transform.
//      *
//      * @param pose The input pose to be converted.
//      * @return The resulting transform.
//      */
//     geometry_msgs::msg::Transform getTransformFromPose(geometry_msgs::msg::Pose &pose)
//     {
//         geometry_msgs::msg::Transform transform;
//         transform.translation.x = pose.position.x;
//         transform.translation.y = pose.position.y;
//         transform.translation.z = pose.position.z;

//         transform.rotation = pose.orientation;
//         return transform;
//     }

//     /**
//      * @brief Retrieves node data and optimal transform for a given node ID from map data.
//      *
//      * This function searches for node data and optimal transform corresponding to the specified node ID
//      * within the provided map data.
//      *
//      * This is done because the optimal transform and the data are in different arrays.
//      *
//      * @param node_id The ID of the node to retrieve data for.
//      * @param map_data The map data containing node and pose information.
//      * @param logger_ The logger for error messages.
//      * @return A pair containing the node data and optimal transform if found, otherwise null pointers.
//      */
//     std::pair<slam_msgs::msg::KeyFrame, geometry_msgs::msg::Transform> getNodeDataAndOptTransform(int node_id, rtabmap_msgs::msg::MapData &map_data, rclcpp::Logger logger_)
//     {
//         geometry_msgs::msg::Transform::SharedPtr T_w_c;
//         slam_msgs::msg::KeyFrame::SharedPtr node_data;
//         for (int i = 0; i < map_data.graph.poses.size(); i++)
//         {
//             if (node_id == map_data.graph.poses_id[i])
//             {
//                 T_w_c = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(map_data.graph.poses[i]));
//                 break;
//             }
//         }
//         for (int i = 0; i < map_data.nodes.size(); i++)
//         {
//             if (node_id == map_data.nodes[i].id)
//             {
//                 node_data = std::make_shared<slam_msgs::msg::KeyFrame>(map_data.nodes[i]);
//                 break;
//             }
//         }
//         if (T_w_c && node_data)
//         {
//             return std::make_pair(*node_data, *T_w_c);
//         }
//         RCLCPP_ERROR(logger_, "The pointer does not have data. Null Pointer");
//         return std::make_pair(*node_data, *T_w_c);
//     }

//     /**
//      * @brief Computes information for a given pose based on its neighboring poses.
//      *
//      * This function computes information for a given pose by considering its neighboring poses
//      * within the specified camera frustum parameters and map data. Information is computed for
//      * points inside the frustum overlap region.
//      *
//      * @param pose The pose for which information is to be computed.
//      * @param neighbouring_poses The neighboring poses.
//      * @param neighbouring_ids The IDs of the neighboring poses.
//      * @param map_data The map data containing node and pose information.
//      * @param max_depth The maximum depth of observation for the camera.
//      * @param hfov The horizontal field of view of the camera.
//      * @param max_depth_error The maximum depth error.
//      * @param Q The covariance matrix Q.
//      * @param logger_ The logger for error messages.
//      * @return The computed information for the pose.
//      */
//     double computeInformationForPose(geometry_msgs::msg::Pose &pose,
//                                      std::vector<geometry_msgs::msg::Pose> &neighbouring_poses, std::vector<int> neighbouring_ids,
//                                      rtabmap_msgs::msg::MapData &map_data, double max_depth, double hfov, double max_depth_error, Eigen::Matrix3d Q, rclcpp::Logger logger_)
//     {
//         double pose_information = 0;
//         geometry_msgs::msg::Transform::SharedPtr T_w_c_est;
//         T_w_c_est = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(pose));
//         auto vertices_pose = frontier_exploration_utils::getVerticesOfFrustum2D(pose, max_depth, hfov);
//         for (int i = 0; i < neighbouring_poses.size(); i++)
//         {
//             int node_id = neighbouring_ids[i];
//             if (frontier_exploration_utils::frustumOverlap(pose, neighbouring_poses[i], max_depth, hfov, max_depth_error))
//             {
//                 auto node_data_trnsfrm = getNodeDataAndOptTransform(node_id, map_data, logger_);
//                 // p3d_c_m : p3d_camera frame measurement.
//                 for (auto p3d_c_m : node_data_trnsfrm.first.word_pts)
//                 {
//                     Eigen::Vector4d point_4_c_m(p3d_c_m.x, p3d_c_m.y, p3d_c_m.z, 1.0);
//                     std::vector<double> point_c_m = {p3d_c_m.x, p3d_c_m.y, p3d_c_m.z};
//                     Eigen::Vector4d point_4_w_m = computeTransformationMatrix(node_data_trnsfrm.second) * point_4_c_m;
//                     std::vector<double> point_w_m = {point_4_w_m(0), point_4_w_m(1), point_4_w_m(2)};
//                     if (frontier_exploration_utils::isPointInsideTriangle(point_w_m, vertices_pose))
//                     {
//                         pose_information += computeInformationOfPoint(point_4_c_m, node_data_trnsfrm.second, *T_w_c_est, Q);
//                     }
//                 }
//             }
//         }
//         return pose_information;
//     }

//     /**
//      * @brief Computes information for a given pose based on its neighboring poses.
//      *
//      * This function computes information for a given pose by considering its neighboring poses
//      * within the specified camera frustum parameters and map data. Information is computed for
//      * points inside the frustum overlap region.
//      *
//      * @param pose The pose for which information is to be computed.
//      * @param neighbouring_poses The neighboring poses.
//      * @param neighbouring_ids The IDs of the neighboring poses.
//      * @param map_data The map data containing node and pose information.
//      * @param max_depth The maximum depth of observation for the camera.
//      * @param hfov The horizontal field of view of the camera.
//      * @param max_depth_error The maximum depth error.
//      * @param Q The covariance matrix Q.
//      * @param pcl_return Flag indicating whether to return point cloud data.
//      * @param logger_ The logger for error messages.
//      * @param costmap_ The costmap for indexing information.
//      * @return A pair containing the computed information for the pose and optional point cloud data for visualization.
//      */
//     std::pair<double, std::vector<std::vector<double>>> computeInformationForPose(geometry_msgs::msg::Pose &pose,
//                                                                                   std::vector<geometry_msgs::msg::Pose> &neighbouring_poses, std::vector<int> neighbouring_ids,
//                                                                                   rtabmap_msgs::msg::MapData &map_data, double max_depth, double hfov, double max_depth_error, Eigen::Matrix3d Q, bool pcl_return, rclcpp::Logger logger_,
//                                                                                   nav2_costmap_2d::Costmap2D *costmap_)
//     {
//         // Variable used to hold the computed information to avoid recomputation.
//         std::map<unsigned int, double> information_map;
//         double pose_information = 0;
//         std::vector<std::vector<double>> point_w_m_arr;
//         geometry_msgs::msg::Transform::SharedPtr T_w_c_est;
//         T_w_c_est = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(pose));
//         auto vertices_pose = frontier_exploration_utils::getVerticesOfFrustum2D(pose, max_depth, hfov);
//         for (int i = 0; i < neighbouring_poses.size(); i++)
//         {
//             int node_id = neighbouring_ids[i];
//             if (frontier_exploration_utils::frustumOverlap(pose, neighbouring_poses[i], max_depth, hfov, max_depth_error))
//             {
//                 auto node_data_trnsfrm = getNodeDataAndOptTransform(node_id, map_data, logger_);
//                 // p3d_c_m : p3d_camera frame measurement.
//                 for (auto p3d_c_m : node_data_trnsfrm.first.word_pts)
//                 {
//                     Eigen::Vector4d point_4_c_m(p3d_c_m.x, p3d_c_m.y, p3d_c_m.z, 1.0);
//                     std::vector<double> point_c_m = {p3d_c_m.x, p3d_c_m.y, p3d_c_m.z};
//                     Eigen::Vector4d point_4_w_m = computeTransformationMatrix(node_data_trnsfrm.second) * point_4_c_m;
//                     std::vector<double> point_w_m = {point_4_w_m(0), point_4_w_m(1), point_4_w_m(2)};
//                     if (frontier_exploration_utils::isPointInsideTriangle(point_w_m, vertices_pose))
//                     {
//                         unsigned int mx, my;
//                         if (costmap_->worldToMap(point_4_w_m(0), point_4_w_m(1), mx, my))
//                         {
//                             auto index = costmap_->getIndex(mx, my);
//                             std::map<unsigned int, double>::iterator it = information_map.find(index);
//                             // if information for a certain index is computed, just add it. Do not recompute.
//                             if (it != information_map.end())
//                             {
//                                 pose_information += information_map[index];
//                             }
//                             else
//                             {
//                                 information_map[index] = computeInformationOfPoint(point_4_c_m, node_data_trnsfrm.second, *T_w_c_est, Q);
//                                 pose_information += information_map[index];
//                                 point_w_m_arr.push_back(point_w_m);
//                                 // Set the information of surrounding indices to the computed value. To avoid recomputation.
//                                 information_map[index + 1] = information_map[index];
//                                 information_map[index - 1] = information_map[index];
//                                 information_map[index + 2] = information_map[index];
//                                 information_map[index - 2] = information_map[index];

//                                 information_map[index + costmap_->getSizeInCellsX()] = information_map[index];
//                                 information_map[index + costmap_->getSizeInCellsX() + 1] = information_map[index];
//                                 information_map[index + costmap_->getSizeInCellsX() - 1] = information_map[index];
//                                 information_map[index + costmap_->getSizeInCellsX() + 2] = information_map[index];
//                                 information_map[index + costmap_->getSizeInCellsX() - 2] = information_map[index];
//                                 information_map[index - costmap_->getSizeInCellsX()] = information_map[index];
//                                 information_map[index - costmap_->getSizeInCellsX() + 1] = information_map[index];
//                                 information_map[index - costmap_->getSizeInCellsX() - 1] = information_map[index];
//                                 information_map[index - costmap_->getSizeInCellsX() + 2] = information_map[index];
//                                 information_map[index - costmap_->getSizeInCellsX() - 2] = information_map[index];

//                                 information_map[index + 2 * costmap_->getSizeInCellsX()] = information_map[index];
//                                 information_map[index + 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
//                                 information_map[index + 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
//                                 information_map[index + 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
//                                 information_map[index + 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
//                                 information_map[index - 2 * costmap_->getSizeInCellsX()] = information_map[index];
//                                 information_map[index - 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
//                                 information_map[index - 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
//                                 information_map[index - 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
//                                 information_map[index - 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//         return std::make_pair(pose_information, point_w_m_arr);
//     }
// }

namespace frontier_exploration_planning
{

    double distanceBetweenTwoPoints(geometry_msgs::msg::Point point_from, geometry_msgs::msg::Point point_to)
    {
        // Calculate the differences in X, Y, and Z coordinates
        double dx = point_to.x - point_from.x;
        double dy = point_to.y - point_from.y;

        // Calculate the Euclidean distance
        double distance = std::sqrt(dx * dx + dy * dy);

        return distance;
    }

    std::pair<std::vector<geometry_msgs::msg::Pose>, std::vector<int>> getNodesInRadius(std::vector<geometry_msgs::msg::PoseStamped> &poses, std::vector<int> &poses_id, double radius, geometry_msgs::msg::Pose &start_pose, rclcpp::Logger logger_)
    {
        std::vector<geometry_msgs::msg::Pose> neighbouring_poses;
        std::vector<int> neighbouring_ids;
        for (int k = 0; k < poses.size(); k++)
        {
            if (distanceBetweenTwoPoints(start_pose.position, poses[k].pose.position) <= radius)
            {
                neighbouring_poses.push_back(poses[k].pose);
                neighbouring_ids.push_back(poses_id[k]);
            }
        }
        return std::make_pair(neighbouring_poses, neighbouring_ids);
    }
}

// ===============================


namespace frontier_exploration_information_affine
{

    /**
     * @brief Computes the skew-symmetric matrix for a 3D vector.
     *
     * This function calculates the skew-symmetric matrix representation of a 3D vector.
     * The skew-symmetric matrix has the property that when multiplied by another vector,
     * it produces the cross product of the original vector and the other vector.
     *
     * @param v The 3D vector.
     * @return The skew-symmetric matrix corresponding to the input vector.
     *         The last entry of the input vector is not considered for computation.
     * @note The last element in v is 1. It is not used for computation.
     */
    Eigen::Matrix3f getSkewMatrix(const Eigen::Vector3f &v)
    {
        Eigen::Matrix3f skewMat;
        skewMat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return skewMat;
    }

    /**
     * @brief Transforms a 3D point from camera frame to world frame using a transformation matrix.
     *
     * This function performs the transformation of a 3D point from the camera frame to the world frame
     * using the provided transformation matrix T_w_c.
     *
     * @param p3d_c The 3D point in the camera frame represented as a 4D homogeneous vector.
     * @param T_w_c The transformation matrix from camera frame to world frame.
     * @return The transformed 3D point in the world frame represented as a 4D homogeneous vector.
     */
    Eigen::Vector4d transformPoint_C_W(Eigen::Vector4d &p3d_c, const Eigen::Matrix4d &T_w_c)
    {
        return T_w_c * p3d_c;
    }

    /**
     * @brief Computes the Jacobian matrix for a 3D point transformation.
     *
     * This function computes the Jacobian matrix for transforming a 3D point from camera frame to world frame
     * with respect to the given camera-to-world transformation (T_w_c) and the estimated camera-to-world
     * transformation (T_w_c_est). The Jacobian matrix represents the sensitivity of the transformed point
     * to changes in the camera-to-world transformation.
     *
     * @param p3d_c4_measurement The 3D point in the camera frame represented as a 4D homogeneous vector.
     * @param T_w_c The transformation from camera frame to world frame represented as a geometry_msgs::msg::Transform.
     * @param T_w_c_est The estimated transformation from camera frame to world frame represented as a geometry_msgs::msg::Transform.
     * @return The Jacobian matrix for the point transformation, represented as a 3x6 matrix.
     */
    Eigen::Matrix<float, 3, 6> computeJacobianForPoint(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est)
    {
        // convert the world coordinates to camera coordinates of the pose used for estimation.
        Eigen::Vector3f p3d_c_eig_est = T_w_c_est.inverse() * p3d_w_eig;

        // df_dp
        const float n = p3d_c_eig_est.norm();
        Eigen::Matrix3f df_dpc = (1 / n) * Eigen::Matrix3f::Identity() -
                                 (1 / (n * n * n)) * p3d_c_eig_est * p3d_c_eig_est.transpose();

        // dp_dTwc
        Eigen::Matrix<float, 3, 3> leftMat = T_w_c_est.inverse().rotation();
        Eigen::Matrix<float, 3, 6> rightMat;
        rightMat.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
        rightMat.block<3, 3>(0, 3) = (-1.0) * getSkewMatrix(p3d_w_eig);
        Eigen::Matrix<float, 3, 6> dpc_dtwc = leftMat * rightMat;

        Eigen::Matrix<float, 3, 6> jacobian = df_dpc * dpc_dtwc;
        // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
        return jacobian;
    }

    /**
     * @brief Computes the Fisher Information Matrix (FIM) for a point.
     *
     * This function computes the Fisher Information Matrix (FIM) for a 3D point transformation
     * based on the provided Jacobian matrix and the covariance matrix Q. The FIM quantifies
     * the amount of information provided by the point transformation.
     *
     * @param jacobian The Jacobian matrix for the point transformation, represented as a 3x6 matrix.
     * @param Q The covariance matrix, represented as a 3x3 Eigen::Matrix3f.
     * @return The Fisher Information Matrix (FIM) for the point, represented as a 6x6 matrix.
     */
    Eigen::Matrix<float, 6, 6> computeFIM(Eigen::Matrix<float, 3, 6> &jacobian, Eigen::Matrix3f &Q)
    {
        return jacobian.transpose() * Q.inverse() * jacobian;
    }

    /**
     * @brief Computes the total information of a point from its Fisher Information Matrix (FIM).
     *
     * This function computes the total information of a point transformation from its Fisher Information Matrix (FIM).
     * The total information is calculated as the trace of the FIM.
     *
     * @param FIM The Fisher Information Matrix (FIM) for the point, represented as a 6x6 matrix.
     * @return The total information of the point.
     */
    float computeInformationOfPoint(Eigen::Matrix<float, 6, 6> FIM)
    {
        return FIM.trace();
    }

    /**
     * @brief Computes the total information of a point based on its measurements and transforms.
     *
     * This function computes the total information of a 3D point based on its measurements,
     * the transform between the world and camera (T_w_c), the estimated transform between
     * the world and camera (T_w_c_est), and the covariance matrix Q.
     *
     * @param p3d_c4_measurement The 4D homogeneous coordinates of the 3D point measurement in the camera frame.
     * @param T_w_c The transform between the world and camera, represented as geometry_msgs::msg::Transform. This is the transform from the pose graph.
     * @param T_w_c_est The transform between the world and camera used for estimation, represented as geometry_msgs::msg::Transform. 
     *                  This is the transform obtained from the pose of the point in the path.
     * @param Q The covariance matrix of the measurement, represented as a 3x3 Eigen::Matrix3f.
     * @return The total information of the point.
     */
    float computeInformationOfPoint(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                     Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f &Q)
    {
        auto jac = computeJacobianForPoint(p3d_c_eig, p3d_w_eig, T_w_c_est);
        auto fim = computeFIM(jac, Q);
        return computeInformationOfPoint(fim);
    }

    /**
     * @brief Converts a pose to a transform.
     *
     * This function converts a geometry_msgs::msg::Pose to a geometry_msgs::msg::Transform.
     *
     * @param pose The input pose to be converted.
     * @return The resulting transform.
     */
    Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose &pose)
    {
        // Extract translation and rotation from the pose message
        Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        // Construct the transformation matrix
        Eigen::Affine3f transform = Eigen::Translation3f(translation) * Eigen::Quaternionf(rotation);

        return transform;
    }

    /**
     * @brief Retrieves node data and optimal transform for a given node ID from map data.
     *
     * This function searches for node data and optimal transform corresponding to the specified node ID
     * within the provided map data.
     *
     * @param node_id The ID of the node to retrieve data for.
     * @param map_data The map data containing node and pose information.
     * @param logger_ The logger for error messages.
     * @return A pair containing the node data and optimal transform if found, otherwise null pointers.
     */
    std::pair<slam_msgs::msg::KeyFrame, Eigen::Affine3f> getNodeDataAndOptTransform(int node_id, slam_msgs::msg::MapData &map_data, rclcpp::Logger logger_)
    {
        std::shared_ptr<Eigen::Affine3f> T_w_c;
        slam_msgs::msg::KeyFrame::SharedPtr node_data;
        for (int i = 0; i < map_data.graph.poses.size(); i++)
        {
            if (node_id == map_data.graph.poses_id[i])
            {
                T_w_c = std::make_shared<Eigen::Affine3f>(getTransformFromPose(map_data.graph.poses[i].pose));
                break;
            }
        }
        for (int i = 0; i < map_data.nodes.size(); i++)
        {
            if (node_id == map_data.nodes[i].id)
            {
                node_data = std::make_shared<slam_msgs::msg::KeyFrame>(map_data.nodes[i]);
                break;
            }
        }
        if (T_w_c && node_data)
        {
            return std::make_pair(*node_data, *T_w_c);
        }
        RCLCPP_ERROR(logger_, "The pointer does not have data. Null Pointer");
        return std::make_pair(*node_data, *T_w_c);
    }

    /**
     * @brief Computes information for a given pose based on its neighboring poses.
     *
     * This function computes information for a given pose by considering its neighboring poses
     * within the specified camera frustum parameters and map data. Information is computed for
     * points inside the frustum overlap region.
     *
     * @param pose The pose for which information is to be computed.
     * @param neighbouring_poses The neighboring poses.
     * @param neighbouring_ids The IDs of the neighboring poses.
     * @param map_data The map data containing node and pose information.
     * @param max_depth The maximum depth of observation for the camera.
     * @param hfov The horizontal field of view of the camera.
     * @param max_depth_error The maximum depth error.
     * @param Q The covariance matrix Q.
     * @param pcl_return Flag indicating whether to return point cloud data.
     * @param logger_ The logger for error messages.
     * @param costmap_ The costmap for indexing information.
     * @return A pair containing the computed information for the pose and optional point cloud data for visualization.
     */
    std::pair<float, std::vector<Eigen::Vector3f>> computeInformationForPose(geometry_msgs::msg::Pose &pose,
                                                                                  std::vector<geometry_msgs::msg::Pose> &neighbouring_poses, std::vector<int> neighbouring_ids,
                                                                                  slam_msgs::msg::MapData &map_data, double max_depth, double hfov, double max_depth_error, Eigen::Matrix3f Q, bool pcl_return, rclcpp::Logger logger_,
                                                                                  nav2_costmap_2d::Costmap2D *costmap_, bool affine)
    {
        // Variable used to hold the computed information to avoid recomputation.
        std::map<unsigned int, float> information_map;
        float pose_information = 0;
        std::vector<Eigen::Vector3f> point_w_m_arr;
        std::shared_ptr<Eigen::Affine3f> T_w_c_est;
        T_w_c_est = std::make_shared<Eigen::Affine3f>(getTransformFromPose(pose));
        auto vertices_pose = frontier_exploration_utils::getVerticesOfFrustum2D(pose, max_depth, hfov);
        for (int i = 0; i < neighbouring_poses.size(); i++)
        {
            int node_id = neighbouring_ids[i];
            if (frontier_exploration_utils::frustumOverlap(pose, neighbouring_poses[i], max_depth, hfov, max_depth_error))
            {
                // transform in this return the camera pose in world frame.
                auto node_data_trnsfrm = getNodeDataAndOptTransform(node_id, map_data, logger_);
                // p3d_w_opt : the optimized point in the world frame. (geometry_msgs::msg::Vector3f)
                for (auto p3d_w : node_data_trnsfrm.first.word_pts)
                {
                    Eigen::Vector3f p3d_w_eig(p3d_w.x, p3d_w.y, p3d_w.z);
                    if (frontier_exploration_utils::isPointInsideTriangle(p3d_w_eig, vertices_pose))
                    {
                        unsigned int mx, my;
                        if (costmap_->worldToMap(p3d_w_eig(0), p3d_w_eig(1), mx, my))
                        {
                            auto index = costmap_->getIndex(mx, my);
                            std::map<unsigned int, float>::iterator it = information_map.find(index);
                            // if information for a certain index is computed, just add it. Do not recompute.
                            if (it != information_map.end())
                            {
                                pose_information += information_map[index];
                            }
                            else
                            {
                                // TODO: important!: Convert p3d_w_eig to camera coord before sending here.
                                auto p3d_c_eig = node_data_trnsfrm.second.inverse() * p3d_w_eig;
                                information_map[index] = computeInformationOfPoint(p3d_c_eig, p3d_w_eig, *T_w_c_est, Q);
                                pose_information += information_map[index];
                                point_w_m_arr.push_back(p3d_w_eig);
                                // Set the information of surrounding indices to the computed value. To avoid recomputation.
                                // information_map[index + 1] = information_map[index];
                                // information_map[index - 1] = information_map[index];
                                // information_map[index + 2] = information_map[index];
                                // information_map[index - 2] = information_map[index];

                                // information_map[index + costmap_->getSizeInCellsX()] = information_map[index];
                                // information_map[index + costmap_->getSizeInCellsX() + 1] = information_map[index];
                                // information_map[index + costmap_->getSizeInCellsX() - 1] = information_map[index];
                                // information_map[index + costmap_->getSizeInCellsX() + 2] = information_map[index];
                                // information_map[index + costmap_->getSizeInCellsX() - 2] = information_map[index];
                                // information_map[index - costmap_->getSizeInCellsX()] = information_map[index];
                                // information_map[index - costmap_->getSizeInCellsX() + 1] = information_map[index];
                                // information_map[index - costmap_->getSizeInCellsX() - 1] = information_map[index];
                                // information_map[index - costmap_->getSizeInCellsX() + 2] = information_map[index];
                                // information_map[index - costmap_->getSizeInCellsX() - 2] = information_map[index];

                                // information_map[index + 2 * costmap_->getSizeInCellsX()] = information_map[index];
                                // information_map[index + 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
                                // information_map[index + 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
                                // information_map[index + 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
                                // information_map[index + 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
                                // information_map[index - 2 * costmap_->getSizeInCellsX()] = information_map[index];
                                // information_map[index - 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
                                // information_map[index - 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
                                // information_map[index - 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
                                // information_map[index - 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
                            }
                        }
                    }
                }
            }
        }
        return std::make_pair(pose_information, point_w_m_arr);
    }

}




#endif