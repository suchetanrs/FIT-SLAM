#ifndef FRONTIER_EXPLORATION_UTIL_H_
#define FRONTIER_EXPLORATION_UTIL_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rtabmap_msgs/msg/node_data.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>

namespace frontier_exploration_utils {

    // @brief Checks if a 2D point is inside a triangle. Triangle vertices size = 3 and order = x, y 
    bool isPointInsideTriangle(std::vector<double> point, std::vector<std::vector<double>> triangle_vertices) {
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

    // @brief the geometry_msgs quaternion to tf2 quaternion
    // @return roll, pitch, yaw
    // TODO: Verify if this quaternion has to be normalized.
    std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf2_quaternion(
            quat.x, quat.y, quat.z, quat.w);

        // Convert tf2 quaternion to Euler angles
        tf2::Matrix3x3 matrix(tf2_quaternion);
        std::vector<double> rpy = {0, 0, 0};
        matrix.getRPY(rpy[0], rpy[1], rpy[2]);
        return rpy;
    }
    
    // @brief This function returns vertices of a triangle given the pose, max_depth of observation and hfov. These parameters can be found in the URDF of the depth camera.
    std::vector<std::vector<double>> getVerticesOfFrustum2D (geometry_msgs::msg::Pose& pose, double max_depth, double hfov) {
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

    // @brief This function returns the mid points of the 3 sides of a 2D triangle along with its vertices.
    // Modify this function if you want to check for more points other than just these 6.
    std::vector<std::vector<double>> getVerticesToCheck (geometry_msgs::msg::Pose& pose, double max_depth, double hfov) {
        std::vector<std::vector<double>> vertices = getVerticesOfFrustum2D (pose, max_depth, hfov);

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

    // @brief Checks if the camera frustums of two poses have any overlap. The max_depth_error is the value to be added to max_depth of the 3D camera.
    // If the error is 0.5, the camera fov triangle will have a height of max_depth + max_depth_error.
    bool frustumOverlap(geometry_msgs::msg::Pose& curr_pose, geometry_msgs::msg::Pose& pose_to_check, double max_depth, double hfov, double max_depth_error) {
        auto vertices_pose = getVerticesOfFrustum2D(curr_pose, max_depth + max_depth_error, hfov);
        auto vertices_check = getVerticesToCheck(pose_to_check, max_depth + max_depth_error, hfov);
        for (auto pose : vertices_check) {
            if(isPointInsideTriangle(pose, vertices_pose)) {
                return true;
            }
        }
        return false;
    }

    geometry_msgs::msg::Point getPointFromVector(std::vector<double> vec) {
        geometry_msgs::msg::Point pnt;
        pnt.x = vec[0];
        pnt.y = vec[1];
        return pnt;
    }
}

namespace frontier_exploration_information {

    // last entry is 1. Does not consider the last entry for computation
    Eigen::Matrix3d getSkewMatrix(const Eigen::Vector4d& v) {
        Eigen::Matrix3d skewMat;
        skewMat <<  0, -v(2),  v(1),
                    v(2),  0,  -v(0),
                    -v(1),  v(0),  0;
        return skewMat;
    }


    Eigen::Matrix4d computeTransformationMatrix(geometry_msgs::msg::Transform& transform) {
        Eigen::Quaterniond quaternion(
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z
        );
        quaternion.normalize();
        Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
        Eigen::Matrix<double, 4, 1> translation_matrix;
        translation_matrix << transform.translation.x, transform.translation.y, transform.translation.z, 1;
        Eigen::Matrix<double, 4, 3> rt_mat;
        rt_mat.topRows(3) = rotation_matrix;
        rt_mat.bottomRows(1).setZero();

        Eigen::Matrix4d transformation_matrix;

        transformation_matrix << rt_mat, translation_matrix;
        return transformation_matrix;
    }


    Eigen::Matrix3d computeRotationMatrix(const Eigen::Matrix4d& transformation_matrix) {
        return transformation_matrix.block<3, 3>(0, 0);
    }


    Eigen::Vector4d transformPoint_C_W(Eigen::Vector4d& p3d_c, const Eigen::Matrix4d& T_w_c) {
        return T_w_c * p3d_c;
    }


    Eigen::Vector4d transformPoint_C_W(Eigen::Vector4d& p3d_c, geometry_msgs::msg::Transform& T_w_c) {
        return computeTransformationMatrix(T_w_c) * p3d_c;
    }

    // @brief T_w_c is the transform between world and camera with which the point in the camera frame can be converted to world frame.
    // T_w_c_est is the transform between world and the camera pose at which you wish to estimate the information of the pixel.
    Eigen::Matrix<double, 3, 6> computeJacobianForPoint(Eigen::Vector4d& p3d_c4_measurement, geometry_msgs::msg::Transform& T_w_c, geometry_msgs::msg::Transform& T_w_c_est) {
        Eigen::Matrix4d transformation_matrix_w_c = computeTransformationMatrix(T_w_c);
        Eigen::Vector4d p3d_w4 = transformation_matrix_w_c * p3d_c4_measurement;

        Eigen::Matrix4d transformation_matrix_w_c_est = computeTransformationMatrix(T_w_c_est);
        Eigen::Vector4d p3d_c4 = transformation_matrix_w_c_est.inverse() * p3d_w4;

        // df_dp
        const double n = p3d_c4.head<3>().norm();
        Eigen::Matrix3d df_dpc = (1 / n) * Eigen::Matrix3d::Identity() -
                                (1 / (n * n * n)) * p3d_c4.head<3>() * p3d_c4.head<3>().transpose();


        // dp_dTwc
        Eigen::Matrix<double, 3, 6> rightMat;
        rightMat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        rightMat.block<3, 3>(0, 3) = (-1.0) * getSkewMatrix(p3d_w4);
        Eigen::Matrix<double, 3, 3> leftMat;
        leftMat = computeRotationMatrix(transformation_matrix_w_c_est).inverse();
        Eigen::Matrix<double, 3, 6> dpc_dtwc = leftMat * rightMat;

        Eigen::Matrix<double, 3, 6> jacobian = df_dpc * dpc_dtwc;
        // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
        return jacobian;
    }


    Eigen::Matrix<double, 6, 6> computeFIM(Eigen::Matrix<double, 3, 6>& jacobian, Eigen::Matrix3d& Q) {
        return jacobian.transpose() * Q.inverse() * jacobian;
    }


    double computeInformationOfPoint(Eigen::Matrix<double, 6, 6> FIM) {
        return FIM.trace();
    }


// NEW
    double computeInformationOfPoint(Eigen::Vector4d& p3d_c4_measurement, geometry_msgs::msg::Transform& T_w_c,
                                    geometry_msgs::msg::Transform& T_w_c_est, Eigen::Matrix3d& Q) {
        auto jac = computeJacobianForPoint(p3d_c4_measurement, T_w_c, T_w_c_est);
        auto fim = computeFIM(jac, Q);
        return computeInformationOfPoint(fim);
    }

    geometry_msgs::msg::Transform getTransformFromPose(geometry_msgs::msg::Pose& pose) {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = pose.position.x;
        transform.translation.y = pose.position.y;
        transform.translation.z = pose.position.z;

        transform.rotation = pose.orientation;
        return transform;
    }


    std::pair<rtabmap_msgs::msg::NodeData, geometry_msgs::msg::Transform> getNodeDataAndOptTransform(int node_id, rtabmap_msgs::msg::MapData& map_data, rclcpp::Logger logger_) {
        geometry_msgs::msg::Transform::SharedPtr T_w_c;
        rtabmap_msgs::msg::NodeData::SharedPtr node_data;
        for (int i =0; i < map_data.graph.poses.size(); i++) {
            if(node_id == map_data.graph.poses_id[i]) {
                T_w_c = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(map_data.graph.poses[i]));
                // RCLCPP_INFO(logger_, "Checkpoint 3.7");
                break;
            }
        }
        // RCLCPP_INFO(logger_, "Checkpoint 3.75");
        for (int i =0; i < map_data.nodes.size(); i++) {
            if(node_id == map_data.nodes[i].id) {
                node_data = std::make_shared<rtabmap_msgs::msg::NodeData>(map_data.nodes[i]);
                // RCLCPP_INFO(logger_, "Checkpoint 3.8");
                break;
            }
        }
        // RCLCPP_INFO(logger_, "Checkpoint 3.85");
        if(T_w_c && node_data) {
            // RCLCPP_INFO(logger_, "Checkpoint 3.9");
            return std::make_pair(*node_data, *T_w_c);
        }
        RCLCPP_ERROR(logger_, "The pointer does not have data. Null Pointer");
        // RCLCPP_INFO(logger_, "Checkpoint 3.95");
        return std::make_pair(*node_data, *T_w_c);
        // TODO: Find a way to print error message below this. The function should never complete the for loop without returning.
    }

    double computeInformationForPose(geometry_msgs::msg::Pose& pose,
    std::vector<geometry_msgs::msg::Pose>& neighbouring_poses, std::vector<int> neighbouring_ids, 
    rtabmap_msgs::msg::MapData& map_data, double max_depth, double hfov, double max_depth_error, Eigen::Matrix3d Q, rclcpp::Logger logger_)
    {
        double pose_information = 0;
        geometry_msgs::msg::Transform::SharedPtr T_w_c_est;
        T_w_c_est = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(pose));
        auto vertices_pose = frontier_exploration_utils::getVerticesOfFrustum2D(pose, max_depth, hfov);
        for(int i=0; i < neighbouring_poses.size(); i++) {
            int node_id = neighbouring_ids[i];
            if(frontier_exploration_utils::frustumOverlap(pose, neighbouring_poses[i], max_depth, hfov, max_depth_error)) {
                auto node_data_trnsfrm = getNodeDataAndOptTransform(node_id, map_data, logger_);
                // p3d_c_m : p3d_camera frame measurement.
                for(auto p3d_c_m : node_data_trnsfrm.first.word_pts) {
                    Eigen::Vector4d point_4_c_m(p3d_c_m.x, p3d_c_m.y, p3d_c_m.z, 1.0);
                    std::vector<double> point_c_m = {p3d_c_m.x, p3d_c_m.y, p3d_c_m.z};
                    Eigen::Vector4d point_4_w_m = computeTransformationMatrix(node_data_trnsfrm.second) * point_4_c_m;
                    std::vector<double> point_w_m = {point_4_w_m(0), point_4_w_m(1), point_4_w_m(2)};
                    if(frontier_exploration_utils::isPointInsideTriangle(point_w_m, vertices_pose)) {
                        pose_information += computeInformationOfPoint(point_4_c_m, node_data_trnsfrm.second, *T_w_c_est, Q);
                    }
                }
            }
        }
        return pose_information;
    }

    std::pair<double, std::vector<std::vector<double>>> computeInformationForPose(geometry_msgs::msg::Pose& pose,
    std::vector<geometry_msgs::msg::Pose>& neighbouring_poses, std::vector<int> neighbouring_ids, 
    rtabmap_msgs::msg::MapData& map_data, double max_depth, double hfov, double max_depth_error, Eigen::Matrix3d Q, bool pcl_return, rclcpp::Logger logger_,
    nav2_costmap_2d::Costmap2D* costmap_)
    {   
        std::map<unsigned int, double> information_map;
        // RCLCPP_INFO(logger_, "Checkpoint 0");
        double pose_information = 0;
        std::vector<std::vector<double>> point_w_m_arr;
        geometry_msgs::msg::Transform::SharedPtr T_w_c_est;
        T_w_c_est = std::make_shared<geometry_msgs::msg::Transform>(getTransformFromPose(pose));
        auto vertices_pose = frontier_exploration_utils::getVerticesOfFrustum2D(pose, max_depth, hfov);
        // RCLCPP_INFO(logger_, "Checkpoint 1");
        for(int i=0; i < neighbouring_poses.size(); i++) {
            int node_id = neighbouring_ids[i];
            // RCLCPP_INFO(logger_, "Checkpoint 2");
            if(frontier_exploration_utils::frustumOverlap(pose, neighbouring_poses[i], max_depth, hfov, max_depth_error)) {
                // RCLCPP_INFO(logger_, "Checkpoint 3");
                auto node_data_trnsfrm = getNodeDataAndOptTransform(node_id, map_data, logger_);
                // p3d_c_m : p3d_camera frame measurement.
                // checkpoint
                // RCLCPP_INFO(logger_, "Checkpoint 3.95");
                for(auto p3d_c_m : node_data_trnsfrm.first.word_pts) {
                    Eigen::Vector4d point_4_c_m(p3d_c_m.x, p3d_c_m.y, p3d_c_m.z, 1.0);
                    std::vector<double> point_c_m = {p3d_c_m.x, p3d_c_m.y, p3d_c_m.z};
                    Eigen::Vector4d point_4_w_m = computeTransformationMatrix(node_data_trnsfrm.second) * point_4_c_m;
                    std::vector<double> point_w_m = {point_4_w_m(0), point_4_w_m(1), point_4_w_m(2)};
                    if(frontier_exploration_utils::isPointInsideTriangle(point_w_m, vertices_pose)) {
                        // RCLCPP_INFO(logger_, "Checkpoint 6");
                        // pose_information += computeInformationOfPoint(point_4_c_m, node_data_trnsfrm.second, *T_w_c_est, Q);


                        unsigned int mx, my;
                        if(costmap_->worldToMap(point_4_w_m(0), point_4_w_m(1), mx, my)) {
                            // RCLCPP_INFO(logger_, "Checkpoint 6");
                            auto index = costmap_->getIndex(mx, my);
                            std::map<unsigned int, double>::iterator it = information_map.find(index);
                            if(it != information_map.end()) {
                                pose_information += information_map[index];
                                // RCLCPP_INFO(logger_, "Checkpoint 5");
                            }
                            else {
                                information_map[index] = computeInformationOfPoint(point_4_c_m, node_data_trnsfrm.second, *T_w_c_est, Q);
                                pose_information += information_map[index];
                                // RCLCPP_INFO(logger_, "Checkpoint 4");
                                point_w_m_arr.push_back(point_w_m);
                                information_map[index+1] = information_map[index];
                                information_map[index-1] = information_map[index];
                                information_map[index+2] = information_map[index];
                                information_map[index-2] = information_map[index];

                                information_map[index + costmap_->getSizeInCellsX()] = information_map[index];
                                information_map[index + costmap_->getSizeInCellsX() + 1] = information_map[index];
                                information_map[index + costmap_->getSizeInCellsX() - 1] = information_map[index];
                                information_map[index + costmap_->getSizeInCellsX() + 2] = information_map[index];
                                information_map[index + costmap_->getSizeInCellsX() - 2] = information_map[index];
                                information_map[index - costmap_->getSizeInCellsX()] = information_map[index];
                                information_map[index - costmap_->getSizeInCellsX() + 1] = information_map[index];
                                information_map[index - costmap_->getSizeInCellsX() - 1] = information_map[index];
                                information_map[index - costmap_->getSizeInCellsX() + 2] = information_map[index];
                                information_map[index - costmap_->getSizeInCellsX() - 2] = information_map[index];

                                information_map[index + 2 * costmap_->getSizeInCellsX()] = information_map[index];
                                information_map[index + 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
                                information_map[index + 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
                                information_map[index + 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
                                information_map[index + 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
                                information_map[index - 2 * costmap_->getSizeInCellsX()] = information_map[index];
                                information_map[index - 2 * costmap_->getSizeInCellsX() + 1] = information_map[index];
                                information_map[index - 2 * costmap_->getSizeInCellsX() - 1] = information_map[index];
                                information_map[index - 2 * costmap_->getSizeInCellsX() + 2] = information_map[index];
                                information_map[index - 2 * costmap_->getSizeInCellsX() - 2] = information_map[index];
                            }
                        }

                        // point_w_m_arr.push_back(point_w_m);
                    }
                }
            }
        }
        return std::make_pair(pose_information, point_w_m_arr);
    }
}

namespace frontier_exploration_planning {

    double distanceBetweenTwoPoints(geometry_msgs::msg::Point point_from, geometry_msgs::msg::Point point_to) {
        // Calculate the differences in X, Y, and Z coordinates
        double dx = point_to.x - point_from.x;
        double dy = point_to.y - point_from.y;

        // Calculate the Euclidean distance
        double distance = std::sqrt(dx * dx + dy * dy);

        return distance;
    }

    geometry_msgs::msg::Pose getRelativePoseGivenTwoPoints(geometry_msgs::msg::Point point_from, geometry_msgs::msg::Point point_to) {
        // size_t plan_size = plan.poses.size();
        // if (plan_size == 1) {
        //   plan.poses.back().pose.orientation = start.orientation;
        // } else if (plan_size > 1) {
        double dx, dy, theta;
        dx = point_to.x - point_from.x;
        dy = point_to.y - point_from.y;
        theta = atan2(dy, dx);
        geometry_msgs::msg::Pose oriented_pose;
        oriented_pose.position = point_from;
        oriented_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
        return oriented_pose;
    }

    std::pair<std::vector<geometry_msgs::msg::Pose>, std::vector<int>> getNodesInRadius(std::vector<geometry_msgs::msg::Pose>& poses, std::vector<int>& poses_id, double radius, geometry_msgs::msg::Pose& start_pose, rclcpp::Logger logger_) {
        std::vector<geometry_msgs::msg::Pose> neighbouring_poses;
        std::vector<int> neighbouring_ids;
        for(int k = 0; k < poses.size(); k++) {
            if(distanceBetweenTwoPoints(start_pose.position, poses[k].position) <= radius) {
                neighbouring_poses.push_back(poses[k]);
                neighbouring_ids.push_back(poses_id[k]);
            }
        }
        return std::make_pair(neighbouring_poses, neighbouring_ids);
    }
}

#endif