//
// Created by d.vivet on 19/04/23.
//

#ifndef TRAVERSABILITY_TRAVERSABILITYGRID_HPP_
#define TRAVERSABILITY_TRAVERSABILITYGRID_HPP_

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class NodeMetaData{

public:
    NodeMetaData(){
        z_min = 100.;
        z_max = -100.;
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void reset(){
        z_min = 100.;
        z_max = -100.;
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void insert(Eigen::Vector3d &p3d){
        N++;
        // deal with min max
        // TODO: Check logic of min and max.
        z_min = std::min(p3d.z(), z_min);
        z_max = std::max(p3d.z(), z_max);

        // update momentums
        sx += p3d.x();
        sy += p3d.y();
        sz += p3d.z();
        sx2 += p3d.x()*p3d.x();
        sy2 += p3d.y()*p3d.y();
        sz2 += p3d.z()*p3d.z();
        sxy += p3d.x()*p3d.y();
        sxz += p3d.x()*p3d.z();
        syz += p3d.y()*p3d.z();
    }

    unsigned int N=0;
    double z_min=0.;
    double z_max=0.;
    double sx = 0.;
    double sy = 0.;
    double sz = 0.;
    double sx2 = 0.;
    double sy2 = 0.;
    double sz2 =0.;
    double sxy =0.;
    double sxz =0.;
    double syz = 0.;
};



class traversabilityGrid {
public:
    traversabilityGrid(double resolution, Eigen::Vector2d ahalfside, std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> planeLMSPub)
            : _resolution(resolution), _halfside(ahalfside), planeLMSPub_(planeLMSPub) {

        // resize grid
        NodeMetaData default_value;
        double Nd_x = _halfside.x()/_resolution;
        double Nd_y = _halfside.y()/_resolution;
        size_x_ = 2.*(std::ceil(Nd_x))+1;
        size_y_ = 2.*(std::ceil(Nd_y))+1;
        _grid.resize(size_x_, std::vector<NodeMetaData>(size_y_, default_value));
        reset();
    }

    // @brief responsible for inserting a 3D point into the appropriate cell within the grid.
    // The appropriate cell here is that all points with the same x and y coordinates as the node in the grid map are grouped regardless of the z direction.
    void insert_data(Eigen::Vector3d &p3d);

    // @brief resets metadata for all cells in the grid.
    void reset(){ 
        for(int i=0; i < size_x_; ++i)
            for(int j=0; j < size_y_; ++j)
                _grid.at(i).at(j).reset();
        pose_array_msg.poses.clear();
    }

    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates can be in meters.
    Eigen::VectorXd get_goodness_m(Eigen::Vector2d meters, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates must be in indices (row and column of the traversability grid map).
    Eigen::VectorXd get_goodness(Eigen::Vector2d ind, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief
    void get_traversability(std::vector<Eigen::Matrix<double,6,1>> &cells, const double distance, const double ground_clearance, const double max_pitch);

    uint getNbCells(){ return size_x_*size_y_;}

    Eigen::Vector2d meter2ind(Eigen::Vector2d meter){
        Eigen::Vector2d idx =  (meter+_halfside)/_resolution;
        return Eigen::Vector2d(std::floor(idx.x()) , std::floor(idx.y()));
    }

    Eigen::Vector2d ind2meter(Eigen::Vector2d ind){
        return (ind*_resolution -_halfside);
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

    geometry_msgs::msg::TransformStamped computeGeometryMsgsTransform(Eigen::Matrix4d& mat) {
        geometry_msgs::msg::TransformStamped transform_msg;

        // Populate the translation component of the transform
        transform_msg.transform.translation.x = mat(0, 3);
        transform_msg.transform.translation.y = mat(1, 3);
        transform_msg.transform.translation.z = mat(2, 3);

        // Extract the rotation component from the transformation matrix
        Eigen::Matrix3d rotation_matrix = mat.block<3, 3>(0, 0);
        
        // Convert the rotation matrix to a quaternion
        Eigen::Quaterniond quaternion(rotation_matrix);
        
        // Populate the rotation component of the transform as a quaternion
        transform_msg.transform.rotation.x = quaternion.x();
        transform_msg.transform.rotation.y = quaternion.y();
        transform_msg.transform.rotation.z = quaternion.z();
        transform_msg.transform.rotation.w = quaternion.w();

        return transform_msg;
    }

private :
    double _resolution;                           /// cell size
    Eigen::Vector2d _center;                      /// Center
    Eigen::Vector2d _halfside;                    /// Half-size of the cube

    NodeMetaData _meta_data;                      /// Meta-data container

    int size_x_;
    int size_y_;

    std::vector<std::vector<NodeMetaData>> _grid;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>> planeLMSPub_;
    geometry_msgs::msg::PoseArray pose_array_msg;
};


#endif
