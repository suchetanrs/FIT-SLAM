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

// -------------------------- FISHER INFORMATION COMPUTATION RELATED --------------------------------
    Eigen::Matrix3f getSkewMatrix(const Eigen::Vector3f &v)
    {
        Eigen::Matrix3f skewMat;
        skewMat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return skewMat;
    }

    Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose &pose)
    {
        // Extract translation and rotation from the pose message
        Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        // Construct the transformation matrix
        Eigen::Affine3f transform = Eigen::Translation3f(translation) * Eigen::Quaternionf(rotation);

        return transform;
    }

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

    Eigen::Matrix<float, 6, 6> computeFIM(Eigen::Matrix<float, 3, 6> &jacobian, Eigen::Matrix3f &Q)
    {
        return jacobian.transpose() * Q.inverse() * jacobian;
    }

    float computeInformationOfPoint(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                     Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f Q)
    {
        auto jac = computeJacobianForPoint(p3d_c_eig, p3d_w_eig, T_w_c_est);
        auto fim = computeFIM(jac, Q);
        return fim.trace();
    }

    float computeInformationFrontierPair(std::vector<geometry_msgs::msg::Point>& lndmrk_w, 
                                         geometry_msgs::msg::Pose& kf_pose_w, geometry_msgs::msg::Pose& est_pose_w, 
                                         std::vector<Point2D>& FOVFrontierPair)
    {
        float pair_information = 0;
        Eigen::Affine3f T_w_c_est = getTransformFromPose(est_pose_w);
        Eigen::Affine3f T_w_c = getTransformFromPose(kf_pose_w);
        for (auto p3d_w : lndmrk_w)
        {
            Eigen::Vector3f p3d_w_eig(p3d_w.x, p3d_w.y, p3d_w.z);
            if(isInside(p3d_w.x, p3d_w.y, FOVFrontierPair[0], FOVFrontierPair[1], FOVFrontierPair[2]))
            {
                auto p3d_c_eig = T_w_c.inverse() * p3d_w_eig;
                auto Q = Eigen::Matrix3f::Identity();
                pair_information += computeInformationOfPoint(p3d_c_eig, p3d_w_eig, T_w_c_est, Q);
            }
        }
        return pair_information;
    }
}