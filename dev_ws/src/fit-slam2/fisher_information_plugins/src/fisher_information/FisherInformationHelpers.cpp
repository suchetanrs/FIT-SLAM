#include "fisher_information_plugins/fisher_information/FisherInformationHelpers.hpp"


namespace roadmap_explorer
{
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

    Eigen::Matrix<float, 3, 6> computeJacobianForPointGlobal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est)
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
        // // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
        return jacobian;
    }

    Eigen::Matrix<float, 3, 6> computeJacobianForPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig, Eigen::Affine3f &T_w_c_est)
    {
        // convert the world coordinates to camera coordinates of the pose used for estimation.
        Eigen::Vector3f p3d_c_eig_est = T_w_c_est.inverse() * p3d_w_eig;

        // df_dp
        const float n = p3d_c_eig_est.norm();
        Eigen::Matrix3f df_dpc = (1 / n) * Eigen::Matrix3f::Identity() -
                                 (1 / (n * n * n)) * p3d_c_eig_est * p3d_c_eig_est.transpose();

        // dp_dTwc
        Eigen::Matrix<float, 3, 6> rightMat;
        rightMat.block<3, 3>(0, 0) = (-1.0) * Eigen::Matrix3f::Identity();
        rightMat.block<3, 3>(0, 3) = getSkewMatrix(p3d_c_eig_est);
        Eigen::Matrix<float, 3, 6> dpc_dtwc = rightMat;

        Eigen::Matrix<float, 3, 6> jacobian = df_dpc * dpc_dtwc;
        // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
        return jacobian;
    }

    Eigen::Matrix<float, 3, 6> computeJacobianForPointLocal(Eigen::Vector3f &p3d_c_eig)
    {
        // df_dp
        const float n = p3d_c_eig.norm();
        Eigen::Matrix3f df_dpc = (1 / n) * Eigen::Matrix3f::Identity() -
                                 (1 / (n * n * n)) * p3d_c_eig * p3d_c_eig.transpose();

        // std::cout << "df_dpc" << df_dpc << std::endl;

        // dp_dTwc
        Eigen::Matrix<float, 3, 6> rightMat;
        rightMat.block<3, 3>(0, 0) = (-1.0) * Eigen::Matrix3f::Identity();
        rightMat.block<3, 3>(0, 3) = getSkewMatrix(p3d_c_eig);
        Eigen::Matrix<float, 3, 6> dpc_dtwc = rightMat;

        // std::cout << "dpc_dtwc" << dpc_dtwc << std::endl;

        Eigen::Matrix<float, 3, 6> jacobian = df_dpc * dpc_dtwc;
        // std::cout << std::endl << "Jacobian is: " << jacobian << std::endl;
        return jacobian;
    }

    Eigen::Matrix<float, 6, 6> computeFIM(Eigen::Matrix<float, 3, 6> &jacobian, Eigen::Matrix3f &Q)
    {
        return jacobian.transpose() * Q.inverse() * jacobian;
    }

    float computeInformationOfPointGlobal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                          Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f Q)
    {
        auto jac = computeJacobianForPointGlobal(p3d_c_eig, p3d_w_eig, T_w_c_est);
        auto fim = computeFIM(jac, Q);
        return fim.trace();
    }

    float computeInformationOfPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Vector3f &p3d_w_eig,
                                         Eigen::Affine3f &T_w_c_est, Eigen::Matrix3f Q)
    {
        auto jac = computeJacobianForPointLocal(p3d_c_eig, p3d_w_eig, T_w_c_est);
        auto fim = computeFIM(jac, Q);
        return fim.trace();
    }

    float computeInformationOfPointLocal(Eigen::Vector3f &p3d_c_eig, Eigen::Matrix3f Q)
    {
        auto jac = computeJacobianForPointLocal(p3d_c_eig);
        auto fim = computeFIM(jac, Q);
        // std::cout << std::endl;
        // std::cout << "FIM is: " << fim << std::endl;
        // std::cout << std::endl;
        // std::cout << "FIM inverse is: " << fim << std::endl;
        return fim.trace();
    }

    float computeInformationFrontierPair(std::vector<geometry_msgs::msg::Point> &lndmrk_w,
                                         geometry_msgs::msg::Pose &kf_pose_w, geometry_msgs::msg::Pose &est_pose_w,
                                         std::vector<Point2D> &FOVFrontierPair)
    {
        float pair_information = 0;
        Eigen::Affine3f T_w_c_est = getTransformFromPose(est_pose_w);
        Eigen::Affine3f T_w_c = getTransformFromPose(kf_pose_w);
        for (auto p3d_w : lndmrk_w)
        {
            Eigen::Vector3f p3d_w_eig(p3d_w.x, p3d_w.y, p3d_w.z);
            if (isInside(p3d_w.x, p3d_w.y, FOVFrontierPair[0], FOVFrontierPair[1], FOVFrontierPair[2]))
            {
                auto p3d_c_eig = T_w_c.inverse() * p3d_w_eig;
                auto Q = Eigen::Matrix3f::Identity();
                pair_information += computeInformationOfPointLocal(p3d_c_eig, p3d_w_eig, T_w_c_est, Q);
            }
        }
        return pair_information;
    }
}