//
// Created by d.vivet on 19/04/23.
//
#include "traversability_gridmap/traversabilityGrid.hpp"
#include <Eigen/Eigenvalues>

void traversabilityGrid::insert_data(Eigen::Vector3d &p3d)
{

    // Are we at maximal subdivision lvl?
    Eigen::Vector2d ind = meter2ind(Eigen::Vector2d(p3d.x(), p3d.y()));

    if (ind.x() >= size_x_ || ind.y() >= size_y_ ||
        ind.x() < 0 || ind.y() < 0)
        return;
    _grid.at(ind.x()).at(ind.y()).insert(p3d);
}

Eigen::VectorXd traversabilityGrid::get_goodness_m(Eigen::Vector2d meters, const double distance, const double ground_clearance, const double max_pitch)
{
    return get_goodness(meter2ind(meters), distance, ground_clearance, max_pitch);
}

Eigen::VectorXd traversabilityGrid::get_goodness(Eigen::Vector2d ind, const double distance, const double ground_clearance, const double max_pitch)
{
    double delta = distance / _resolution;
    // TODO: std::max instead of std::min?
    uint delta_ind = std::min(1.0, std::ceil(delta));

    // Point goodness cannot be calculated (border of the area) or too close to robot
    if (ind.x() < delta_ind || ind.x() > (size_x_ - delta_ind) ||
        ind.y() < delta_ind || ind.y() > (size_y_ - delta_ind))
    {
        Eigen::VectorXd X(5);
        X << -1., 0., 0., 0., 0.;
        return X;
    }

    // // Get mean and covariance of the grid.
    // Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    // mean(0) = _grid.at(ind.x()).at(ind.y()).sx / _grid.at(ind.x()).at(ind.y()).N;
    // mean(1) = _grid.at(ind.x()).at(ind.y()).sy / _grid.at(ind.x()).at(ind.y()).N;
    // mean(2) = _grid.at(ind.x()).at(ind.y()).sz / _grid.at(ind.x()).at(ind.y()).N;

    // Eigen::Matrix3d matCov = Eigen::Matrix3d::Zero();
    // matCov(0, 0) = ((_grid.at(ind.x()).at(ind.y()).sx2) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(0) * mean(0)) - (2 * mean(0) * _grid.at(ind.x()).at(ind.y()).sx)) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(0, 1) = ((_grid.at(ind.x()).at(ind.y()).sxy) - (mean(1) * _grid.at(ind.x()).at(ind.y()).sx) - (mean(0) * _grid.at(ind.x()).at(ind.y()).sy) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(0) * mean(1))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(0, 2) = ((_grid.at(ind.x()).at(ind.y()).sxz) - (mean(2) * _grid.at(ind.x()).at(ind.y()).sx) - (mean(0) * _grid.at(ind.x()).at(ind.y()).sz) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(0) * mean(2))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(1, 0) = ((_grid.at(ind.x()).at(ind.y()).sxy) - (mean(1) * _grid.at(ind.x()).at(ind.y()).sx) - (mean(0) * _grid.at(ind.x()).at(ind.y()).sy) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(0) * mean(1))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(1, 1) = ((_grid.at(ind.x()).at(ind.y()).sy2) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(1) * mean(1)) - (2 * mean(1) * _grid.at(ind.x()).at(ind.y()).sy)) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(1, 2) = ((_grid.at(ind.x()).at(ind.y()).syz) - (mean(2) * _grid.at(ind.x()).at(ind.y()).sy) - (mean(1) * _grid.at(ind.x()).at(ind.y()).sz) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(1) * mean(2))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(2, 0) = ((_grid.at(ind.x()).at(ind.y()).sxz) - (mean(2) * _grid.at(ind.x()).at(ind.y()).sx) - (mean(0) * _grid.at(ind.x()).at(ind.y()).sz) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(0) * mean(2))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(2, 1) = ((_grid.at(ind.x()).at(ind.y()).syz) - (mean(2) * _grid.at(ind.x()).at(ind.y()).sy) - (mean(1) * _grid.at(ind.x()).at(ind.y()).sz) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(1) * mean(2))) / _grid.at(ind.x()).at(ind.y()).N;
    // matCov(2, 2) = ((_grid.at(ind.x()).at(ind.y()).sz2) + ((_grid.at(ind.x()).at(ind.y()).N) * mean(2) * mean(2)) - (2 * mean(2) * _grid.at(ind.x()).at(ind.y()).sz)) / _grid.at(ind.x()).at(ind.y()).N;

    // Init params
    uint border_hazard = 0;
    double zM = -100., zm = 100.;

    uint nb_min = 0.25 * (2 * delta_ind + 1) * (2 * delta_ind + 1);

    // Get all points to be considered for the current cell and get min/max altitude
    std::vector<Eigen::Vector3d> P3Ds; // stores all centers of neighbouring cells.
    std::vector<double> traceCOVs;     // stores all the covariances of the neighbouring cells.
    for (int i = std::max(0, int(ind.x() - delta_ind)); i < std::min(int(ind.x() + delta_ind), size_x_); i++)
    {
        for (int j = std::max(0, int(ind.y() - delta_ind)); j < std::min(int(ind.y() + delta_ind), size_y_); j++)
        {
            if (_grid.at(i).at(j).N < nb_min)
            {
                border_hazard++;
            }
            else
            {
                // center of all the points in the neighbouring cell.
                Eigen::Vector3d center =
                    Eigen::Vector3d(_grid.at(i).at(j).sx, _grid.at(i).at(j).sy, _grid.at(i).at(j).sz) /
                    _grid.at(i).at(j).N;

                // TODO: check formula of calculating cov of each neighbouring cell.
                double traceCov = _grid.at(i).at(j).sx2 + _grid.at(i).at(j).sx / _grid.at(i).at(j).N +
                                  _grid.at(i).at(j).sy2 + _grid.at(i).at(j).sy / _grid.at(i).at(j).N +
                                  _grid.at(i).at(j).sz2 + _grid.at(i).at(j).sz / _grid.at(i).at(j).N;

                P3Ds.push_back(center);
                traceCOVs.push_back(traceCov);
                // TODO: Check logic of min and max.
                zM = std::max(zM, _grid.at(i).at(j).z_max);
                zm = std::min(zm, _grid.at(i).at(j).z_min);
            }
        }
    }

    if (border_hazard > 0)
    {
        Eigen::VectorXd X(5);
        X << -1., 0., 0., 0., 0.;
        return X;
    }

    // Check if delta z is bigger than the robot ground clearance (how to deals with grass?)
    double step_hazard = ((zM - zm) / ground_clearance);
    if (step_hazard > 1)
        step_hazard = 1;

    // Process surface normal

    /*
        // Method n°1 : classical LMS with cells barycenters
        // AX=B
        // Eigen::Vector3f X = A.colPivHouseholderQr().solve(B);
        Eigen::MatrixXd P3Ds_M(3, P3Ds.size());
        for(uint i=0; i < P3Ds.size(); ++i)
            P3Ds_M.block<3,1>(0,i) = P3Ds.at(i);
        Eigen::VectorXd ONE = Eigen::VectorXd::Ones(P3Ds.size()) ;
        Eigen::Vector3d planeLMS = P3Ds_M.transpose().colPivHouseholderQr().solve(ONE);
        planeLMS.normalize();

        double roughness = (Eigen::VectorXd::Ones(P3Ds.size())-P3Ds_M.transpose()*planeLMS).norm()/P3Ds.size();
    */

    // Method n°2 : using covariance ponderation (trace of P3D covariance as weight)
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    for (uint i = 0; i < P3Ds.size(); ++i)
    {
        A(0, 0) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).x();
        A(0, 1) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).y();
        A(0, 2) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).z();
        A(1, 0) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).x();
        A(1, 1) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).y();
        A(1, 2) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).z();
        A(2, 0) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).x();
        A(2, 1) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).y();
        A(2, 2) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).z();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A); // eigenvalue
    Eigen::Vector3d planeLMS = es.eigenvectors().col(0);  // eigenvector having the direction of surface normal

    if (planeLMS.z() < 0)
        planeLMS = -planeLMS;

    // std::cout << planeLMS.transpose() << std::endl;

    double roughness = 0.0;
    for (uint i = 0; i < P3Ds.size(); ++i)
    {
        // dot product of points coordinates and surface normal ** 2
        roughness += double(P3Ds.at(i).transpose() * planeLMS) * double(P3Ds.at(i).transpose() * planeLMS);
    }
    // This metric represents the degree of alignment between the points and the calculated surface normal.
    roughness = std::sqrt(roughness) / P3Ds.size();

    // Check if ground roughness is bigger than the robot ground clearance (how to deals with grass?)
    // std::cout << "roughness : " << roughness << std::endl;
    double roughness_hazard = 3. * roughness / ground_clearance;
    if (roughness_hazard > 1)
        roughness_hazard = 1;

    // Check if ground slope is bigger than the robot admissive max slope
    double pitch = std::abs(std::acos(planeLMS.dot(Eigen::Vector3d(0., 0., 1.))));
    // std::cout << "pitch : " << pitch << std::endl;
    double pitch_hazard = pitch / max_pitch;
    // if(pitch_hazard > 1) pitch_hazard = 1;

    // std::cout << "pitch_hazard : " << pitch_hazard << " roughness_hazard : " << roughness_hazard << " step_hazard : " << step_hazard << std::endl;

    Eigen::VectorXd haz(30);
    haz(0) = std::max(std::max(step_hazard, roughness_hazard), 0.0 * pitch_hazard);
    haz(1) = step_hazard;
    haz(2) = roughness_hazard;
    haz(3) = pitch_hazard;
    haz(4) = border_hazard;
    haz(5) = _grid.at(ind.x()).at(ind.y()).sz / _grid.at(ind.x()).at(ind.y()).N;

    // for (uint i = 0; i <= 2; i++)
    // {
    //     haz(i + 6) = _grid.at(ind.x()).at(ind.y()).mean(i);
    //     haz(i + 9) = _grid.at(ind.x()).at(ind.y()).matCov(0, i);
    //     haz(i + 12) = _grid.at(ind.x()).at(ind.y()).matCov(1, i);
    //     haz(i + 15) = _grid.at(ind.x()).at(ind.y()).matCov(2, i);
    // }

    return haz;
}

void traversabilityGrid::get_traversability(std::vector<Eigen::Matrix<double, 6, 1>> &cells, const double distance, const double ground_clearance, const double max_pitch)
{
    for (int i = 0; i < size_x_; ++i)
    {
        for (int j = 0; j < size_y_; ++j)
        {

            Eigen::VectorXd haz = get_goodness(Eigen::Vector2d(i, j), distance, ground_clearance, max_pitch);
            Eigen::Vector2d center = ind2meter(Eigen::Vector2d(i, j));

            Eigen::Matrix<double, 6, 1> node;
            node << center.x(), center.y(), _halfside.x(), _halfside.y(), haz(0), 0.;
            cells.push_back(node);
        }
    }
}

//
// void traversabilityGrid::getStructure(std::vector<Eigen::Matrix<double,6,1>> &nodes){
//    if(_level <= _max_level){
//        if(this->has_childs()){
//            for(uint i=0; i < 4; ++i) {
//                if (!_childs.at(i))
//                    continue;
//                else {
//                    this->_childs.at(i)->getStructure(nodes);
//                }
//            }
//        }
//
//        if(this->has_childs())
//            return;
//
//        Eigen::Matrix<double,6,1> node;
//        node << _center.x(), _center.y(), _halfside.x(), _halfside.y(), _level, 0.; //, this->get_goodness(Eigen::Vector3d(_center.x(), _center.y(), 0.), 1., 0.25, 25.*M_PI/180.);
//        nodes.push_back(node);
//
//
//    }
//}
