#include <frontier_exploration/FrontierCostsManager.hpp>

namespace frontier_exploration
{

    FrontierCostsManager::FrontierCostsManager(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    {
        // Creating a client node for internal use

        alpha_ = parameterInstance.getValue<double>("frontierCostsManager/alpha");
        beta_ = parameterInstance.getValue<double>("frontierCostsManager/beta");
        planner_allow_unknown_ = parameterInstance.getValue<bool>("frontierCostsManager/planner_allow_unknown");
        N_best_for_u2_ = parameterInstance.getValue<int>("frontierCostsManager/N_best_for_u2");
        add_heading_cost_ = parameterInstance.getValue<bool>("frontierCostsManager/add_heading_cost");
        max_vx_ = parameterInstance.getValue<double>("frontierCostsManager/vx_max");
        max_wx_ = parameterInstance.getValue<double>("frontierCostsManager/wz_max");
        LOG_DEBUG("Cost manager recieved parameters");
        
        costmap_ = explore_costmap_ros->getCostmap();
        LOG_DEBUG("Got costmap pointer");
        costCalculator_ = std::make_shared<FrontierCostCalculator>(explore_costmap_ros);
        LOG_DEBUG("Making cost calculator instance");
    }

    std::vector<FrontierPtr> findDuplicates(const std::vector<FrontierPtr> &vec)
    {
        std::vector<FrontierPtr> duplicates;

        // Iterate through the vector
        for (size_t i = 0; i < vec.size(); ++i)
        {
            // Compare the current element with all subsequent elements
            for (size_t j = i + 1; j < vec.size(); ++j)
            {
                if (vec[i] == vec[j])
                {
                    // If a duplicate is found, add it to the duplicates vector
                    duplicates.push_back(vec[i]);
                    break; // Break the inner loop to avoid adding the same duplicate multiple times
                }
            }
        }

        return duplicates;
    }

    bool FrontierCostsManager::assignCosts(std::vector<FrontierPtr> &frontier_list, std::vector<double> polygon_xy_min_max,
                                           geometry_msgs::msg::Pose start_pose_w, std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data,
                                           std::vector<std::vector<std::string>> &costTypes)
    {
        costCalculator_->reset();
        costCalculator_->setMaxArrivalInformation();
        LOG_DEBUG("FrontierCostsManager::assignCosts");
        // sanity checks
        if (frontier_list.size() == 0)
        {
            LOG_ERROR("No frontiers found from frontier search.");
            return false;
        }

        if (polygon_xy_min_max.size() <= 0)
        {
            LOG_ERROR("FrontierPtr cannot be selected, no polygon.");
            return false;
        }

        // Iterate through each frontier
        LOG_DEBUG("FrontierPtr list size is (loop): " + std::to_string(frontier_list.size()));
        if (findDuplicates(frontier_list).size() > 0)
        {
            throw std::runtime_error("Duplicate frontiers found.");
        }
        LOG_DEBUG("Blacklist size is: " << frontier_blacklist_.size());
        for (int frontier_idx = 0; frontier_idx < frontier_list.size(); frontier_idx++)
        {
            auto &frontier = frontier_list[frontier_idx];
            if (frontier_blacklist_.count(frontier) > 0)
            {
                frontier->setArrivalInformation(0.0);
                frontier->setGoalOrientation(0.0);
                frontier->setFisherInformation(0.0);
                frontier->setPathLength(std::numeric_limits<double>::max());
                frontier->setPathLengthInM(std::numeric_limits<double>::max());
                frontier->setWeightedCost(std::numeric_limits<double>::max());
                continue;
            }

            /**
             * IMPORTANT! Always compute the arrival information first before the planning.
             * If the frontier is not achievable, the plan is not computed.
             */
            if (vectorContains(costTypes[frontier_idx], std::string("ArrivalInformation")))
            {
                // Calculate frontier information and add to metadata vector
                // camera_fov / 2 is added because until here the maxIndex is only the starting index.
                costCalculator_->setArrivalInformationForFrontier(frontier, polygon_xy_min_max);
            }
            if (vectorContains(costTypes[frontier_idx], std::string("A*PlannerDistance")))
            {
                costCalculator_->setPlanForFrontier(start_pose_w, frontier, map_data, false, planner_allow_unknown_);
            }
            else if (vectorContains(costTypes[frontier_idx], std::string("RoadmapPlannerDistance")))
            {
                costCalculator_->setPlanForFrontierRoadmap(start_pose_w, frontier, map_data, false, planner_allow_unknown_);
            }
            else if (vectorContains(costTypes[frontier_idx], std::string("EuclideanDistance")))
            {
                costCalculator_->setPlanForFrontierEuclidean(start_pose_w, frontier, map_data, false, planner_allow_unknown_);
            }
            if (vectorContains(costTypes[frontier_idx], std::string("RandomCosts")))
            {
                costCalculator_->setRandomMetaData(frontier);
            }
            if (vectorContains(costTypes[frontier_idx], std::string("ClosestFrontier")))
            {
                costCalculator_->setClosestFrontierMetaData(start_pose_w, frontier, map_data, false, planner_allow_unknown_);
            }
            costCalculator_->recomputeNormalizationFactors(frontier);
        } // frontier end

        LOG_DEBUG("utility U1 max info:" << costCalculator_->getMaxArrivalInformation());
        LOG_DEBUG("utility U1 max distance:" << costCalculator_->getMaxPlanDistance());
        LOG_DEBUG("utility U1 min info:" << costCalculator_->getMinArrivalInformation());
        LOG_DEBUG("utility U1 min distance:" << costCalculator_->getMinPlanDistance());
        // U1 Utility
        for (auto &frontier_with_properties : frontier_list)
        {
            LOG_DEBUG("================");
            if (!frontier_with_properties->isAchievable())
            {
                frontier_with_properties->setWeightedCost(std::numeric_limits<double>::max());
                frontier_with_properties->setCost("arrival_gain_utility", -69.8);
                frontier_with_properties->setCost("distance_utility", -1.8);
                continue;
            }
            // the distance term is inverted because we need to choose the closest frontier with tradeoff.
            // the entire utility value is inverted * beta later on to make it a cost for minimisation problem.
            double arrival_info_utility;
            if (static_cast<double>(costCalculator_->getMaxArrivalInformation() - costCalculator_->getMinArrivalInformation()) == 0.0)
                arrival_info_utility = 0.0;
            else
                // arrival_info_utility = static_cast<double>(frontier_with_properties->getArrivalInformation() - costCalculator_->getMinArrivalInformation()) /
                //                        static_cast<double>(costCalculator_->getMaxArrivalInformation() - costCalculator_->getMinArrivalInformation());

                arrival_info_utility = static_cast<double>(frontier_with_properties->getArrivalInformation()) /
                                       static_cast<double>(costCalculator_->getMaxArrivalInformation());

            if(arrival_info_utility > 1.0)
                throw std::runtime_error("ARRIVAL UTILITY ERROR! MORE THAN 1. It is: " + std::to_string(frontier_with_properties->getArrivalInformation()));

            double frontier_plan_utility;
            if (static_cast<double>((costCalculator_->getMaxPlanDistance()  / max_vx_ + M_PI / max_wx_) - (costCalculator_->getMinPlanDistance() / max_vx_ + 0.0 / max_wx_) == 0.0))
                frontier_plan_utility = 1.0; // keep it 1 cuz it's -1.0'd later
            else
                // frontier_plan_utility = static_cast<double>(frontier_with_properties->getPathLength() - costCalculator_->getMinPlanDistance()) /
                //                         static_cast<double>(costCalculator_->getMaxPlanDistance() - costCalculator_->getMinPlanDistance());

                frontier_plan_utility = static_cast<double>(frontier_with_properties->getPathLength() / max_vx_ + frontier_with_properties->getPathHeading() / max_wx_) /
                                        static_cast<double>(costCalculator_->getMaxPlanDistance() / max_vx_ + M_PI / max_wx_);
            frontier_plan_utility = 1.0 - frontier_plan_utility;

            LOG_DEBUG("Path length : " << frontier_with_properties->getPathLength());
            LOG_DEBUG("Min Path length : " << costCalculator_->getMinPlanDistance());
            LOG_DEBUG("Max Path length : " << costCalculator_->getMaxPlanDistance());
            LOG_DEBUG("Path Utility:" << frontier_plan_utility);

            LOG_DEBUG("****************************");
            LOG_DEBUG("Current info : " << frontier_with_properties->getArrivalInformation());
            LOG_DEBUG("Min Info: " << costCalculator_->getMinArrivalInformation());
            LOG_DEBUG("Max Info : " << costCalculator_->getMaxArrivalInformation());
            LOG_DEBUG("Info Utility:" << arrival_info_utility);

            if (arrival_info_utility > 1.0 || arrival_info_utility < 0.0 || frontier_plan_utility > 1.0 || frontier_plan_utility < 0.0)
                throw std::runtime_error("Cost out of bounds");

            double utility = (alpha_ * arrival_info_utility) +
                             ((1.0 - alpha_) * frontier_plan_utility);
            if (utility == 0.0)
            {
                // set to small value to prevent infinite costs.
                utility = 1e-16;
            }

            // LOG_DEBUG("Weighted utility: " << utility);

            // RCLCPP_INFO_STREAM(logger_, "Utility U1 information:" << frontier_with_properties->getArrivalInformation());
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 distance:" << costCalculator_->getMinPlanDistance());
            // RCLCPP_INFO_STREAM(logger_, "arrival utility" << alpha_ * (static_cast<double>(frontier_with_properties->getArrivalInformation()) / static_cast<double>(costCalculator_->getMaxArrivalInformation())));
            // RCLCPP_INFO_STREAM(logger_, "distance utility" << (1.0 - alpha_) * (static_cast<double>(costCalculator_->getMinPlanDistance()) / frontier_with_properties->getPathLength()));
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 before fix:" << utility);
            // It is added with Beta multiplied. If the frontier lies in the N best then this value will be replaced with information on path.
            // If it does not lie in the N best then the information on path is treated as 0 and hence it is appropriate to multiply with beta.

            // if(frontier_costs.count(frontier_with_properties.frontier_) == 1)
            //     throw std::runtime_error("Something is wrong. Duplicate frontiers?");

            // TODO: Set along with Fisher information (alpha).
            frontier_with_properties->setWeightedCost(1 / (beta_ * utility));
            frontier_with_properties->setCost("arrival_gain_utility", arrival_info_utility);
            frontier_with_properties->setCost("distance_utility", frontier_plan_utility);
            LOG_DEBUG("Weighted cost: " << 1 / (beta_ * utility));
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 cost after fix:" << 1 / (beta_ * utility));
            // RCLCPP_INFO_STREAM(logger_, "MAX info:" << costCalculator_->getMaxArrivalInformation());
            // RCLCPP_INFO_STREAM(logger_, "Currnt path length" << frontier_with_properties->getPathLength());
        }
        // FrontierU1ComparatorCost frontier_u1_comp;
        // // sort the frontier list based on the utility value
        // std::sort(frontier_with_u1_utility.begin(), frontier_with_u1_utility.end(), frontier_u1_comp);

        LOG_DEBUG("The selected frontier was not updated after U2 computation.");
        LOG_DEBUG("Returning for input list size: " << frontier_list.size());
        return true;
    }

    void FrontierCostsManager::setFrontierBlacklist(std::vector<FrontierPtr> &blacklist)
    {
        std::lock_guard<std::mutex> lock(blacklist_mutex_);
        for (auto frontier : blacklist)
        {
            frontier_blacklist_[frontier] = true;
        }
    }
}
