#include <frontier_exploration/FrontierCostsManager.hpp>

namespace frontier_exploration
{

    FrontierCostsManager::FrontierCostsManager(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
    {
        logger_ = rclcpp::get_logger(static_cast<std::string>(node->get_namespace()) + ".frontier_costs_manager");

        // Creating a client node for internal use
        frontier_costs_manager_node_ = rclcpp::Node::make_shared("frontier_costs_manager_node");

        frontier_costs_manager_node_->declare_parameter("alpha", 0.35);
        frontier_costs_manager_node_->get_parameter("alpha", alpha_);

        frontier_costs_manager_node_->declare_parameter("beta", 0.50);
        frontier_costs_manager_node_->get_parameter("beta", beta_);

        frontier_costs_manager_node_->declare_parameter("frontier_detect_radius", 0.80);
        frontier_costs_manager_node_->get_parameter("frontier_detect_radius", frontierDetectRadius_);

        frontier_costs_manager_node_->declare_parameter("planner_allow_unknown", false);
        frontier_costs_manager_node_->get_parameter("planner_allow_unknown", planner_allow_unknown_);

        frontier_costs_manager_node_->declare_parameter("N_best_for_u2", 6);
        frontier_costs_manager_node_->get_parameter("N_best_for_u2", N_best_for_u2_);

        frontier_costs_manager_node_->declare_parameter("use_planning", true);
        frontier_costs_manager_node_->get_parameter("use_planning", use_planning_);

        costmap_ = costmap;
        costCalculator_ = std::make_shared<FrontierCostCalculator>(node, costmap_);
    }

    std::vector<Frontier> findDuplicates(const std::vector<Frontier> &vec)
    {
        std::vector<Frontier> duplicates;

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

    bool FrontierCostsManager::assignCosts(std::vector<Frontier> &frontier_list, std::vector<double> polygon_xy_min_max,
                                               geometry_msgs::msg::Point start_point_w, std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data,
                                               std::vector<std::vector<std::string>>& costTypes)
    {
        costCalculator_->updateRoadmapData(frontier_list);
        costCalculator_->reset();
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("FrontierCostsManager::assignCostsOurs", logger_.get_name()));
        // sanity checks
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found from frontier search.");
            return false;
        }

        if (polygon_xy_min_max.size() <= 0)
        {
            RCLCPP_ERROR(logger_, "Frontier cannot be selected, no polygon.");
            return false;
        }

        // Iterate through each frontier
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Frontier list size is (loop): " + std::to_string(frontier_list.size()), logger_.get_name()));
        if (findDuplicates(frontier_list).size() > 0)
        {
            throw std::runtime_error("Duplicate frontiers found.");
        }
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("Blacklist size is: " + std::to_string(frontier_blacklist_.size()), logger_.get_name()));
        for (int frontier_idx = 0; frontier_idx < frontier_list.size(); frontier_idx++)
        {
            auto& frontier = frontier_list[frontier_idx];
            if (frontier_blacklist_.count(frontier) > 0)
            {
                frontier.setArrivalInformation(0.0);
                frontier.setGoalOrientation(0.0);
                frontier.setFisherInformation(0.0);
                frontier.setPathLength(std::numeric_limits<double>::max());
                frontier.setWeightedCost(std::numeric_limits<double>::max());
                continue;
            }
            
            if (vectorContains(costTypes[frontier_idx], std::string("A*PlannerDistance")))
            {
                costCalculator_->setPlanForFrontier(start_point_w, frontier, map_data, false, planner_allow_unknown_);
            }
            else if(vectorContains(costTypes[frontier_idx], std::string("RoadmapPlannerDistance")))
            {
                costCalculator_->setPlanForFrontierRoadmap(start_point_w, frontier, map_data, false, planner_allow_unknown_);
            }
            else if(vectorContains(costTypes[frontier_idx], std::string("EuclideanDistance")))
            {
                costCalculator_->setPlanForFrontierEuclidean(start_point_w, frontier, map_data, false, planner_allow_unknown_);
            }
            if(vectorContains(costTypes[frontier_idx], std::string("ArrivalInformation")))
            {
                // Calculate frontier information and add to metadata vector
                // camera_fov / 2 is added because until here the maxIndex is only the starting index.
                costCalculator_->setArrivalInformationForFrontier(frontier, polygon_xy_min_max);
            }
            if(vectorContains(costTypes[frontier_idx], std::string("RandomCosts")))
            {
                costCalculator_->setRandomMetaData(frontier);
            }
            if(vectorContains(costTypes[frontier_idx], std::string("ClosestFrontier")))
            {
                costCalculator_->setClosestFrontierMetaData(start_point_w, frontier, map_data, false, planner_allow_unknown_);
            }
            costCalculator_->recomputeNormalizationFactors(frontier);
        } // frontier end
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // U1 Utility
        for (auto &frontier_with_properties : frontier_list)
        {
            // the distance term is inverted because we need to choose the closest frontier with tradeoff.
            auto utility = (alpha_ * (static_cast<double>(frontier_with_properties.getArrivalInformation()) / static_cast<double>(costCalculator_->getMaxArrivalInformation()))) +
                           ((1.0 - alpha_) * (static_cast<double>(costCalculator_->getMinPlanDistance()) / frontier_with_properties.getPathLength()));

            // RCLCPP_INFO_STREAM(logger_, "Utility U1 information:" << frontier_with_properties.getArrivalInformation());
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 distance:" << costCalculator_->getMinPlanDistance());
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 max info:" << costCalculator_->getMaxArrivalInformation());
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 max distance:" << frontier_with_properties.getPathLength());
            // RCLCPP_INFO_STREAM(logger_, "arrival utility" << alpha_ * (static_cast<double>(frontier_with_properties.getArrivalInformation()) / static_cast<double>(costCalculator_->getMaxArrivalInformation())));
            // RCLCPP_INFO_STREAM(logger_, "distance utility" << (1.0 - alpha_) * (static_cast<double>(costCalculator_->getMinPlanDistance()) / frontier_with_properties.getPathLength()));
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 before fix:" << utility);
            // It is added with Beta multiplied. If the frontier lies in the N best then this value will be replaced with information on path.
            // If it does not lie in the N best then the information on path is treated as 0 and hence it is appropriate to multiply with beta.

            // if(frontier_costs.count(frontier_with_properties.frontier_) == 1)
            //     throw std::runtime_error("Something is wrong. Duplicate frontiers?");

            frontier_with_properties.setWeightedCost((beta_ * utility) == 0 ? std::numeric_limits<double>::max() : 1 / (beta_ * utility));
            // RCLCPP_INFO_STREAM(logger_, "Utility U1 cost after fix:" << 1 / (beta_ * utility));
            // RCLCPP_INFO_STREAM(logger_, "MAX info:" << costCalculator_->getMaxArrivalInformation());
            // RCLCPP_INFO_STREAM(logger_, "Currnt path length" << frontier_with_properties.getPathLength());
        }
        RCLCPP_DEBUG_STREAM(logger_, "Alpha_: " << alpha_);
        RCLCPP_DEBUG_STREAM(logger_, "Beta_: " << beta_);
        // FrontierU1ComparatorCost frontier_u1_comp;
        // // sort the frontier list based on the utility value
        // std::sort(frontier_with_u1_utility.begin(), frontier_with_u1_utility.end(), frontier_u1_comp);

        // // U2 UTILITY processed for N best.
        // Frontier::SharedPtr frontier_selected_post_u2;
        // double theta_s_star_post_u2;
        // // std::map of frontier information mapped to the new utility
        // std::map<frontier_exploration::FrontierWithMetaData, double> frontier_with_path_information;

        // if (static_cast<int>(frontier_with_u1_utility.size()) > 0)
        // {
        //     RCLCPP_DEBUG_STREAM(logger_, "The value of N_best_for_u2 is: " << N_best_for_u2_);
        //     double max_u2_utility = 0;
        //     double maximum_path_information = 0;
        //     if (N_best_for_u2_ == -1)
        //     { // if -1 then the path information for all the frontiers is computed.
        //         N_best_for_u2_ = static_cast<int>(frontier_with_u1_utility.size());
        //     }
        //     RCLCPP_DEBUG_STREAM(logger_, "The value of loop min is: " << static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_);
        //     for (int m = static_cast<int>(frontier_with_u1_utility.size()) - 1; m >= static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_; m--)
        //     {
        //         if (m == 0)
        //         {
        //             break;
        //         }
        //         auto plan_result = getPlanForFrontier(start_point_w, frontier_with_u1_utility[m].first.frontier_, map_data, true);
        //         if (plan_result.first.information_total > maximum_path_information)
        //         {
        //             maximum_path_information = plan_result.first.information_total;
        //         }
        //         frontier_with_path_information[frontier_with_u1_utility[m].first] = plan_result.first.information_total;
        //     }
        //     for (int m = static_cast<int>(frontier_with_u1_utility.size()) - 1; m >= static_cast<int>(frontier_with_u1_utility.size()) - N_best_for_u2_; m--)
        //     {
        //         if (m == 0)
        //         {
        //             break;
        //         }
        //         double current_utility = ((beta_ * frontier_with_u1_utility[m].second) + ((1 - beta_) * (frontier_with_path_information[frontier_with_u1_utility[m].first] / maximum_path_information)));
        //         frontier_costs[frontier_with_u1_utility[m].first] = current_utility;
        //         if (current_utility > max_u2_utility)
        //         {
        //             RCLCPP_DEBUG_STREAM(logger_, "Current U2 utility: " << current_utility);
        //             RCLCPP_DEBUG_STREAM(logger_, "U1 utility for current one: " << frontier_with_u1_utility[m].second);
        //             max_u2_utility = current_utility;
        //             theta_s_star_post_u2 = frontier_with_u1_utility[m].first.theta_s_star_;
        //             frontier_selected_post_u2 = std::make_shared<Frontier>(frontier_with_u1_utility[m].first.frontier_);
        //         }
        //     }
        // }
        // else
        // {
        //     RCLCPP_WARN_STREAM(logger_, COLOR_STR("The number of frontiers after U1 compute is zero.", logger_.get_name()));
        //     SelectionResult selection_result;
        //     selection_result.frontier = selected_frontier;
        //     selection_result.orientation = selected_orientation;
        //     selection_result.success = false;
        //     selection_result.frontier_costs = frontier_costs;
        //     return selection_result;
        // }

        RCLCPP_WARN_STREAM(logger_, COLOR_STR("The selected frontier was not updated after U2 computation.", logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Returning for input list size: " + std::to_string(frontier_list.size()), logger_.get_name()));
        return true;
    }

    void FrontierCostsManager::setFrontierBlacklist(std::vector<Frontier> &blacklist)
    {
        std::lock_guard<std::mutex> lock(blacklist_mutex_);
        for (auto frontier : blacklist)
        {
            frontier_blacklist_[frontier] = true;
        }
        // for (auto frontier : frontier_blacklist_)
        // {
        //     RCLCPP_ERROR_STREAM(logger_, COLOR_STR("Blacklist: " + std::to_string(frontier.first.goal_point.x), logger_.get_name()));
        // }
    }
}
