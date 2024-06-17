#include <frontier_exploration/frontier_selection.hpp>

namespace frontier_exploration
{

    FrontierSelectionNode::FrontierSelectionNode(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D *costmap)
    {
        logger_ = rclcpp::get_logger(static_cast<std::string>(node->get_namespace()) + ".frontier_selection");

        path_pose_array_ = node->create_publisher<geometry_msgs::msg::PoseArray>("frontier_plan_poses", 10);
        // Creating a client node for internal use
        frontier_selection_node_ = rclcpp::Node::make_shared("frontier_selection_node");

        frontier_selection_node_->declare_parameter("alpha", 0.35);
        frontier_selection_node_->get_parameter("alpha", alpha_);

        frontier_selection_node_->declare_parameter("beta", 0.50);
        frontier_selection_node_->get_parameter("beta", beta_);

        frontier_selection_node_->declare_parameter("frontier_detect_radius", 0.80);
        frontier_selection_node_->get_parameter("frontier_detect_radius", frontierDetectRadius_);

        frontier_selection_node_->declare_parameter("planner_allow_unknown", false);
        frontier_selection_node_->get_parameter("planner_allow_unknown", planner_allow_unknown_);

        frontier_selection_node_->declare_parameter("N_best_for_u2", 6);
        frontier_selection_node_->get_parameter("N_best_for_u2", N_best_for_u2_);

        frontier_selection_node_->declare_parameter("use_planning", true);
        frontier_selection_node_->get_parameter("use_planning", use_planning_);

        frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);
        costmap_ = costmap;
        rosVisualizer_ = std::make_shared<RosVisualizer>(node, costmap_);
        costCalculator_ = std::make_shared<FrontierCostCalculator>(node, costmap_);
    }

    std::vector<frontier_msgs::msg::Frontier> findDuplicates(const std::vector<frontier_msgs::msg::Frontier> &vec)
    {
        std::vector<frontier_msgs::msg::Frontier> duplicates;

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

    void FrontierSelectionNode::frontierPlanViz(nav_msgs::msg::Path path)
    {
        frontier_plan_pub_->publish(path);
    }

    SelectionResult FrontierSelectionNode::selectFrontierOurs(std::vector<frontier_msgs::msg::Frontier> &frontier_list, std::vector<double> polygon_xy_min_max,
                                                              geometry_msgs::msg::Point start_point_w, std::shared_ptr<slam_msgs::srv::GetMap_Response> map_data)
    {
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("FrontierSelectionNode::selectFrontierOurs", logger_.get_name()));
        // Vector to store frontiers with meta data and the u1 utility values
        std::vector<FrontierWithMetaData> frontier_meta_data_vector;
        std::map<frontier_msgs::msg::Frontier, FrontierWithMetaData, FrontierLessThan> frontier_costs;       
        SelectionResult selection_result;
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found from frontier search.");
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        if (polygon_xy_min_max.size() <= 0)
        {
            RCLCPP_ERROR(logger_, "Frontier cannot be selected, no polygon.");
            selection_result.success = false;
            selection_result.frontier_costs = frontier_costs;
            return selection_result;
        }

        // -------Cost comparision variables------
        double min_traversable_distance = std::numeric_limits<double>::max();
        int max_arrival_info_per_frontier = 0.0;
        // Iterate through each frontier
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Frontier list size is (loop): " + std::to_string(frontier_list.size()), logger_.get_name()));
        if(findDuplicates(frontier_list).size() > 0)
        {
            throw std::runtime_error("Duplicate frontiers found.");
        }
        RCLCPP_INFO_STREAM(logger_, COLOR_STR("Blacklist size is: " + std::to_string(frontier_blacklist_.size()), logger_.get_name()));
        std::cout << "Costmap pointer:" << costmap_ << std::endl;
        for (auto& frontier : frontier_list)
        {
            // Continue to next frontier if frontier is in blacklist
            bool frontier_exists_in_blacklist_ = false;
            if(frontier_blacklist_.count(frontier) > 0)
                frontier_exists_in_blacklist_ = true;

            // Preliminary checks and path planning for each frontier
            auto startTimePlan = std::chrono::high_resolution_clock::now();
            double length_to_frontier;
            if(use_planning_)
            {
                auto plan_of_frontier = costCalculator_->getPlanForFrontier(start_point_w, frontier, map_data, false, planner_allow_unknown_);
                // costCalculator_->getPlanForFrontierRRT(start_point_w, frontier, map_data, false, planner_allow_unknown_);
                // assume each pose is resolution * ((1 + root2) / 2) distance apart 
                length_to_frontier = static_cast<double>(plan_of_frontier.path.poses.size() * (costmap_->getResolution() * 1.207));
                // Continue to next frontier if path length is zero
                if (length_to_frontier == 0 || plan_of_frontier.success == false)
                {
                    RCLCPP_WARN_STREAM(logger_, COLOR_STR("Path length is zero or false", logger_.get_name()));
                    FrontierWithMetaData f_info_blacklisted(frontier, 0, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                    f_info_blacklisted.setCost(std::numeric_limits<double>::max());
                    frontier_costs[frontier] = f_info_blacklisted;
                    continue;
                }
                frontierPlanViz(plan_of_frontier.path);
            }
            else
            {
                length_to_frontier = sqrt(pow(start_point_w.x - frontier.goal_point.x, 2) + pow(start_point_w.y - frontier.goal_point.y, 2));
            }
            auto endTimePlan = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> durationPlan = (endTimePlan - startTimePlan);
            RCLCPP_DEBUG_STREAM(logger_, COLOR_STR("Time taken to Plan is: " + std::to_string(durationPlan.count()), logger_.get_name()));


            // Proceed to the function only if frontier is above the detection radius.
            if (length_to_frontier > frontierDetectRadius_ && frontier_exists_in_blacklist_ == false)
            {
                auto arrival_information_of_frontier = costCalculator_->getArrivalInformationForFrontier(frontier, polygon_xy_min_max);
                // Calculate frontier information and add to metadata vector
                // camera_fov / 2 is added because until here the maxIndex is only the starting index.
                FrontierWithMetaData f_info(frontier, arrival_information_of_frontier.information, arrival_information_of_frontier.theta_s_star_, length_to_frontier);
                frontier_meta_data_vector.push_back(f_info);
                max_arrival_info_per_frontier = std::max(max_arrival_info_per_frontier, arrival_information_of_frontier.information);
                min_traversable_distance = std::min(min_traversable_distance, length_to_frontier);
            }
            else
            {
                RCLCPP_ERROR_STREAM(logger_, COLOR_STR("Adding max cost: " + std::to_string(frontier.unique_id) + " ," + std::to_string(frontier.min_distance > frontierDetectRadius_) + " " + std::to_string(frontier_exists_in_blacklist_ == false), logger_.get_name()));
                FrontierWithMetaData f_info_blacklisted(frontier, 0, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                f_info_blacklisted.setCost(std::numeric_limits<double>::max());
                frontier_costs[frontier] = f_info_blacklisted;
            }
        } // frontier end
        std::vector<std::pair<FrontierWithMetaData, double>> frontier_with_u1_utility;
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(before) Frontier costs size u1 is: " + std::to_string(frontier_costs.size()), logger_.get_name()));

        // U1 Utility
        for (auto& frontier_with_properties : frontier_meta_data_vector)
        {
            // the distance term is inverted because we need to choose the closest frontier with tradeoff.
            auto utility = (alpha_ * (static_cast<double>(frontier_with_properties.information_) / static_cast<double>(max_arrival_info_per_frontier))) +
                           ((1.0 - alpha_) * (static_cast<double>(min_traversable_distance) / frontier_with_properties.path_length_));

            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 information:" << frontier_with_properties.information_);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 distance:" << min_traversable_distance);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 max info:" << max_arrival_info_per_frontier);
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1 max distance:" << frontier_with_properties.path_length_);
            // It is added with Beta multiplied. If the frontier lies in the N best then this value will be replaced with information on path.
            // If it does not lie in the N best then the information on path is treated as 0 and hence it is appropriate to multiply with beta.
            if(frontier_costs.count(frontier_with_properties.frontier_) == 1)
                throw std::runtime_error("Something is wrong. Duplicate frontiers?");
            frontier_with_properties.setCost((beta_ * utility) == 0 ? std::numeric_limits<double>::max() : 1 / (beta_ * utility));
            frontier_costs[frontier_with_properties.frontier_] = frontier_with_properties;
            frontier_with_u1_utility.push_back(std::make_pair(frontier_with_properties, beta_ * utility));
            RCLCPP_DEBUG_STREAM(logger_, "Utility U1:" << 1 / (beta_ * utility));
        }
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(after) Frontier u1 with utility size is: " + std::to_string(frontier_with_u1_utility.size()), logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("(after) Frontier costs size after u1 is: " + std::to_string(frontier_costs.size()), logger_.get_name()));

        RCLCPP_DEBUG_STREAM(logger_, "Alpha_: " << alpha_);
        RCLCPP_DEBUG_STREAM(logger_, "Beta_: " << beta_);
        // FrontierU1ComparatorCost frontier_u1_comp;
        // // sort the frontier list based on the utility value
        // std::sort(frontier_with_u1_utility.begin(), frontier_with_u1_utility.end(), frontier_u1_comp);

        // // U2 UTILITY processed for N best.
        // frontier_msgs::msg::Frontier::SharedPtr frontier_selected_post_u2;
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
        //             frontier_selected_post_u2 = std::make_shared<frontier_msgs::msg::Frontier>(frontier_with_u1_utility[m].first.frontier_);
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

        if(frontier_costs.size() != frontier_list.size())
        {
            throw std::runtime_error("The returned size is not the same as input size.");
        }

        RCLCPP_WARN_STREAM(logger_, COLOR_STR("The selected frontier was not updated after U2 computation.", logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Returning for input list size: " + std::to_string(frontier_list.size()), logger_.get_name()));
        RCLCPP_WARN_STREAM(logger_, COLOR_STR("Returning frontier costs size: " + std::to_string(frontier_costs.size()), logger_.get_name()));
        selection_result.success = true;
        selection_result.frontier_costs = frontier_costs;
        return selection_result;
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierClosest(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        // create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::max();

        // initialize with false, becomes true if frontier is selected.
        bool frontierSelectionFlag = false;
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found");
            frontierSelectionFlag = false;
            return std::make_pair(selected, false);
        }

        // Iterate through each frontier in the list
        for (auto &frontier : frontier_list)
        {
            // check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if (frontier.min_distance > frontierDetectRadius_)
            {
                bool frontier_exists_in_blacklist_ = false;
                // check if its blacklisted
                if(frontier_blacklist_.count(frontier) > 0)
                    frontier_exists_in_blacklist_ = true;
                if (frontier.min_distance < selected.min_distance && frontier_exists_in_blacklist_ == false)
                {
                    selected = frontier;
                    frontierSelectionFlag = true;
                }
            }
        }

        // If no frontier is selected, return
        if (frontierSelectionFlag == false)
        {
            RCLCPP_ERROR_STREAM(logger_, "No clustered frontiers are outside the minimum detection radius. Radius is: " << frontierDetectRadius_);
            return std::make_pair(selected, frontierSelectionFlag);
        }
        // Add the selected frontier to the blacklist to prevent re-selection
        return std::make_pair(selected, frontierSelectionFlag);
    }

    std::pair<frontier_msgs::msg::Frontier, bool> FrontierSelectionNode::selectFrontierRandom(std::vector<frontier_msgs::msg::Frontier> &frontier_list)
    {
        // create placeholder for selected frontier
        frontier_msgs::msg::Frontier selected;
        selected.min_distance = std::numeric_limits<double>::max();

        bool frontierSelectionFlag = false;
        if (frontier_list.size() == 0)
        {
            RCLCPP_ERROR(logger_, "No frontiers found");
            frontierSelectionFlag = false;
            return std::make_pair(selected, false);
        }

        // Create a vector to store eligible frontiers
        std::vector<frontier_msgs::msg::Frontier> frontier_list_imp;
        // Iterate through each frontier in the list
        for (auto &frontier : frontier_list)
        {
            // check if this frontier is the nearest to robot and ignore if very close (frontier_detect_radius)
            if (frontier.min_distance > frontierDetectRadius_)
            {
                bool frontier_exists_in_blacklist_ = false;
                if(frontier_blacklist_.count(frontier) > 0)
                    frontier_exists_in_blacklist_ = true;
                if (frontier_exists_in_blacklist_ == false)
                {
                    frontier_list_imp.push_back(frontier);
                }
            }
        }
        if (frontier_list_imp.size() > 0)
        {
            // Seed the random number generator
            std::random_device rd;
            std::mt19937 gen(rd());

            // Generate a random floating-point number between 0 and 1
            std::uniform_real_distribution<> dist(0.0, 1.0);
            double randomFraction = dist(gen);
            int randomIndex = static_cast<int>(randomFraction * (frontier_list_imp.size() - 1));
            selected = frontier_list_imp[randomIndex];
            frontierSelectionFlag = true;
        }
        else
        {
            frontierSelectionFlag = false;
        }

        if (frontierSelectionFlag == false)
        {
            RCLCPP_ERROR_STREAM(logger_, "No clustered frontiers are outside the minimum detection radius. Radius is: " << frontierDetectRadius_);
            return std::make_pair(selected, frontierSelectionFlag);
        }

        // Add the selected frontier to the blacklist
        return std::make_pair(selected, frontierSelectionFlag);
    }

    void FrontierSelectionNode::setFrontierBlacklist(std::vector<frontier_msgs::msg::Frontier>& blacklist)
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
