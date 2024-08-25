#include "frontier_exploration/FisherInfoManager.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

namespace frontier_exploration
{
    FisherInformationManager::FisherInformationManager(rclcpp::Node::SharedPtr node, std::shared_ptr<FrontierRoadMap> roadmap)
        : keyframe_poses_cache_()
    {
        node_ = node;
        client_node_ = rclcpp::Node::make_shared("FIMManagerClient");
        client_ = client_node_->create_client<slam_msgs::srv::GetMap>("/scout_2/orb_slam3_get_map_data");
        timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        roadmap_ptr_ = roadmap;
        timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FisherInformationManager::callServices, this), timer_cb_group_);
        // rosVisualizer_ = std::make_shared<RosVisualizer>(client_node_);
    }

    FisherInformationManager::~FisherInformationManager()
    {
        node_.reset();
        client_node_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }

    void FisherInformationManager::callServices()
    {
        PROFILE_FUNCTION;
        std::vector<int> changedKFIds;
        {
            // call service to get keyframe positions
            RCLCPP_ERROR(client_node_->get_logger(), "Calling service");
            if (!client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(client_node_->get_logger(), "Waiting for the map data service...");
                return;
            }
            RCLCPP_WARN(client_node_->get_logger(), "Got service");

            auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
            request->tracked_points = false;

            auto result = client_->async_send_request(request);
            // if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
            if (rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                checkKeyframeChanges(response->data.graph, changedKFIds);
                // RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Poses in optimized graph: " << response->data.graph.poses.size());
            }
            else
            {
                RCLCPP_ERROR(client_node_->get_logger(), "Failed to call the map data service.");
            }
        }

        {
            // call service to get landmarks of updated keyframes.
            RCLCPP_ERROR(client_node_->get_logger(), "Calling service2");
            if (!client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(client_node_->get_logger(), "Waiting for the map data service...");
                return;
            }
            RCLCPP_WARN(client_node_->get_logger(), "Got service2");

            auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
            request->tracked_points = true;
            request->kf_id_for_landmarks = changedKFIds;
            // RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Landmark for KFs: " << changedKFIds.size());
            for (int i: changedKFIds)
                std::cout << i << ' ';
            std::cout << std::endl;

            auto result = client_->async_send_request(request);
            // if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
            if (rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                computeAndPopulateFIMs(response);
                int lm_count = 0;
                for (int k=0; k<response->data.nodes.size(); k++)
                {
                    lm_count = lm_count + response->data.nodes[k].word_pts.size();
                }
                RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Landmark count: " << lm_count);
            }
            else
            {
                RCLCPP_ERROR(client_node_->get_logger(), "Failed to call the map data service.");
            }
        }
    }

    void FisherInformationManager::checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes, std::vector<int>& changedKFIds)
    {
        // PROFILE_FUNCTION;
        for (int z = 0; z < keyframes.poses.size(); z++)
        {
            auto it = keyframe_poses_cache_.find(keyframes.poses_id[z]);
            if (it != keyframe_poses_cache_.end())
            {
                auto orient1 = quatToEuler(it->second.pose.orientation);
                auto orient2_quat = keyframes.poses[z].pose.orientation;
                auto orient2 = quatToEuler(orient2_quat);
                if(distanceBetweenPoints(it->second.pose.position, keyframes.poses[z].pose.position) > DISTANCE3D_THRESHOLD_KF_CHANGE && 
                                         abs(std::accumulate(orient1.begin(), orient1.end(), 0)) - abs(std::accumulate(orient2.begin(), orient2.end(), 0)) > ANGLESUM_THRESHOLD_KF_CHANGE)
                {
                    RCLCPP_INFO(node_->get_logger(), "Keyframe %d has changed its position.", keyframes.poses_id[z]);
                    keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
                    changedKFIds.push_back(keyframes.poses_id[z]);
                }
                // Check if the position has changed
                // if (it->second.pose != keyframes.poses[z].pose)
                // {
                //     RCLCPP_INFO(node_->get_logger(), "Keyframe %d has changed its position.", keyframes.poses_id[z]);
                //     keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
                //     changedKFIds.push_back(keyframes.poses_id[z]);
                // }
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Keyframe %d is a new keyframe.", keyframes.poses_id[z]);
                keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
                changedKFIds.push_back(keyframes.poses_id[z]);
            }
            // Update or add the keyframe in the map
        }
        RCLCPP_INFO(node_->get_logger(), "**************************");
    }

    void FisherInformationManager::computeAndPopulateFIMs(std::shared_ptr<slam_msgs::srv::GetMap_Response> response)
    {
        for (auto &changedKF : response->data.nodes)
        {
            std::vector<Point2D> changedKFFOV;
            getFOVKeyframe(keyframe_poses_cache_[changedKF.id].pose, 2.5, 1.0472, changedKFFOV);
            // RosVisualizer::getInstance()landmarkViz(changedKFFOV);
            // rclcpp::sleep_for(std::chrono::seconds(1));
            std::vector<Frontier> closestNodeVector;
            roadmap_ptr_->getNodesWithinRadius(keyframe_poses_cache_[changedKF.id].pose.position, closestNodeVector, RADIUS_TO_DECIDE_EDGES);
            for(auto& closerNode : closestNodeVector)
            {
                auto roadmap_ = roadmap_ptr_->getRoadMap();
                roadmap_ptr_->getRoadmapMutex().lock();
                auto nodeChildren = roadmap_[closerNode];
                roadmap_ptr_->getRoadmapMutex().unlock();
                for(auto& nodeChild : nodeChildren)
                {
                    std::vector<Point2D> frontierPairFOV;
                    double pair_yaw = getFOVFrontierPair(closerNode, nodeChild, 1.0472, frontierPairFOV);
                    if(doFOVsOverlap(changedKFFOV, frontierPairFOV))
                    {
                        geometry_msgs::msg::Pose poseForEstimate;
                        poseForEstimate.position = closerNode.getGoalPoint();
                        poseForEstimate.orientation = yawToQuat(pair_yaw);
                        double information = computeInformationFrontierPair(changedKF.word_pts, keyframe_poses_cache_[changedKF.id].pose, poseForEstimate, frontierPairFOV);
                        fisher_information_map_[std::make_pair(closerNode, nodeChild)] = information;
                        // RosVisualizer::getInstance()landmarkViz(frontierPairFOV, 1.0, 0.0, 0.0);
                        // RCLCPP_INFO_STREAM(client_node_->get_logger(), "The information for KF: " << changedKF.id << " is: " << information);
                        // rclcpp::sleep_for(std::chrono::milliseconds(700));
                    }
                    else
                    {
                        // RosVisualizer::getInstance()landmarkViz(frontierPairFOV, 0.0, 1.0, 0.0);
                        // rclcpp::sleep_for(std::chrono::milliseconds(700));
                    }
                }
            }
        }
    }
    
}