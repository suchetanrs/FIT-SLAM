#include "frontier_exploration/FisherInfoManager.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

namespace frontier_exploration
{
    FisherInformationManager::FisherInformationManager(rclcpp::Node::SharedPtr node)
        : keyframe_poses_cache_()
    {
        node_ = node;
        client_node_ = rclcpp::Node::make_shared("FIMManagerClient");
        client_ = client_node_->create_client<slam_msgs::srv::GetLandmarksInView>("orb_slam3_get_landmarks_in_view");
        // rosVisualizer_ = std::make_shared<RosVisualizer>(client_node_);
    }

    FisherInformationManager::FisherInformationManager() 
    : keyframe_poses_cache_()
    {
    }

    FisherInformationManager::~FisherInformationManager()
    {
        node_.reset();
        client_node_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }

    // void FisherInformationManager::callServices()
    // {
    //     PROFILE_FUNCTION;
    //     std::vector<int> changedKFIds;
    //     {
    //         // call service to get keyframe positions
    //         RCLCPP_ERROR(client_node_->get_logger(), "Calling service");
    //         if (!client_->wait_for_service(std::chrono::seconds(1)))
    //         {
    //             RCLCPP_INFO(client_node_->get_logger(), "Waiting for the map data service...");
    //             return;
    //         }
    //         RCLCPP_WARN(client_node_->get_logger(), "Got service");

    //         auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
    //         request->tracked_points = false;

    //         auto result = client_->async_send_request(request);
    //         // if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
    //         if (rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    //         {
    //             auto response = result.get();
    //             checkKeyframeChanges(response->data.graph, changedKFIds);
    //             // RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Poses in optimized graph: " << response->data.graph.poses.size());
    //         }
    //         else
    //         {
    //             RCLCPP_ERROR(client_node_->get_logger(), "Failed to call the map data service.");
    //         }
    //     }

    //     {
    //         // call service to get landmarks of updated keyframes.
    //         RCLCPP_ERROR(client_node_->get_logger(), "Calling service2");
    //         if (!client_->wait_for_service(std::chrono::seconds(1)))
    //         {
    //             RCLCPP_INFO(client_node_->get_logger(), "Waiting for the map data service...");
    //             return;
    //         }
    //         RCLCPP_WARN(client_node_->get_logger(), "Got service2");

    //         auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
    //         request->tracked_points = true;
    //         request->kf_id_for_landmarks = changedKFIds;
    //         // RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Landmark for KFs: " << changedKFIds.size());
    //         for (int i: changedKFIds)
    //             std::cout << i << ' ';
    //         std::cout << std::endl;

    //         auto result = client_->async_send_request(request);
    //         // if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
    //         if (rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    //         {
    //             auto response = result.get();
    //             computeAndPopulateFIMs(response);
    //             int lm_count = 0;
    //             for (int k=0; k<response->data.nodes.size(); k++)
    //             {
    //                 lm_count = lm_count + response->data.nodes[k].word_pts.size();
    //             }
    //             RCLCPP_ERROR_STREAM(client_node_->get_logger(), "Landmark count: " << lm_count);
    //         }
    //         else
    //         {
    //             RCLCPP_ERROR(client_node_->get_logger(), "Failed to call the map data service.");
    //         }
    //     }
    // }

    // void FisherInformationManager::checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes, std::vector<int>& changedKFIds)
    // {
    //     // PROFILE_FUNCTION;
    //     for (int z = 0; z < keyframes.poses.size(); z++)
    //     {
    //         auto it = keyframe_poses_cache_.find(keyframes.poses_id[z]);
    //         if (it != keyframe_poses_cache_.end())
    //         {
    //             auto orient1 = quatToEuler(it->second.pose.orientation);
    //             auto orient2_quat = keyframes.poses[z].pose.orientation;
    //             auto orient2 = quatToEuler(orient2_quat);
    //             if(distanceBetweenPoints(it->second.pose.position, keyframes.poses[z].pose.position) > DISTANCE3D_THRESHOLD_KF_CHANGE && 
    //                                      abs(std::accumulate(orient1.begin(), orient1.end(), 0)) - abs(std::accumulate(orient2.begin(), orient2.end(), 0)) > ANGLESUM_THRESHOLD_KF_CHANGE)
    //             {
    //                 RCLCPP_INFO(node_->get_logger(), "Keyframe %d has changed its position.", keyframes.poses_id[z]);
    //                 keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
    //                 changedKFIds.push_back(keyframes.poses_id[z]);
    //             }
    //             // Check if the position has changed
    //             // if (it->second.pose != keyframes.poses[z].pose)
    //             // {
    //             //     RCLCPP_INFO(node_->get_logger(), "Keyframe %d has changed its position.", keyframes.poses_id[z]);
    //             //     keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
    //             //     changedKFIds.push_back(keyframes.poses_id[z]);
    //             // }
    //         }
    //         else
    //         {
    //             RCLCPP_INFO(node_->get_logger(), "Keyframe %d is a new keyframe.", keyframes.poses_id[z]);
    //             keyframe_poses_cache_[keyframes.poses_id[z]] = keyframes.poses[z];
    //             changedKFIds.push_back(keyframes.poses_id[z]);
    //         }
    //         // Update or add the keyframe in the map
    //     }
    //     RCLCPP_INFO(node_->get_logger(), "**************************");
    // }

    // void FisherInformationManager::computeAndPopulateFIMs(std::shared_ptr<slam_msgs::srv::GetMap_Response> response)
    // {
    //     for (auto &changedKF : response->data.nodes)
    //     {
    //         std::vector<Point2D> changedKFFOV;
    //         getFOVKeyframe(keyframe_poses_cache_[changedKF.id].pose, 2.5, 1.0472, changedKFFOV);
    //         // RosVisualizer::getInstance()landmarkViz(changedKFFOV);
    //         // rclcpp::sleep_for(std::chrono::seconds(1));
    //         std::vector<Frontier> closestNodeVector;
    //         FrontierRoadMap::getInstance().getNodesWithinRadius(keyframe_poses_cache_[changedKF.id].pose.position, closestNodeVector, RADIUS_TO_DECIDE_EDGES);
    //         for(auto& closerNode : closestNodeVector)
    //         {
    //             auto roadmap_ = FrontierRoadMap::getInstance().getRoadMap();
    //             FrontierRoadMap::getInstance().getRoadmapMutex().lock();
    //             auto nodeChildren = roadmap_[closerNode];
    //             FrontierRoadMap::getInstance().getRoadmapMutex().unlock();
    //             for(auto& nodeChild : nodeChildren)
    //             {
    //                 std::vector<Point2D> frontierPairFOV;
    //                 double pair_yaw = getFOVFrontierPair(closerNode, nodeChild, 1.0472, frontierPairFOV);
    //                 if(doFOVsOverlap(changedKFFOV, frontierPairFOV))
    //                 {
    //                     geometry_msgs::msg::Pose poseForEstimate;
    //                     poseForEstimate.position = closerNode.getGoalPoint();
    //                     poseForEstimate.orientation = yawToQuat(pair_yaw);
    //                     double information = computeInformationFrontierPair(changedKF.word_pts, keyframe_poses_cache_[changedKF.id].pose, poseForEstimate, frontierPairFOV);
    //                     fisher_information_map_[std::make_pair(closerNode, nodeChild)] = information;
    //                     // RosVisualizer::getInstance()landmarkViz(frontierPairFOV, 1.0, 0.0, 0.0);
    //                     // RCLCPP_INFO_STREAM(client_node_->get_logger(), "The information for KF: " << changedKF.id << " is: " << information);
    //                     // rclcpp::sleep_for(std::chrono::milliseconds(700));
    //                 }
    //                 else
    //                 {
    //                     // RosVisualizer::getInstance()landmarkViz(frontierPairFOV, 0.0, 1.0, 0.0);
    //                     // rclcpp::sleep_for(std::chrono::milliseconds(700));
    //                 }
    //             }
    //         }
    //     }
    // }

////////////////////////////////////////////////////////////////
    bool FisherInformationManager::isPoseSafe(geometry_msgs::msg::Point point_from,  geometry_msgs::msg::Point point_to)
    {
        // get relative pose
        geometry_msgs::msg::Pose relative_pose;
        getRelativePoseGivenTwoPoints(point_from, point_to, relative_pose);
        // call service to get landmarks visible from pose.
        RCLCPP_ERROR(client_node_->get_logger(), "Calling service");
        if (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(client_node_->get_logger(), "Waiting for the map data service...");
            return false;
        }
        RCLCPP_WARN(client_node_->get_logger(), "Got service");

        auto request = std::make_shared<slam_msgs::srv::GetLandmarksInView::Request>();
        auto response = std::make_shared<slam_msgs::srv::GetLandmarksInView::Response>();
        request->pose = relative_pose;
        request->max_angle_pose_observation = 4.0; // greater than pi to disregard angle of observation.
        request->max_dist_pose_observation = 14.0;

        auto result = client_->async_send_request(request);
        // if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
        if (rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            response = result.get();
        }
        else
        {
            RCLCPP_ERROR(client_node_->get_logger(), "Failed to call the map data service.");
            return false;
        }   
        RCLCPP_WARN_STREAM(client_node_->get_logger(), "Number of points is: " << response->map_points.size());
        if(response->map_points.size() > 150)
            return true;
        return false;  
    }

    void FisherInformationManager::generateLookupTable(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float step)
    {
        const std::string lookupFile = "/root/dev_ws/lookup_table_fi/fisher_information_lookup_table.dat";

        std::ofstream outfile(lookupFile, std::ios::binary);
        if (!outfile) {
            std::cerr << "Error opening file for writing" << std::endl;
            return;
        }

        minX = std::floor(minX * (1 / step)) * step;
        minY = std::floor(minY * (1 / step)) * step;
        minZ = std::floor(minZ * (1 / step)) * step;
        maxX = std::ceil(maxX * (1 / step)) * step;
        maxY = std::ceil(maxY * (1 / step)) * step;
        maxZ = std::ceil(maxZ * (1 / step)) * step;
        std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY << ", minZ: " << minZ << ", maxZ: " << maxZ << std::endl;
        for (float x = minX; x <= maxX; x += step)
        {
            for (float y = minY; y <= maxY; y += step)
            {
                for (float z = minZ; z <= maxZ; z += step)
                {
                    x = std::round(x * (1 / step)) * step;
                    y = std::round(y * (1 / step)) * step;
                    z = std::round(z * (1 / step)) * step;

                    LookupKey key;
                    key.components[0] = x;
                    key.components[1] = y;
                    key.components[2] = z;
                    Eigen::Vector3f landmark_point(x, y, z);
                    
                    Eigen::Matrix3f Q = Eigen::Matrix3f::Identity() * 0.00001; // Adjust this as needed
                    
                    float value = computeInformationOfPointLocal(landmark_point, Q);
                    outfile.write(reinterpret_cast<char*>(&key), sizeof(LookupKey));
                    outfile.write(reinterpret_cast<char*>(&value), sizeof(float));                    
                }
            }
        }

        outfile.close();
        std::cout << "Lookup table generated and saved to " << lookupFile << std::endl;
    }

    void FisherInformationManager::loadLookupTable()
    {
        lookup_table_fi_.clear();
        const std::string lookupFile = "fisher_information_lookup_table.dat";

        // Load the lookup table from the file
        std::ifstream infile(lookupFile, std::ios::binary);
        if (!infile) {
            std::cerr << "Error opening file for reading" << std::endl;
            return;
        }

        auto start = std::chrono::high_resolution_clock::now();

        LookupKey key;
        float value;
        while (infile.read(reinterpret_cast<char*>(&key), sizeof(LookupKey)) &&
            infile.read(reinterpret_cast<char*>(&value), sizeof(float))) {
            lookup_table_fi_[key] = value;
            // std::cout << key.components[0] << ", " << key.components[1] << ", " << key.components[2] << std::endl;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        infile.close();

        std::cout << "Lookup table loaded into unordered_map" << std::endl;
        std::cout << "Size: " << sizeof(lookup_table_fi_) << std::endl;
        std::cout << "Number of entries: " << lookup_table_fi_.size() << std::endl;
        std::cout << "Time taken to load: " << duration.count() << " ms" << std::endl;
    }

    float FisherInformationManager::getInformationFromLookup(Eigen::Vector3f& landmark_camera_frame, float step)
    {
        LookupKey search_key;
        search_key.components[0] = std::round(landmark_camera_frame.x() * (1 / step)) * step;
        search_key.components[1] = std::round(landmark_camera_frame.y() * (1 / step)) * step;
        search_key.components[2] = std::round(landmark_camera_frame.z() * (1 / step)) * step;
        auto it = lookup_table_fi_.find(search_key);
        if (it != lookup_table_fi_.end()) { 
            // std::cout << "Value for key {";
            // for (size_t i = 0; i < search_key.components.size(); ++i) {
            //     std::cout << search_key.components[i];
            //     if (i < search_key.components.size() - 1) std::cout << ", ";
            // }
            // std::cout << "}: " << it->second << std::endl;
            return it->second;
        } else {
            throw std::runtime_error("Key not found in the lookup table >> " + std::to_string(search_key.components[0]) + ", " + std::to_string(search_key.components[1]) + ", " + std::to_string(search_key.components[2]));
        }
    }
}