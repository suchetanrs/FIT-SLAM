#include "frontier_exploration/fisher_information/FisherInfoManager.hpp"
#include "frontier_exploration/util/GeometryUtils.hpp"

namespace frontier_exploration
{
    FisherInformationManager::FisherInformationManager(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        client_node_ = rclcpp::Node::make_shared("FIMManagerClient");
        client_ = client_node_->create_client<slam_msgs::srv::GetLandmarksInView>("orb_slam3_get_landmarks_in_view");
        // rosVisualizer_ = std::make_shared<RosVisualizer>(client_node_);
        loadLookupTable();
        latest_version_ = 1;
        fim_pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("fim_pointcloud", 10);
    }

    FisherInformationManager::FisherInformationManager()
    {
    }

    FisherInformationManager::~FisherInformationManager()
    {
        node_.reset();
        client_node_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }

////////////////////////////////////////////////////////////////
    bool FisherInformationManager::isPoseSafe(geometry_msgs::msg::Point point_from,  geometry_msgs::msg::Point point_to, bool exhaustiveSearch)
    {
        geometry_msgs::msg::Pose relative_pose;
        getRelativePoseGivenTwoPoints(point_from, point_to, relative_pose);
        return isPoseSafe(relative_pose, exhaustiveSearch);
    }

    bool FisherInformationManager::isPoseSafe(geometry_msgs::msg::Pose& given_pose, bool exhaustiveSearch)
    {
        // fi_pointcloud_pcl_.clear();
        // fi_pointcloud_pcl_.width = 10201;
        // fi_pointcloud_pcl_.height = 1;
        // fi_pointcloud_pcl_.points.resize(fi_pointcloud_pcl_.width * fi_pointcloud_pcl_.height);

        ++latest_version_;
        // get relative pose
        geometry_msgs::msg::Pose relative_pose = given_pose;
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
        request->exhaustive_search = exhaustiveSearch;

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
        // Convert map_points to the camera frame
        tf2::Transform world_to_camera;
        tf2::fromMsg(relative_pose, world_to_camera); // Convert Pose message to a tf2 Transform
        tf2::Transform camera_to_world = world_to_camera.inverse(); // Inverse for world to camera frame transformation
        float total_information = 0.0;
        for (const auto& map_point : response->map_points)
        {
            tf2::Vector3 point_in_world(map_point.position.x, map_point.position.y, map_point.position.z);
            tf2::Vector3 point_in_camera = camera_to_world * point_in_world;
            float info = getInformationFromLookup(point_in_camera, 0.3, latest_version_);
            total_information += info;
            LOG_TRACE("Point: x: " << point_in_camera.x() << " y: " << point_in_camera.y() << " z: " << point_in_camera.z() << " information: " << info);
        }
        LOG_WARN("Total information: " << total_information);
        // sensor_msgs::msg::PointCloud2 fi_pointcloud_;
        // pcl::toROSMsg(fi_pointcloud_pcl_, fi_pointcloud_);

        // fi_pointcloud_.header.frame_id = "robot_0/base_link";
        // fi_pointcloud_.header.stamp = node_->now();

        // fim_pointcloud_pub_->publish(fi_pointcloud_);
        // if(response->map_points.size() > 150)
        //     return true;
        // return false;  
        if(total_information > 419.5)
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
                    
                    Eigen::Matrix3f Q = Eigen::Matrix3f::Identity() * 1.0; // Adjust this as needed
                    
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
        const std::string lookupFile = "/root/dev_ws/src/lookup_table_fi/fisher_information_lookup_table.dat";

        // Load the lookup table from the file
        std::ifstream infile(lookupFile, std::ios::binary);
        if (!infile) {
            std::cerr << "Error opening file for reading" << std::endl;
            throw std::runtime_error("Cannot load lookup table. Does it exist in the path?");
        }

        auto start = std::chrono::high_resolution_clock::now();

        LookupKey key;
        float value;
        while (infile.read(reinterpret_cast<char*>(&key), sizeof(LookupKey)) &&
            infile.read(reinterpret_cast<char*>(&value), sizeof(float))) {
            lookup_table_fi_[key].information = value;
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

    float FisherInformationManager::getInformationFromLookup(Eigen::Vector3f& landmark_camera_frame, float step, unsigned int latest_version)
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
            return it->second.information;
        } else {
            return 0.0;
            // throw std::runtime_error("Key not found in the lookup table >> " + std::to_string(search_key.components[0]) + ", " + std::to_string(search_key.components[1]) + ", " + std::to_string(search_key.components[2]));
            LOG_WARN("Key not found in the lookup table >> " + std::to_string(search_key.components[0]) + ", " + std::to_string(search_key.components[1]) + ", " + std::to_string(search_key.components[2]));
        }
    }

    float FisherInformationManager::getInformationFromLookup(tf2::Vector3& landmark_camera_frame, float step, unsigned int latest_version)
    {
        LookupKey search_key;
        // search_key.components[0] = std::round(landmark_camera_frame.x() * (1 / step)) * step;
        search_key.components[0] = 1.2;
        search_key.components[1] = std::round(landmark_camera_frame.y() * (1 / step)) * step;
        search_key.components[2] = std::round(landmark_camera_frame.z() * (1 / step)) * step;
        auto it = lookup_table_fi_.find(search_key);
        if (it != lookup_table_fi_.end()) { 
            if (it->second.version == latest_version) {
                // Voxel is part of the current query
                it->second.pointCount++;
            } else {
                // Voxel is from a previous query, reset it
                it->second.pointCount = 1;
                it->second.version = latest_version;
                // pcl::PointXYZI pclPoint;
                // pclPoint.x = search_key.components[0];
                // pclPoint.y = search_key.components[1];
                // pclPoint.z = search_key.components[2];
                // pclPoint.intensity = 2.0;
                // fi_pointcloud_pcl_.push_back(pclPoint);
            }
            // std::cout << "Value for key {";
            // for (size_t i = 0; i < search_key.components.size(); ++i) {
            //     std::cout << search_key.components[i];
            //     if (i < search_key.components.size() - 1) std::cout << ", ";
            // }
            // std::cout << "}: " << it->second << std::endl;
            return it->second.information * getFactorFromNum(it->second.pointCount, landmark_camera_frame.x());
        } else {
            return 0.0;
            // throw std::runtime_error("Key not found in the lookup table >> " + std::to_string(search_key.components[0]) + ", " + std::to_string(search_key.components[1]) + ", " + std::to_string(search_key.components[2]));
            LOG_WARN("Key not found in the lookup table >> " + std::to_string(search_key.components[0]) + ", " + std::to_string(search_key.components[1]) + ", " + std::to_string(search_key.components[2]));
        }
    }
}