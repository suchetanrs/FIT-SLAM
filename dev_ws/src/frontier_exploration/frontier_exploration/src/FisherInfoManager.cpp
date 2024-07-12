#include "frontier_exploration/FisherInfoManager.hpp"

FisherInformationManager::FisherInformationManager(rclcpp::Node::SharedPtr node)
    : previous_keyframes_()
{
    node_ = node;
    client_ = node_->create_client<slam_msgs::srv::GetMap>("/scout_2/orb_slam3_get_map_data");
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FisherInformationManager::callService, this));
}

FisherInformationManager::~FisherInformationManager()
{
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
}

void FisherInformationManager::callService()
{
    RCLCPP_ERROR(node_->get_logger(), "Calling service");
    if (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for the map data service...");
        return;
    }

    auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
    request->tracked_points = false;

    auto result = client_->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(20)) == std::future_status::ready)
    {
        auto response = result.get();
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Poses in optimized graph: " << response->data.graph.poses.size());
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call the map data service.");
    }
}

void FisherInformationManager::checkKeyframeChanges(const slam_msgs::msg::MapGraph& keyframes)
{
    // for (int z = 0; z < keyframes.poses.size(); z++)
    // {
    //     auto it = previous_keyframes_.find(keyframes.poses_id[z]);
    //     if (it != previous_keyframes_.end())
    //     {
    //         // Check if the position has changed
    //         if (it->second. != keyframe.position)
    //         {
    //             RCLCPP_INFO(node_->get_logger(), "Keyframe %d has changed its position.", keyframe.id);
    //         }
    //     }
    //     // Update or add the keyframe in the map
    //     previous_keyframes_[keyframe.id] = keyframe;
    // }
}