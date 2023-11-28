#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/srv/get_map.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include <fstream>

class GetMapDataNode : public rclcpp::Node
{
public:
    GetMapDataNode() : Node("get_map_data_node_graph")
    {
        node = rclcpp::Node::make_shared("service_call_example");
        // Create a timer to call the service every 0.3 seconds
        subscription_ = this->create_subscription<rtabmap_msgs::msg::MapData>(
            "map_data",
            1, // Queue size
            std::bind(&GetMapDataNode::mapDataCallback, this, std::placeholders::_1)
        );
        client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
        // Initialize the previous poses
        previous_poses_id_.clear();
    }

private:
void mapDataCallback(const rtabmap_msgs::msg::MapData::SharedPtr data)
{
    // Compare the new poses with the previous poses
    for (size_t i=0; i<data->graph.poses.size(); i++)
    {
        if (!isOldPose(data->graph.poses_id[i]))
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "New pose with: " << data->graph.poses_id[i]);
            auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
            // RCLCPP_INFO(rclcpp::get_logger(""), "CH1");
            request->name = "scout";
            request->reference_frame = "world";
            // RCLCPP_INFO(rclcpp::get_logger(""), "CH2");
            rclcpp::Time current_time = this->get_clock()->now();
            double time_elapsed = current_time.seconds();
            // RCLCPP_INFO(rclcpp::get_logger(""), "CH3");
            // Call the service
            auto result = client->async_send_request(request);
            // RCLCPP_INFO(rclcpp::get_logger(""), "CH4");
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
                auto srv_res = result.get();
                if (srv_res->success) {
                    // RCLCPP_INFO_STREAM(node->get_logger(), "Service call was successful. " << srv_res->state.pose.position.x << ", " << srv_res->state.pose.position.y);
                    // Access the response data, e.g., result->state
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
                }
                full_data_map[data->graph.poses_id[i]] = std::make_pair(data->graph.poses[i], srv_res->state.pose);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed to complete.");
            }
        }
        else {
            auto temp_data = full_data_map[data->graph.poses_id[i]];
            full_data_map[data->graph.poses_id[i]] = std::make_pair(data->graph.poses[i], temp_data.second);
        }
    // Iterate through the map and write each entry to the CSV file
    // Open a file for writing
    std::ofstream file("random_opt_map_gt_3.csv");

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }
    for (const auto& entry : full_data_map) {
        int id = entry.first;
        const geometry_msgs::msg::Pose& pose1 = entry.second.first; // rtabmap estimate
        const geometry_msgs::msg::Pose& pose2 = entry.second.second; // grount truth

        // Convert Pose values to strings
        std::stringstream pose1_ss, pose2_ss;
        pose1_ss << pose1.position.x << "," << pose1.position.y << "," << pose1.position.z << ","
                 << pose1.orientation.x << "," << pose1.orientation.y << "," << pose1.orientation.z << "," << pose1.orientation.w;
        pose2_ss << pose2.position.x << "," << pose2.position.y << "," << pose2.position.z << ","
                 << pose2.orientation.x << "," << pose2.orientation.y << "," << pose2.orientation.z << "," << pose2.orientation.w;

        // Write the entry to the CSV file
        file << id << "," << pose1_ss.str() << "," << pose2_ss.str() << "\n";
    }
    file.close();
    }

    // Update the previous poses
    previous_poses_id_ = data->graph.poses_id;
}

    bool isOldPose(int id)
    {
        for (auto item : previous_poses_id_) {
            if(id == item) {
                return true;
            }
        }
        return false; // Modify this as needed
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int> previous_poses_id_;
    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr subscription_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client;
    rclcpp::Node::SharedPtr node;
    std::map<int, std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>> full_data_map;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetMapDataNode>());
    rclcpp::shutdown();
    return 0;
}