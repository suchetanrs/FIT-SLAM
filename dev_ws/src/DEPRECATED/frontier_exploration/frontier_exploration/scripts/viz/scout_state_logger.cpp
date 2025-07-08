#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include <fstream>

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("service_call_example");
    auto start_time_ = node->get_clock()->now();
    // Create a client for the /gazebo/get_entity_state service
    auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    rclcpp::sleep_for(std::chrono::seconds(20));
    // Wait for the service to become available
    if (!client->wait_for_service(std::chrono::seconds(60))) {
        RCLCPP_ERROR(node->get_logger(), "Service not available.");
        return 1;
    }

    // Create a request message
    while(rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(600));
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = "scout";
        request->reference_frame = "world";
        rclcpp::Time current_time = node->get_clock()->now();
        double time_elapsed = current_time.seconds();
        // Call the service
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto srv_res = result.get();
            if (srv_res->success) {
                RCLCPP_INFO_STREAM(node->get_logger(), "Service call was successful. " << srv_res->state.pose.position.x << ", " << srv_res->state.pose.position.y);
                std::ofstream csv_file_1_;
                csv_file_1_ << std::fixed;
                csv_file_1_ << std::setprecision(2);
                csv_file_1_.open("3_gt_xy_3.csv", std::ios::app);
                csv_file_1_ << time_elapsed << "," << srv_res->state.pose.position.x << "," << srv_res->state.pose.position.y << "\n";
                csv_file_1_.close();
                // Access the response data, e.g., result->state
            } else {
                RCLCPP_ERROR(node->get_logger(), "Service call failed.");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Service call failed to complete.");
        }
    }

    // Shutdown the ROS 2 node
    rclcpp::shutdown();
    return 0;
}