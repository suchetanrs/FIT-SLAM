#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/path.hpp"

class PoseArraySubscriber : public rclcpp::Node
{
public:
    PoseArraySubscriber() : Node("pose_array_subscriber")
    {
        subscription_ = this->create_subscription<rtabmap_msgs::msg::Path>(
            "pose_graph", 10,
            std::bind(&PoseArraySubscriber::poseArrayCallback, this, std::placeholders::_1));

        // Initialize the previous message
        previous_pose_array_.poses.resize(0); // Initialize with an empty array
    }

private:
    void poseArrayCallback(const rtabmap_msgs::msg::Path::SharedPtr msg)
    {
        // Compare each element of the new message with the previous one
        for (size_t i = 0; i < previous_pose_array_.poses.size(); ++i)
        {
            if (msg->poses[i] != previous_pose_array_.poses[i])
            {
                RCLCPP_INFO(this->get_logger(), "Element at index %zu has changed!", i);
            }

            if (msg->node_ids[i] != previous_pose_array_.node_ids[i])
            {
                RCLCPP_INFO(this->get_logger(), "NODE ID at index %zu has changed!", i);
            }
        }

        // // Update the previous message
        previous_pose_array_ = *msg;
    }

    rclcpp::Subscription<rtabmap_msgs::msg::Path>::SharedPtr subscription_;
    rtabmap_msgs::msg::Path previous_pose_array_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseArraySubscriber>());
    rclcpp::shutdown();
    return 0;
}