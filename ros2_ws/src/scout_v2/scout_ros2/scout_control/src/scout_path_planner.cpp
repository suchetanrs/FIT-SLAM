#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathCorrectorNode : public rclcpp::Node
{
public:
  PathCorrectorNode()
    : Node("path_corrector_node")
  {
    // Create a subscription to the "plan" topic
    subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "plan", 10, std::bind(&PathCorrectorNode::planCallback, this, std::placeholders::_1));

    // Create a publisher for the "corrected_plan" topic
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("corrected_plan", 10);

    tf_buffer2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer2_);

    // Set the publishing frequency to 20 Hz (50 ms period)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PathCorrectorNode::publishCorrectedPlan, this));

    eraser_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30), std::bind(&PathCorrectorNode::removePoses, this));
  }

private:
  std::pair<bool, double> checkPosesTranslation(
      const geometry_msgs::msg::Pose& pose1,
      const geometry_msgs::msg::Pose& pose2,
      double threshold)
  {
      // Calculate the 2D Euclidean distance between the positions
      double dx = pose1.position.x - pose2.position.x;
      double dy = pose1.position.y - pose2.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // Check if the distance is less than the threshold
      return std::make_pair(distance < threshold, distance);
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // This is the callback function that is called when a new message is received on the "plan" topic.
    // You can add your plan correction logic here if needed.
    
    // Publish the received plan as-is to the "corrected_plan" topic for now.
    latest_plan_ = *msg;
    latest_plan_flag = true;
  }

  void removePoses() {
      auto rate = rclcpp::Rate(20.0);
      while(!tf_buffer2_->canTransform("map", "base_link", tf2::TimePointZero) && rclcpp::ok())
      {
        rate.sleep();
      }
      auto transform = tf_buffer2_->lookupTransform("map", "base_link", tf2::TimePointZero);
      geometry_msgs::msg::Pose current_pose;
      current_pose.position.x = transform.transform.translation.x;
      current_pose.position.y = transform.transform.translation.y;
      current_pose.position.z = transform.transform.translation.z;
      //Keep removing elements in the plan as the robot traverses them except for the last element.
      if (latest_plan_.poses.size() >= 2) {
        auto it = latest_plan_.poses.begin();
        while (it != latest_plan_.poses.end()) {
            auto ans = checkPosesTranslation(it->pose, current_pose, 0.2);
            if (ans.first) {
                // If ans is true, remove the element from latest_plan_
                it = latest_plan_.poses.erase(latest_plan_.poses.begin(), it + 1);
            } else {
                ++it;
            }
        }
      }
  }

  void publishCorrectedPlan()
  {
    if(latest_plan_flag == true) {
      publisher_->publish(latest_plan_);
    }
    // This function is called at a frequency of 20 Hz (50 ms period) to publish the corrected plan.
    // You can perform any additional plan correction and publishing logic here.
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  tf2_ros::Buffer::SharedPtr tf_buffer2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener2_;
  nav_msgs::msg::Path latest_plan_;
  bool latest_plan_flag = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr eraser_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
