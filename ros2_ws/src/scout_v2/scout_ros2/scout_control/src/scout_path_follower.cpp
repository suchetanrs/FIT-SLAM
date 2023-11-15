#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point.hpp>

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode() : Node("scout_path_follower")
  {
    // Subscribe to the "plan" topic
    plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "corrected_plan", 10, std::bind(&PathFollowerNode::planCallback, this, std::placeholders::_1));

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&PathFollowerNode::goalCallback, this, std::placeholders::_1));

    // Publish control commands on the "twist_stamped" topic
    control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    local_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("local_goal_topic", 10);
    goal_reached_ = this->create_publisher<geometry_msgs::msg::PointStamped>("goal_reached_path_follower", 10);

    this->declare_parameter("translation_speed", 0.15);
    this->get_parameter("translation_speed", max_translation_speed_);

    this->declare_parameter("rotation_speed", 0.2);
    this->get_parameter("rotation_speed", max_rotation_speed_);

    this->declare_parameter("local_goal_distance", 1.0);
    this->get_parameter("local_goal_distance", local_goal_distance_);

    this->declare_parameter("resolution", 0.25);
    this->get_parameter("resolution", map_resolution_);

    this->declare_parameter("robot_base_frame", "base_link");
    this->get_parameter("robot_base_frame", robot_base_frame_);

    this->declare_parameter("global_frame", "map");
    this->get_parameter("global_frame", global_frame_id_);

    tf_buffer2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer2_);

    rclcpp::Rate rate2 = rclcpp::Rate(30.0);
  }

  ~PathFollowerNode()
  {
    // Shutdown ROS 2 components properly
    plan_subscription_.reset(); // Unsubscribe and release the subscription
    control_publisher_.reset(); // Unregister the publisher

    // Shutdown the tf2_ros::Buffer and TransformListener
    tf_buffer2_.reset();
    tf_listener2_.reset();

    // Other cleanup tasks, if necessary

    RCLCPP_INFO(this->get_logger(), "PathFollowerNode destroyed");
  }
private:
    void planCallback(const nav_msgs::msg::Path plan)
    {
      current_plan_ = plan;
      RCLCPP_INFO(this->get_logger(), "New plan found");
      if(goal_recvd_flag_ == true) {
        poseCallback();
      }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped)  
    {
      goal_recvd_flag_ = true;
    }

    geometry_msgs::msg::Pose getRelativePoseGivenTwoPoints(geometry_msgs::msg::Point point_from, geometry_msgs::msg::Point point_to) {
        double dx, dy, theta;
        dx = point_to.x - point_from.x;
        dy = point_to.y - point_from.y;
        theta = atan2(dy, dx);
        geometry_msgs::msg::Pose oriented_pose;
        oriented_pose.position = point_from;
        oriented_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
        return oriented_pose;
    }

    std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf2_quaternion(
            quat.x, quat.y, quat.z, quat.w);

        // Convert tf2 quaternion to Euler angles
        tf2::Matrix3x3 matrix(tf2_quaternion);
        std::vector<double> rpy = {0, 0, 0};
        matrix.getRPY(rpy[0], rpy[1], rpy[2]);
        return rpy;
    }

    void nearestPoseTransform(geometry_msgs::msg::Pose& current_pose, geometry_msgs::msg::Pose& nearest_pose) {
      auto rate = rclcpp::Rate(20.0);
      while(!tf_buffer2_->canTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero) && rclcpp::ok())
      {
        rate.sleep();
        // RCLCPP_ERROR(rclcpp::get_logger("Traversability layer"), "Transform2 unavailable");
      }
      auto transform = tf_buffer2_->lookupTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero);
      // Compute control commands based on the path and current pose
      current_pose.position.x = transform.transform.translation.x;
      current_pose.position.y = transform.transform.translation.y;
      current_pose.position.z = transform.transform.translation.z;
      current_pose.orientation = transform.transform.rotation;
      nearest_pose.orientation = getRelativePoseGivenTwoPoints(current_pose.position, nearest_pose.position).orientation;
    }

    void nearestPoseTransformLastPose(geometry_msgs::msg::Pose& current_pose) {
      auto rate = rclcpp::Rate(20.0);
      while(!tf_buffer2_->canTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero) && rclcpp::ok())
      {
        rate.sleep();
        // RCLCPP_ERROR(rclcpp::get_logger("Traversability layer"), "Transform2 unavailable");
      }
      auto transform = tf_buffer2_->lookupTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero);
      // Compute control commands based on the path and current pose
      current_pose.position.x = transform.transform.translation.x;
      current_pose.position.y = transform.transform.translation.y;
      current_pose.position.z = transform.transform.translation.z;
      current_pose.orientation = transform.transform.rotation;
    }

    void poseCallback()
    {
        if(latest_pose_reached_ == true) {
          geometry_msgs::msg::Twist control_command;
          control_publisher_->publish(control_command);
        }
        latest_pose_reached_ = false;
        // Get the current position
        auto rate = rclcpp::Rate(20.0);
        while(!tf_buffer2_->canTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero) && rclcpp::ok())
        {
          rate.sleep();
        }
        auto transform = tf_buffer2_->lookupTransform(global_frame_id_, robot_base_frame_, tf2::TimePointZero);
        // Compute control commands based on the path and current pose
        geometry_msgs::msg::Pose current_pose;
        current_pose.position.x = transform.transform.translation.x;
        current_pose.position.y = transform.transform.translation.y;
        current_pose.position.z = transform.transform.translation.z;
        current_pose.orientation = transform.transform.rotation;

        double nearest_dist = std::numeric_limits<double>::max();
        geometry_msgs::msg::Pose nearest_pose = current_pose;
        for (size_t i = 0; i < current_plan_.poses.size();  i++) {
            double dist = compute2DDistance(current_pose, current_plan_.poses[i].pose);
            // Add distance ros parameter
            if(dist < nearest_dist && dist > local_goal_distance_) {
                nearest_dist = dist;
                nearest_pose = current_plan_.poses[i].pose;
                if(i > current_plan_.poses.size() - local_goal_distance_ / map_resolution_ || current_plan_.poses.size() < local_goal_distance_ / map_resolution_ || checkPosesTranslation(current_pose, current_plan_.poses[current_plan_.poses.size() - 1].pose, 0.2).first) {
                    latest_pose_reached_ = true;
                    nearest_pose = current_plan_.poses[current_plan_.poses.size() - 1].pose;
                    geometry_msgs::msg::PoseStamped nearest_pose_stamped;
                    nearest_pose_stamped.pose = nearest_pose;
                    nearest_pose_stamped.header.frame_id = global_frame_id_;
                    local_publisher_->publish(nearest_pose_stamped);
                    while(!checkPosesOrientation(current_pose, nearest_pose, 0.2).first && rclcpp::ok()) {
                      rclcpp::sleep_for(std::chrono::milliseconds(100));
                      nearestPoseTransformLastPose(current_pose);
                      RCLCPP_INFO(this->get_logger(), "Spinning for last pose!");
                      goal_recvd_flag_ = false;
                      geometry_msgs::msg::Twist control_command;
                      control_command.linear.x = 0.0; // Set your computed linear velocity here
                      if(checkPosesOrientation(current_pose, nearest_pose, 0.3).second > 0) {
                        control_command.angular.z = max_rotation_speed_; // Set your computed angular velocity here
                      }
                      else {
                        control_command.angular.z = -max_rotation_speed_;
                      }
                      control_publisher_->publish(control_command);
                    }
                    geometry_msgs::msg::PointStamped goal_reached_point_;
                    goal_reached_point_.point.x = 10;
                    goal_reached_->publish(goal_reached_point_);
                    }
                }
          }

          if(latest_pose_reached_ == false) {
              geometry_msgs::msg::PointStamped goal_reached_point_;
              goal_reached_->publish(goal_reached_point_);

              nearestPoseTransform(current_pose, nearest_pose);
              geometry_msgs::msg::PoseStamped nearest_pose_stamped;
              nearest_pose_stamped.pose = nearest_pose;
              nearest_pose_stamped.header.frame_id = global_frame_id_;
              local_publisher_->publish(nearest_pose_stamped);
              auto start_time = std::chrono::high_resolution_clock::now();
              // 0.4 is threshold
              while(!checkPosesOrientation(current_pose, nearest_pose, 0.4).first && rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                if(duration > 500) {
                  return;
                }
                nearestPoseTransform(current_pose, nearest_pose);
                geometry_msgs::msg::Twist control_command;
                control_command.linear.x = 0.0; // Set your computed linear velocity here
                if(checkPosesOrientation(current_pose, nearest_pose, 0.2).second > 0) {
                  control_command.angular.z = max_rotation_speed_; // Set your computed angular velocity here
                }
                else {
                  control_command.angular.z = -max_rotation_speed_;
                }
                control_publisher_->publish(control_command);
              }

              start_time = std::chrono::high_resolution_clock::now();
              while(!checkPosesTranslation(current_pose, nearest_pose, local_goal_distance_).first && rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                if(duration > 500) {
                  return;
                }
                nearestPoseTransform(current_pose, nearest_pose);
                geometry_msgs::msg::Twist control_command;
                control_command.linear.x = max_translation_speed_; // Set your computed linear velocity here
                control_publisher_->publish(control_command);
              }

              // Create and publish control commands
            }
    }

    std::pair<bool, double> checkPosesOrientation(
        geometry_msgs::msg::Pose& pose1,
        geometry_msgs::msg::Pose& pose2,
        double threshold)
    {
        auto rpy_p1 = quatToEuler(pose1.orientation); // current_pose
        auto rpy_p2 = quatToEuler(pose2.orientation);

        // Calculate the orientation difference around the z-axis
        double orientation_diff = rpy_p1[2] - rpy_p2[2];
        double result_diff = 0;
        if(rpy_p1[2] < 0) {
          rpy_p1[2] = rpy_p1[2] + 2 * M_PI;
        }
        if(rpy_p2[2] < 0) {
          rpy_p2[2] = rpy_p2[2] + 2 * M_PI;
        }
        if(rpy_p1[2] < rpy_p2[2]) { // goal leading
          if(abs(rpy_p2[2] - rpy_p1[2]) > M_PI) {
            result_diff = -100;
          }
          else {
            result_diff = 100;
          }
        }
        if(rpy_p1[2] > rpy_p2[2]) { // current leading
          if(abs(rpy_p1[2] - rpy_p2[2]) > M_PI) {
            result_diff = 100;
          }
          else {
            result_diff = -100;
          }
        }

        // Check if the orientation difference is less than the threshold
        return std::make_pair(abs(orientation_diff) < threshold, result_diff);
    }

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

    double compute2DDistance(
        const geometry_msgs::msg::Pose& pose1,
        const geometry_msgs::msg::Pose& pose2)
    {
        // Calculate the 2D Euclidean distance between the positions
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;

        return std::sqrt(dx * dx + dy * dy);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_reached_;
    tf2_ros::Buffer::SharedPtr tf_buffer2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener2_;
    double local_goal_distance_;
    double map_resolution_;
    bool goal_recvd_flag_ = true;
    std::string robot_base_frame_;
    std::string global_frame_id_;

    double max_translation_speed_;
    double max_rotation_speed_;

    nav_msgs::msg::Path current_plan_;
    bool latest_pose_reached_ = true;
    bool current_plan_received_ = false; // to make sure we get plan first.
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
