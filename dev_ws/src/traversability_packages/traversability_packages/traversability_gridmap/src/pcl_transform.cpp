#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <slam_msgs/msg/map_data.hpp>
#include <traversability_msgs/msg/pcl2_with_node_id.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("suchetan_pointcloud_transformer2222")
    {
        subscription_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scout_2/velodyne_points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        transformed_publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_point_cloud_pcl", 10);
        transformed_publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_point_cloud_tf2", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::chrono::nanoseconds duration;
        std::chrono::_V2::system_clock::time_point start_time, end_time;
        start_time = std::chrono::high_resolution_clock::now();
        try
        {
            // Get transform from Velodyne frame to base_link frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "scout_2/base_link",  // target frame
                msg->header.frame_id,  // source frame (point cloud frame)
                msg->header.stamp);  // time stamp of the point cloud

            // Transform the point cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            pcl_ros::transformPointCloud("/scout_2/base_link", transform, *msg, transformed_cloud);

            // Publish the transformed point cloud
            transformed_publisher1_->publish(transformed_cloud);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
        }
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        std::cout << "Callback duration: microseconds is1: " << duration.count() * 1e-9 << std::endl;

        start_time = std::chrono::high_resolution_clock::now();
        try
        {
            // Get transform from Velodyne frame to base_link frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "scout_2/base_link",  // target frame
                msg->header.frame_id,  // source frame (point cloud frame)
                msg->header.stamp);  // time stamp of the point cloud

            // Transform the point cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);

            // Publish the transformed point cloud
            transformed_publisher2_->publish(transformed_cloud);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
        }
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        std::cout << "Callback duration: microseconds is2: " << duration.count() * 1e-9 << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcl_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_publisher1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_publisher2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}