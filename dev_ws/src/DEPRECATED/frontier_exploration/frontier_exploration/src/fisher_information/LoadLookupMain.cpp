#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <fstream>
#include "frontier_exploration/Helpers.hpp"
#include "frontier_exploration/util/event_logger.hpp"
#include "frontier_exploration/fisher_information/FisherInfoManager.hpp"
#include "frontier_exploration/Parameters.hpp"

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher()
        : Node("pointcloud_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

        // Timer to publish point cloud data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PointCloudPublisher::publish_pointcloud, this));

        try {
            fim_manager.loadLookupTable();
            std::cout << "Lookup table loaded successfully." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error loading lookup table: " << e.what() << std::endl;
        }
    }

private:
    void publish_pointcloud()
    {
        auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Setup point cloud data
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.width = 10321;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);
        cloud.clear();

        float x = 0.0f;  // Position in x-axis
        float y = 0.0f;  // Position in y-axis
        float z = 0.0f; // Position in z-axis

        float roll = 0.0;  // Roll (rotation around x-axis)
        float pitch = 0.0; // Pitch (rotation around y-axis)
        float yaw = 0.0;  // Yaw (rotation around z-axis)

        // Convert roll, pitch, yaw to a quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

        // Set the pose with the position and quaternion
        geometry_msgs::msg::PoseStamped pose_stamped;
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
        pose_stamped.pose = pose;
        pose_stamped.header.frame_id = "base_link";
        pose_stamped.header.stamp = this->now();

        // Get the transformation matrix from the pose
        Eigen::Affine3f T_w_c_est = frontier_exploration::getTransformFromPose(pose);
        // std::cout << T_w_c_est.translation() << std::endl;
        // std::cout << "**" << std::endl;
        // std::cout << T_w_c_est.rotation() << std::endl;
        double info_sum = 0;
        const float epsilon = 1e-6;

        // Open a CSV file for writing
        std::ofstream file("pointcloud_big.csv");
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing." << std::endl;
            return;
        }

        // Write the CSV header
        file << "x,y,z,intensity\n";
        float step_min = 0.03;
        float step_max = 0.3;

        for (float dx = 0; dx <= 2.2; dx += step_min)
        {
            for (float dy = -3.0; dy <= 3.0; dy += step_min)
            {
                for (float dz = -3.0; dz <= 3.0; dz += step_min)
                {
                    auto landmark_x = x + dx;
                    auto landmark_y = y + dy;
                    auto landmark_z = z + dz;
                    // Define the point in the world coordinates
                    Eigen::Vector3f p3d_w_eig(landmark_x, landmark_y, landmark_z); // Adjust this as needed
                    Eigen::Vector3f cam_pose(1.0, 0.0, 0.0);
                    Eigen::Vector3f p3d_c_eig = T_w_c_est.inverse() * p3d_w_eig;
                    float voxel_x, voxel_y, voxel_z;
                    if(!fim_manager.getVoxelCoordinate(voxel_x, voxel_y, voxel_z, landmark_x, landmark_y, landmark_z))
                    {
                        // LOG_ERROR("Could not get voxel coordinate for " << landmark_x << ", " << landmark_y << ", " << landmark_z);
                    }
                    // std::cout << "cam: " << p3d_c_eig.normalized() << std::endl;
                    // std::cout << "world: " << cam_pose.normalized() << std::endl;
                    // std::cout << "cosine: " << abs(acos(p3d_c_eig.normalized().dot(cam_pose.normalized()))) << std::endl;
                    if(abs(acos(p3d_c_eig.normalized().dot(cam_pose.normalized()))) > 1.0)
                        continue;
                    // Compute the information of the point
                    float information = fim_manager.getInformationFromLookup(p3d_c_eig, step_max, step_min, 0);
                    landmark_x = voxel_x;
                    landmark_y = voxel_y;
                    landmark_z = voxel_z;
                    if(std::isnan(information))
                        continue;
                    info_sum += information;
                    pcl::PointXYZI pclPoint;
                    pclPoint.x = landmark_x;
                    pclPoint.y = landmark_y;
                    pclPoint.z = landmark_z;
                    pclPoint.intensity = information;
                    // Print the result
                    cloud.push_back(pclPoint);
                    if (information > 2.5)
                    {
                        // std::cout << information << ", ";
                        // std::cout << "Info greater thnan 2.5" << std::endl;
                    }
                    file << pclPoint.x << "," << pclPoint.y << "," << pclPoint.z << "," << pclPoint.intensity << "\n";
                }
            }
        }
        std::cout << "LOOP COMPLETE" << info_sum << std::endl;

        pcl::toROSMsg(cloud, *pointcloud_msg);
        file.close();

        pointcloud_msg->header.frame_id = "base_link";
        pointcloud_msg->header.stamp = this->now();

        publisher_->publish(*pointcloud_msg);
        pose_publisher_->publish(pose_stamped);
        // exit(0);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    frontier_exploration::FisherInformationManager fim_manager;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}

