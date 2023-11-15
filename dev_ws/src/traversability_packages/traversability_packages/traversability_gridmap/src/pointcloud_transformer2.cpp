#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <traversability_msgs/msg/pcl2_with_node_id.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        subscription_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        subscription_map_data_ = this->create_subscription<rtabmap_msgs::msg::MapData>(
            "/map_data", 10,
            std::bind(&PointCloudTransformer::mapDataCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<traversability_msgs::msg::PCL2WithNodeID>("velodyne_transformed_points_nodeid", rclcpp::QoS(rclcpp::KeepLast(30)).reliable());

        transformed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        this->declare_parameter("transform_throttle", 0);
        this->get_parameter("transform_throttle", transform_throttle_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf2_quaternion(
            quat.x, quat.y, quat.z, quat.w);
        
        tf2_quaternion.normalize();

        // Convert tf2 quaternion to Euler angles
        tf2::Matrix3x3 matrix(tf2_quaternion);
        std::vector<double> rpy = {0, 0, 0};
        matrix.getRPY(rpy[0], rpy[1], rpy[2]);
        return rpy;
    }

    geometry_msgs::msg::Quaternion eulerToQuat(double roll, double pitch, double yaw) {
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(roll, pitch, yaw);

        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = tf2_quaternion.x();
        quat_msg.y = tf2_quaternion.y();
        quat_msg.z = tf2_quaternion.z();
        quat_msg.w = tf2_quaternion.w();

        return quat_msg;
    }

    geometry_msgs::msg::TransformStamped convertPoseToTransformStamped(geometry_msgs::msg::Pose pose, rclcpp::Time stamp) {
        geometry_msgs::msg::TransformStamped transformStamped;
        // Fill in the header.
        transformStamped.header.stamp = stamp; // Use current time.
        transformStamped.header.frame_id = "map"; // Set the frame ID for the transform.
        transformStamped.child_frame_id = "velodyne"; // Set the child frame ID for the transform.

        while(!tf_buffer_->canTransform("base_link", "velodyne", tf2::TimePointZero))
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            RCLCPP_ERROR(rclcpp::get_logger("Traversability layer"), "Transform2 unavailable");
        }
        auto transform = tf_buffer_->lookupTransform("base_link", "velodyne", tf2::TimePointZero);
        auto rpy_b_v = quatToEuler(transform.transform.rotation);
        auto rpy_m_b = quatToEuler(pose.orientation); // this is not the latest transform in the tree. This is the transform given by SLAM.

        // Fill in the transform.
        transformStamped.transform.translation.x = pose.position.x;
        transformStamped.transform.translation.y = pose.position.y;
        transformStamped.transform.translation.z = pose.position.z;
        transformStamped.transform.rotation = eulerToQuat(rpy_b_v[0] + rpy_m_b[0], rpy_b_v[1] + rpy_m_b[1], rpy_b_v[2] + rpy_m_b[2]);

        return transformStamped;
    }

    sensor_msgs::msg::PointCloud2 transformPCL(sensor_msgs::msg::PointCloud2 msg, geometry_msgs::msg::TransformStamped transform, std_msgs::msg::Header header)
    {
        // Transform the PCL with transform
        pcl::PCLPointCloud2 msg_cloud;
        // Convert message to pcl : out = msg_cloud
        pcl_conversions::toPCL(msg, msg_cloud);

        // Convert msg_cloud to pcl::PointCloud<T> : out = pcl_cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromPCLPointCloud2(msg_cloud, pcl_cloud);

        // Transform the pcl_cloud out: transfomed_cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl_ros::transformPointCloud(pcl_cloud, transformed_cloud, transform);

        // Convert tranformed_cloud to ros msg.
        sensor_msgs::msg::PointCloud2 transformed_msg;
        pcl::toROSMsg(transformed_cloud, transformed_msg);

        transformed_msg.header = header;

        return transformed_msg;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
    	pcl_message_throttle_++;
    	if(pcl_message_throttle_ > transform_throttle_) {
    	    pcl_message_throttle_ = 0;
            std::lock_guard<std::mutex> lock(pcl_buffer_mutex);
            pcl_buffer_.push_back(*msg);
            new_pcl = true;
        }
    }

    void mapDataCallback(const rtabmap_msgs::msg::MapData::SharedPtr map_msg)
    {        
        if (calledonceflag == false)
        {            
            int index_to_prune = -1;            
            size_t starting_index = previous_map_msg.graph.poses.size();            
            for (size_t i = starting_index; i < map_msg->graph.poses.size(); ++i)
            {
                int nodeid_pose = map_msg->graph.poses_id[i];
                auto last_pose_ = map_msg->graph.poses[i];
                if (new_pcl == true)
                {                    
                    sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg;
                    traversability_msgs::msg::PCL2WithNodeID pclwithnodeid;

                    // Sent in map frame.
                    std::lock_guard<std::mutex> lock(pcl_buffer_mutex);                  
                    for (size_t k=0; k<pcl_buffer_.size(); k++) {                        
                        auto pcl_selected = pcl_buffer_[k];                        
                        int64_t time_diff = (pcl_selected.header.stamp.sec - map_msg->header.stamp.sec) * 1e9 +
                                        (static_cast<int64_t>(pcl_selected.header.stamp.nanosec) - static_cast<int64_t>(map_msg->header.stamp.nanosec));                      
                        if(pcl_buffer_.size() > 0) {
                            if (abs(time_diff) > closest_time || k == pcl_buffer_.size()-1) {                                                                                                
                                sensor_msgs::msg::PointCloud2 closest_pcl;
                                if(k == pcl_buffer_.size()-1) {
                                    closest_pcl = pcl_buffer_[k];                  
                                }
                                else {
                                    closest_pcl = pcl_buffer_[k-1];
                                }
                                auto transform = convertPoseToTransformStamped(last_pose_, map_msg->header.stamp);                            
                                transformed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(transformPCL(closest_pcl, transform, map_msg->header));                            
                                transformed_publisher_->publish(*transformed_msg);
                                index_to_prune = k-1;
                                break;
                            }                            
                            closest_time = abs(time_diff);
                        }
                        if(pcl_buffer_.size() == 0) {
                            RCLCPP_ERROR(this->get_logger(), "PCL BUFFER SIZE = 0, This should never happen.");
                        }                        
                    }                    
                    closest_time = std::numeric_limits<int64_t>::max();
                    if(transformed_msg) {                        
                        pclwithnodeid.pcl = *transformed_msg;
                        pclwithnodeid.node_id = nodeid_pose;
                        pclwithnodeid.graph = map_msg->graph;
                        pclwithnodeid.pose = last_pose_;
                        publisher_->publish(pclwithnodeid);
                        RCLCPP_INFO_STREAM(get_logger(), "Published with NODEID: " << nodeid_pose);
                    }
                    else {
                        RCLCPP_ERROR(this->get_logger(), "Could not find any transformed messages. This should never happen.");
                        rclcpp::shutdown();
                    }
                    new_pcl = false;
                }
            }
            if(index_to_prune!= -1)
                pcl_buffer_.erase(pcl_buffer_.begin(), pcl_buffer_.begin() + index_to_prune);            
        }
        previous_map_msg = *map_msg;
        if (calledonceflag == true)
        {
            calledonceflag = false;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcl_;
    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr subscription_map_data_;
    rclcpp::Publisher<traversability_msgs::msg::PCL2WithNodeID>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_publisher_;
    std::vector<sensor_msgs::msg::PointCloud2> pcl_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    bool calledonceflag;
    bool new_pcl;
    int64_t closest_time = std::numeric_limits<int64_t>::max();
    rtabmap_msgs::msg::MapData previous_map_msg;
    std::mutex pcl_buffer_mutex;
    int pcl_message_throttle_ = 0;
    int transform_throttle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}
