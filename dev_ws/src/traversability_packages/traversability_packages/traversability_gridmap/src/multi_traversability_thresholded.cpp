#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

class TraversabilityThresholdNode : public rclcpp::Node
{
public:
    TraversabilityThresholdNode() : Node("multi_traversability_threshold_node")
    {
        costmap_sub1_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "/scout_1/traversability_costmap/costmap_raw", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&TraversabilityThresholdNode::costmapCallback1, this, std::placeholders::_1)
        );

        costmap_sub2_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "/scout_2/traversability_costmap/costmap_raw", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&TraversabilityThresholdNode::costmapCallback2, this, std::placeholders::_1)
        );

        occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/traversability_thresholded", rclcpp::QoS(10).transient_local().reliable()
        );

        this->declare_parameter("threshold_value", rclcpp::ParameterValue(176));
        this->get_parameter("threshold_value", threshold_value_);
    }

private:
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub1_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_publisher_;
    int threshold_value_;
    nav_msgs::msg::OccupancyGrid thresholded_map;
    mutable std::mutex traversability_mutex_;
    bool map_initialized_ = false;

    void updateMergedMap(const nav2_msgs::msg::Costmap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(traversability_mutex_);
        if(map_initialized_ == false) {
            thresholded_map.header = msg->header;
            thresholded_map.info.origin = msg->metadata.origin;
            thresholded_map.info.map_load_time = msg->metadata.map_load_time;
            thresholded_map.info.resolution = msg->metadata.resolution;
            thresholded_map.info.width = msg->metadata.size_x;
            thresholded_map.info.height = msg->metadata.size_y;
            thresholded_map.data = std::vector<int8_t>(msg->data.size(), nav2_costmap_2d::NO_INFORMATION);
            map_initialized_ = true;
        }
        if(map_initialized_ == true) {
            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                if (msg->data[i] >= threshold_value_ && msg->data[i] != nav2_costmap_2d::NO_INFORMATION)
                {
                    // Set values above the threshold to 100 (Lethal Obstacle)
                    thresholded_map.data[i] = 100;
                }
                else if (msg->data[i] <= threshold_value_)
                {
                    if(thresholded_map.data[i] == -1)
                        thresholded_map.data[i] = 0;
                    else if(thresholded_map.data[i] == 100)
                        thresholded_map.data[i] = 100;
                    // Set other values to 0 (Free Space)
                }
            }
            occupancy_map_publisher_->publish(thresholded_map);
        }
    }

    void costmapCallback1(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        updateMergedMap(msg);
    }

    void costmapCallback2(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        updateMergedMap(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityThresholdNode>());
    rclcpp::shutdown();
    return 0;
}