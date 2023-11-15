#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

class TraversabilityThresholdNode : public rclcpp::Node
{
public:
    TraversabilityThresholdNode() : Node("traversability_threshold_node")
    {
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "traversability_costmap/costmap_raw", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&TraversabilityThresholdNode::costmapCallback, this, std::placeholders::_1)
        );

        occupancy_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "traversability_thresholded", rclcpp::QoS(10).transient_local().reliable()
        );

        this->declare_parameter("threshold_value", rclcpp::ParameterValue(176));
        this->get_parameter("threshold_value", threshold_value_);

        this->declare_parameter("allow_unknown", rclcpp::ParameterValue(true));
        this->get_parameter("allow_unknown", allow_unknown_);
    }

private:
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_publisher_;
    int threshold_value_;
    bool allow_unknown_;

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Callback!");
        nav_msgs::msg::OccupancyGrid thresholded_map;
        thresholded_map.header = msg->header;
        thresholded_map.info.origin = msg->metadata.origin;
        thresholded_map.info.map_load_time = msg->metadata.map_load_time;
        thresholded_map.info.resolution = msg->metadata.resolution;
        thresholded_map.info.width = msg->metadata.size_x;
        thresholded_map.info.height = msg->metadata.size_y;
        thresholded_map.data = std::vector<int8_t>(msg->data.size(), nav2_costmap_2d::NO_INFORMATION);
        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            if (msg->data[i] == nav2_costmap_2d::NO_INFORMATION)
            {
                if(allow_unknown_) {
                    thresholded_map.data[i] = 0;
                }
                else {
                    thresholded_map.data[i] = nav2_costmap_2d::NO_INFORMATION;
                }
            }
            else if (msg->data[i] >= 176)
            {
                // Set values above the threshold to 100 (Lethal Obstacle)
                thresholded_map.data[i] = 100;
            }
            else
            {
                thresholded_map.data[i] = 0;
                // Set other values to 0 (Free Space)
            }
        }
        occupancy_map_publisher_->publish(thresholded_map);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityThresholdNode>());
    rclcpp::shutdown();
    return 0;
}