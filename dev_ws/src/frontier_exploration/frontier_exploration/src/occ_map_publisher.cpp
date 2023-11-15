#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("occupancy_map_publisher", options);

    // Create an occupancy grid message
    auto occupancy_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    
    // Set the header
    occupancy_map->header.frame_id = "map";
    occupancy_map->header.stamp = node->now();

    // Set the map info
    occupancy_map->info.resolution = 0.25; // 0.25 meters per cell
    occupancy_map->info.width = 40; // 10 meters width / 0.25 resolution
    occupancy_map->info.height = 40; // 10 meters height / 0.25 resolution
    occupancy_map->info.origin.position.x = -5.0; // Adjust as needed
    occupancy_map->info.origin.position.y = -5.0; // Adjust as needed
    occupancy_map->info.origin.position.z = 0.0;

    // Populate the data with unknown regions (value: -1)
    occupancy_map->data.assign(occupancy_map->info.width * occupancy_map->info.height, -1);

    // Set some unoccupied regions (value: 0)
    // For example, let's set a 2x2 meter unoccupied region at the center of the map
    int center_x = occupancy_map->info.width / 2;
    int center_y = occupancy_map->info.height / 2;
    int unoccupied_width = 4 / occupancy_map->info.resolution;
    int unoccupied_height = 4 / occupancy_map->info.resolution;
    for (int i = center_x - unoccupied_width / 2; i < center_x + unoccupied_width / 2; i++) {
        for (int j = center_y - unoccupied_height / 2; j < center_y + unoccupied_height / 2; j++) {
            int index = i + j * occupancy_map->info.width;
            occupancy_map->data[index] = 0;
        }
    }

    // Create a publisher for the occupancy map
    auto occupancy_map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_test", rclcpp::QoS(10).transient_local().reliable());

    // Publish the map periodically
    rclcpp::WallRate loop_rate(15); // 1 Hz
    while (rclcpp::ok()) {
        occupancy_map->header.stamp = node->now();
        occupancy_map_publisher->publish(*occupancy_map);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}