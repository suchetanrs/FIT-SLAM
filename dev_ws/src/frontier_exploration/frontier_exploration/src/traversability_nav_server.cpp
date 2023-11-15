#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>


namespace frontier_exploration{

class TraversabilityNavServer : public rclcpp::Node {
public:

    TraversabilityNavServer() : Node("traversability_nav_server_node")
    {
        temp_traversability_nav_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("traversability_nav_costmap",std::string{get_namespace()},"traversability_nav_costmap");
        temp_traversability_nav_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        temp_traversability_nav_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(temp_traversability_nav_costmap_ros_);
        temp_traversability_nav_costmap_ros_->activate();

    }

    ~TraversabilityNavServer()
    {
        temp_traversability_nav_costmap_ros_->deactivate();
        temp_traversability_nav_costmap_ros_->cleanup();
        temp_traversability_nav_costmap_thread_.reset();
    }

private:

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> temp_traversability_nav_costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> temp_traversability_nav_costmap_thread_;


};

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_exploration::TraversabilityNavServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}