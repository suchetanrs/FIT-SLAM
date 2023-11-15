#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>


namespace frontier_exploration{

class TraversabilityServer : public rclcpp::Node {
public:

    TraversabilityServer() : Node("traversability_server_node")
    {
        traversability_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("traversability_costmap",std::string{get_namespace()},"traversability_costmap");
        traversability_costmap_ros_->configure();
        // Launch a thread to run the costmap node
        traversability_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(traversability_costmap_ros_);
        traversability_costmap_ros_->activate();

    }

    ~TraversabilityServer()
    {
        traversability_costmap_ros_->deactivate();
        traversability_costmap_ros_->cleanup();
        traversability_costmap_thread_.reset();
    }

private:

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> traversability_costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> traversability_costmap_thread_;


};

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_exploration::TraversabilityServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}