#ifndef EXPLORE_CLIENT_HPP_
#define EXPLORE_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <frontier_msgs/action/explore_task.hpp>
#include <frontier_msgs/srv/get_next_frontier.hpp>
#include <frontier_msgs/srv/update_boundary_polygon.hpp>

#include <action_msgs/msg/goal_status.hpp>

#include <visualization_msgs/msg/marker.hpp>

namespace frontier_exploration
{

    /**
     * @brief Client for FrontierExplorationServer that receives
     * control points from rviz, and creates boundary polygon for frontier exploration
     */
    class FrontierExplorationClient : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the client.
         */
        FrontierExplorationClient();

        /**
         * @brief The callback for dynamically changing the set parameters during runtime.
         */
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
            std::vector<rclcpp::Parameter> parameters);

    private:
        /**
         * @brief Publish markers for visualization of points for boundary polygon.
         */
        void vizPubCb();

        /**
         * @brief Build boundary polygon from points received through rviz gui.
         * @param point Received point from rviz
         */
        void pointCb(const std::shared_ptr<const geometry_msgs::msg::PointStamped> point);

        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_viz_pub_;
        rclcpp::TimerBase::SharedPtr point_viz_timer_;
        geometry_msgs::msg::PolygonStamped input_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        std::shared_ptr<rclcpp_action::Client<frontier_msgs::action::ExploreTask>> exploreClient_;
        bool waiting_for_center_;
        double costmap_resolution_;
    };

}
#endif