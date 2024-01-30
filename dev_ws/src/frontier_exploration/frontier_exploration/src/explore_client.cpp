#include <frontier_exploration/geometry_tools.hpp>
#include <frontier_exploration/explore_client.hpp>

namespace frontier_exploration
{

    FrontierExplorationClient::FrontierExplorationClient() : Node("explore_client"), waiting_for_center_(false)
    {
        input_.header.frame_id = "map";

        point_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&FrontierExplorationClient::pointCb, this, std::placeholders::_1));
        point_viz_pub_ = create_publisher<visualization_msgs::msg::Marker>("exploration_polygon_marker", 10);
        point_viz_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&FrontierExplorationClient::vizPubCb, this));

        RCLCPP_INFO(rclcpp::get_logger("explore_client"),
                    "Please use the 'Point' tool in Rviz to select an exporation boundary.");

        this->declare_parameter("costmap_resolution", 0.05);
        this->get_parameter("costmap_resolution", costmap_resolution_);

        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
                &FrontierExplorationClient::dynamicParametersCallback,
                this, std::placeholders::_1));
    }

    void FrontierExplorationClient::vizPubCb()
    {

        visualization_msgs::msg::Marker points, line_strip;

        points.header = line_strip.header = input_.header;
        points.ns = line_strip.ns = "explore_points";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

        if (!input_.polygon.points.empty())
        {

            points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = costmap_resolution_;

            for (const auto &point : input_.polygon.points)
            {
                line_strip.points.push_back(nav2_costmap_2d::toPoint(point));
                points.points.push_back(nav2_costmap_2d::toPoint(point));
            }

            if (waiting_for_center_)
            {
                line_strip.points.push_back(nav2_costmap_2d::toPoint(input_.polygon.points.front()));
                points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            }
            else
            {
                points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
            }
        }
        else
        {
            points.action = line_strip.action = visualization_msgs::msg::Marker::DELETE;
        }
        point_viz_pub_->publish(points);
        point_viz_pub_->publish(line_strip);
    }

    void FrontierExplorationClient::pointCb(const std::shared_ptr<const geometry_msgs::msg::PointStamped> point)
    {
        RCLCPP_INFO(rclcpp::get_logger("explore_client"), "Point clicked");

        double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

        if (waiting_for_center_)
        {
            // flag is set so this is the last point of boundary polygon, i.e. center

            if (!pointInPolygon(point->point, input_.polygon))
            {
                RCLCPP_ERROR(rclcpp::get_logger("explore_client"), "Center not inside polygon, restarting");
            }
            else
            {
                auto exploreClient = rclcpp_action::create_client<frontier_msgs::action::ExploreTask>(this, "explore_action");
                frontier_msgs::action::ExploreTask::Goal goal;
                goal.explore_center = *point;
                goal.explore_boundary = input_;
                exploreClient->async_send_goal(goal);
            }
            waiting_for_center_ = false;
            input_.polygon.points.clear();
        }
        else if (input_.polygon.points.empty())
        {
            // first control point, so initialize header of boundary polygon

            input_.header = point->header;
            input_.polygon.points.push_back(nav2_costmap_2d::toPoint32(point->point));
        }
        else if (input_.header.frame_id != point->header.frame_id)
        {
            RCLCPP_ERROR(rclcpp::get_logger("explore_client"), "Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();
        }
        else if (input_.polygon.points.size() > 1 && pointsNearby(input_.polygon.points.front(), point->point,
                                                                  average_distance * 0.1))
        {
            // check if last boundary point, i.e. nearby to first point

            if (input_.polygon.points.size() < 3)
            {
                RCLCPP_ERROR(rclcpp::get_logger("explore_client"), "Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }
            else
            {
                waiting_for_center_ = true;
                RCLCPP_WARN(rclcpp::get_logger("explore_client"), "Please select an initial point for exploration inside the polygon");
            }
        }
        else
        {
            // otherwise, must be a regular point inside boundary polygon
            input_.polygon.points.push_back(nav2_costmap_2d::toPoint32(point->point));
            input_.header.stamp = rclcpp::Clock().now();
            RCLCPP_INFO(rclcpp::get_logger("explore_client"), "Point taken into account.");
        }
    }

    rcl_interfaces::msg::SetParametersResult FrontierExplorationClient::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters)
        {
            const auto &param_type = parameter.get_type();
            const auto &param_name = parameter.get_name();

            if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param_name == "costmap_resolution")
                {
                    costmap_resolution_ = parameter.as_double();
                }
            }
        }

        result.successful = true;
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_exploration::FrontierExplorationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}