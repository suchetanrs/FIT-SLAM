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

        RCLCPP_INFO(this->get_logger(),
                    "Please use the 'Point' tool in Rviz to select an exporation boundary.");

        this->declare_parameter("costmap_resolution", rclcpp::ParameterValue(0.05));
        this->get_parameter("costmap_resolution", costmap_resolution_);

        this->declare_parameter("use_config", rclcpp::ParameterValue(true));
        this->get_parameter("use_config", use_config_);

        config_ = {"10.0", "10.0", "10.0", "-10.0", "-10.0", "-10.0", "-10.0", "10.0"};
        this->declare_parameter("config", rclcpp::ParameterValue(config_));
        this->get_parameter("config", config_);

        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
                &FrontierExplorationClient::dynamicParametersCallback,
                this, std::placeholders::_1));

        exploreClient_ = rclcpp_action::create_client<frontier_msgs::action::ExploreTask>(this, "explore_action");
        if (!exploreClient_->wait_for_action_server(std::chrono::seconds(25)))
        {
            RCLCPP_ERROR(this->get_logger(), "Explore action server not available after waiting for 25 seconds.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Found explore action server");
        }

        if (use_config_)
        {
            // if use_config is true, directly send the parameter values to the action server.
            sendActionFromConfig();
            point_.reset();
            point_viz_timer_.reset();
            point_viz_pub_.reset();
        }
    }

    FrontierExplorationClient::~FrontierExplorationClient()
    {
        point_.reset();
        point_viz_timer_.reset();
        point_viz_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "Shutting down explore client.");
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
        RCLCPP_INFO(this->get_logger(), "Point clicked");

        double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

        if (waiting_for_center_)
        {
            // flag is set so this is the last point of boundary polygon, i.e. center

            if (!pointInPolygon(point->point, input_.polygon))
            {
                RCLCPP_ERROR(this->get_logger(), "Center not inside polygon, restarting");
            }
            else
            {
                frontier_msgs::action::ExploreTask::Goal goal;
                goal.explore_center = *point;
                goal.explore_boundary = input_;
                exploreClient_->async_send_goal(goal);
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
            RCLCPP_ERROR(this->get_logger(), "Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();
        }
        else if (input_.polygon.points.size() > 1 && pointsNearby(input_.polygon.points.front(), point->point,
                                                                  average_distance * 0.1))
        {
            // check if last boundary point, i.e. nearby to first point

            if (input_.polygon.points.size() < 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }
            else
            {
                waiting_for_center_ = true;
                RCLCPP_WARN(this->get_logger(), "Please select an initial point for exploration inside the polygon");
            }
        }
        else
        {
            // otherwise, must be a regular point inside boundary polygon
            input_.polygon.points.push_back(nav2_costmap_2d::toPoint32(point->point));
            input_.header.stamp = rclcpp::Clock().now();
            RCLCPP_INFO(this->get_logger(), "Point taken into account.");
        }
    }

    void FrontierExplorationClient::sendActionFromConfig()
    {
        input_.header.stamp = rclcpp::Clock().now();
        frontier_msgs::action::ExploreTask::Goal goal;
        for (int i = 0; i < config_.size(); i += 2)
        {
            geometry_msgs::msg::Point32 point;
            point.x = std::stof(config_[i]);
            point.y = std::stof(config_[i + 1]);
            input_.polygon.points.push_back(point);
        }
        goal.explore_boundary = input_;
        goal.explore_boundary.header.frame_id = "map";
        goal.explore_center.header.frame_id = "map";
        goal.explore_center.point.x = 5.5;
        goal.explore_center.point.y = 5.5;
        exploreClient_->async_send_goal(goal);
        for (const auto &point : goal.explore_boundary.polygon.points)
        {
            RCLCPP_INFO(this->get_logger(), "Sending Polygon from config x: %f, y: %f, z: %f",
                        point.x, point.y, point.z);
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
    node.reset();
    rclcpp::shutdown();
}