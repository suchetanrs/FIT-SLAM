#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer() : Node("path_visualizer"), marker_id_(0), point_count_(0)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/state", 10, std::bind(&PathVisualizer::odom_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PathVisualizer::timer_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_lock);
        latest_odometry_msg_ = msg;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(odom_lock);
        geometry_msgs::msg::Point point;
        point.x = latest_odometry_msg_->pose.pose.position.x;
        point.y = latest_odometry_msg_->pose.pose.position.y;
        point.z = latest_odometry_msg_->pose.pose.position.z;

        points_.push_back(point);
        colors_.push_back(compute_color(point_count_));

        point_count_++;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line_strip;

        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "path";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = marker_id_++;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.scale.x = 0.25; // Line width

        for (size_t i = 0; i < points_.size(); ++i)
        {
            line_strip.points.push_back(points_[i]);
            line_strip.colors.push_back(compute_color(i));
        }

        marker_array.markers.push_back(line_strip);
        marker_pub_->publish(marker_array);
    }

    std_msgs::msg::ColorRGBA compute_color(size_t index)
    {
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0; // Fully opaque

        // Hue calculation: map the index to a value between 0 and 360 (degrees)
        float hue = static_cast<float>(index) / points_.size() * 360.0f;

        // Convert HSV to RGB
        float c = 1.0f; // Chroma is full (since saturation is 1)
        float fHPrime = fmod(hue / 60.0, 6);
        float x = (1 - fabs(fmod(fHPrime, 2) - 1));
        float m = 0.0f; // Value is 1, so m = 0 in RGB

        float r_prime = 0.0f, g_prime = 0.0f, b_prime = 0.0f;
        if (0 <= hue && hue < 60)
        {
            r_prime = c;
            g_prime = x;
            b_prime = 0;
        }
        else if (60 <= hue && hue < 120)
        {
            r_prime = x;
            g_prime = c;
            b_prime = 0;
        }
        else if (120 <= hue && hue < 180)
        {
            r_prime = 0;
            g_prime = c;
            b_prime = x;
        }
        else if (180 <= hue && hue < 240)
        {
            r_prime = 0;
            g_prime = x;
            b_prime = c;
        }
        else if (240 <= hue && hue < 300)
        {
            r_prime = x;
            g_prime = 0;
            b_prime = c;
        }
        else if (300 <= hue && hue < 360)
        {
            r_prime = c;
            g_prime = 0;
            b_prime = x;
        }

        color.r = r_prime + m;
        color.g = g_prime + m;
        color.b = b_prime + m;

        return color;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Point> points_;
    std::vector<std_msgs::msg::ColorRGBA> colors_;
    size_t marker_id_;
    size_t point_count_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_msg_;
    std::mutex odom_lock;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}