#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <fstream>

class MapDataCovarianceNode : public rclcpp::Node {
public:
    MapDataCovarianceNode() : Node("localization_pose_sub_node") {
        // Initialize the subscriber
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "localization_pose", 10, std::bind(&MapDataCovarianceNode::poseCallback, this, std::placeholders::_1));

        // Initialize CSV file

        // Initialize node start time
        start_time_ = this->get_clock()->now();
        this->declare_parameter("exploration_mode", "random");
        this->get_parameter("exploration_mode", mode_);

        this->declare_parameter("counter", 1);
        this->get_parameter("counter", counter_);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Calculate time elapsed since node start
        rclcpp::Time current_time = this->get_clock()->now();
        double time_elapsed = current_time.seconds();

        // Calculate the trace of the covariance matrix
        double trace = 0.0;
        for (int i = 0; i < 36; i += 7) {
            trace += msg->pose.covariance[i];
        }
        double wx = msg->pose.pose.position.x;
        double wy = msg->pose.pose.position.y;

        // Write data to the CSV file
        std::ofstream csv_file_1_;
        csv_file_1_ << std::fixed;
        csv_file_1_ << std::setprecision(2);
        csv_file_1_.open(std::to_string(counter_) + "_" + mode_ + "_loc_covariance_" + ".csv", std::ios::app);
        csv_file_1_ << time_elapsed << "," << trace << "\n";
        csv_file_1_.close();
        std::ofstream csv_file_2_;
        csv_file_2_ << std::fixed;
        csv_file_2_ << std::setprecision(2);
        csv_file_2_.open(std::to_string(counter_) + "_" + mode_ + "_loc_xy_" + ".csv", std::ios::app);
        std::to_string(start_time_.seconds());
        csv_file_2_ << time_elapsed << "," << wx << "," << wy << "\n";
        csv_file_2_.close();
        RCLCPP_INFO(this->get_logger(), "Hello!");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Time start_time_;
    std::string mode_;
    int counter_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapDataCovarianceNode>());
    rclcpp::shutdown();
    return 0;
}
