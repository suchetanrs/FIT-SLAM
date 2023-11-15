#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "traversability_gridmap/traversabilityGrid.hpp"


class TraversabilityNode : public rclcpp::Node
{
public:
    TraversabilityNode()
            : Node("traversability_node")
    {
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
                ("/velodyne_traversability_points", sensor_qos, std::bind(&TraversabilityNode::pointcloud_callback, this, std::placeholders::_1));

        localizationSubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                        "/localization_pose", 10,
                        std::bind(&TraversabilityNode::localizationCallback, this, std::placeholders::_1));

        pubTraversability_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        	"/RTQuadtree_struct",  rclcpp::QoS(1).transient_local());

        pubImage_ =
        this->create_publisher<sensor_msgs::msg::Image>("RTQuadtree_image", rclcpp::QoS(1).transient_local());

        pubOccupancy_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("RTQuadtree_occupancyGrid", rclcpp::QoS(1).transient_local());


        this->declare_parameter("half_size", rclcpp::ParameterValue(40.0));
        this->get_parameter("half_size", half_size_);
        this->declare_parameter("resolution", rclcpp::ParameterValue(0.25));
        this->get_parameter("resolution", resolution_);

        this->declare_parameter("security_distance", rclcpp::ParameterValue(0.6));
        this->get_parameter("security_distance", security_distance_);
        this->declare_parameter("ground_clearance", rclcpp::ParameterValue(0.2));
        this->get_parameter("ground_clearance", ground_clearance_);
        this->declare_parameter("robot_height", rclcpp::ParameterValue(0.5));
        this->get_parameter("robot_height", robot_height_);        
        this->declare_parameter("max_slope", rclcpp::ParameterValue(0.4));
        this->get_parameter("max_slope", max_slope_);
        // this->declare_parameter("frame_id", rclcpp::ParameterValue("base_footprint"));
        // this->get_parameter("frame_id", frame_id_);

        this->declare_parameter("robot_width", rclcpp::ParameterValue(0.8));
        this->get_parameter("robot_width", robot_width_);
        this->declare_parameter("robot_length", rclcpp::ParameterValue(1.1));
        this->get_parameter("robot_length", robot_length_);
        this->declare_parameter("draw_isodistance_each", rclcpp::ParameterValue(1.));
        this->get_parameter("draw_isodistance_each", draw_isodistance_each_);

        //half_size_ = 7.5;
        //resolution_ = 0.25;
        traversabilityMap = std::make_shared<traversabilityGrid>(resolution_,
                                                                 Eigen::Vector2d(half_size_, half_size_));

    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

        point_cloud_ = point_cloud;
        std_msgs::msg::Header header_ = point_cloud->header;
        publishtraversabilityMap(header_);
        
    }


    void publishtraversabilityMap(std_msgs::msg::Header header) {

	// Fill traversability structure
        traversabilityMap->reset();
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*point_cloud_, "x"); it != it.end(); ++it) {
            Eigen::Vector3d pt3(it[0], it[1], it[2]);
            if( ((it[0] == 0) && (it[1] == 0)) || (it[2] > robot_height_) )
                continue;
            traversabilityMap->insert_data(pt3);
        }

        
        // Publish as grid map
        // Create grid map.
        grid_map::GridMap map({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation",
        "meanx", "meany", "meanz", "cov00", "cov01", "cov02", "cov10", "cov11", "cov12", "cov20", "cov21", "cov22"});
        map.setFrameId("velodyne_gravity");
        map.setGeometry(grid_map::Length(2.*half_size_, 2.*half_size_), resolution_);
        
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map.getPosition(*it, position);
            Eigen::VectorXd haz = traversabilityMap->get_goodness_m(
               Eigen::Vector2d(position.x(), position.y()), 
               security_distance_, ground_clearance_, max_slope_);
               
                if(haz(0) < 0.)
                    continue;

                map.at("hazard", *it) = haz(0);
                map.at("step_haz", *it) = haz(1);
                map.at("roughness_haz", *it) = haz(2);
                map.at("slope_haz", *it) = haz(3);
                map.at("border_haz", *it) = haz(4);
                map.at("elevation", *it) = haz(5);

                map.at("meanx", *it) = haz(6);
                map.at("meany", *it) = haz(7);
                map.at("meanz", *it) = haz(8);

                for (int i = 0; i < 9; ++i) {
                    map.at("cov" + std::to_string(i / 3) + std::to_string(i % 3), *it) = haz(9 + i);
                }
                RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_node"), "\n [" << haz(9) << ", " << haz(10) << ", " << haz(11) << "] \n"
                                                                           << "[" << haz(12) << ", " << haz(13) << ", " << haz(14) << "] \n" 
                                                                           << "[" << haz(15) << ", " << haz(16) << ", " << haz(17) << "] \n \n \n");                                                                       
        }
        
        localGrids_.push_back(map);
        auto message = grid_map::GridMapRosConverter::toMessage(map);
        pubTraversability_->publish(std::move(message));


        // convert grid map to CV image.
        cv::Mat originalImage, destImage;
        bool useTransparency = false;
        if (useTransparency) {
            // Note: The template parameters have to be set based on your encoding
            // of the image. For 8-bit images use `unsigned char`.
            grid_map::GridMapCvConverter::toImage<unsigned char, 3>(
            map, "hazard", CV_8UC3, 0.0, 1.0, originalImage);
        } else {
            // TODO: 1 and 0 should be interchanged.
            grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
            map, "hazard", CV_8UC1, 0.0, 1.0, originalImage);
        }

        // Transform image for visualization
        double scaling = 2;
        cv::resize(originalImage, originalImage, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
        cv::applyColorMap(originalImage, originalImage, cv::COLORMAP_JET);

        // Draw robot footprint
        cv::Point2i center = originalImage.size()/2;
        cv::Point2i half_rect_size = cv::Point2i(ceil(scaling*0.5*robot_width_/resolution_), ceil(scaling*0.5*robot_length_/resolution_));
        cv::rectangle(originalImage, center-half_rect_size, center+half_rect_size, cv::Scalar(0,0,255), 1);

        // Draw isodistances
        if(draw_isodistance_each_ > 0){
            uint N = half_size_*draw_isodistance_each_;
            for(uint i =1; i <N; ++i)
                cv::circle(originalImage, center, i*scaling*draw_isodistance_each_/resolution_, cv::Scalar(0,0,0) );
        }

        auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", originalImage)
               .toImageMsg();
        pubImage_->publish(*msg_.get());


        nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
        grid_map::GridMapRosConverter::toOccupancyGrid(map, "hazard", 0., 1., occupancyGrid_msg);
        pubOccupancy_->publish(occupancyGrid_msg);
    }

    void localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped msg){
        localizationPosesWithCov_.push_back(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localizationSubscription_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pubTraversability_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancy_;

    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;
    std::shared_ptr<traversabilityGrid> traversabilityMap;
    std::vector<grid_map::GridMap> localGrids_;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> localizationPosesWithCov_;
    double half_size_ ;
    double resolution_;
    double security_distance_;
    double ground_clearance_;
    double max_slope_;
    double robot_height_;
    double robot_length_;
    double robot_width_;
    double draw_isodistance_each_;
    // std::string frame_id_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
