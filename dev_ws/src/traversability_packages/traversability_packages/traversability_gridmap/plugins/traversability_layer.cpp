#include <traversability_gridmap/traversability_layer.hpp>
#include <chrono>
#include <rclcpp/parameter.hpp>

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace traversability_gridmap
{
    using rcl_interfaces::msg::ParameterType;

    TraversabilityLayer::TraversabilityLayer() {}

    TraversabilityLayer::~TraversabilityLayer()
    {
        dyn_params_handler_.reset();
    }

    void TraversabilityLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node traversability_layer"};
        }
        subscription_ = node->create_subscription<traversability_msgs::msg::PCL2WithNodeID>("velodyne_transformed_points_nodeid", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&TraversabilityLayer::pointcloud_callback, this, std::placeholders::_1));

        pubTraversability_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
            "RTQuadtree_struct", rclcpp::QoS(1).transient_local());
        pubTraversability_->on_activate();

        pubImage_ =
            node->create_publisher<sensor_msgs::msg::Image>("RTQuadtree_image", rclcpp::QoS(1).transient_local());
        pubImage_->on_activate();

        pubOccupancy_ =
            node->create_publisher<nav_msgs::msg::OccupancyGrid>("RTQuadtree_occupancyGrid", rclcpp::QoS(1).transient_local());
        pubOccupancy_->on_activate();

        planeLMSPub_ =
            node->create_publisher<geometry_msgs::msg::PoseArray>("surface_normals", 1);
        planeLMSPub_->on_activate();

        resolution_ = layered_costmap_->getCostmap()->getResolution();
        
        declareParameter("square_half_side_length", rclcpp::ParameterValue(10.0));
        node->get_parameter(name_ + "." + "square_half_side_length", squareHalfSideLength_);

        declareParameter("half_size", rclcpp::ParameterValue(7.0));
        node->get_parameter(name_ + "." + "half_size", half_size_);

        declareParameter("security_distance", rclcpp::ParameterValue(0.6));
        node->get_parameter(name_ + "." + "security_distance", security_distance_);
        
        declareParameter("ground_clearance", rclcpp::ParameterValue(0.2));
        node->get_parameter(name_ + "." + "ground_clearance", ground_clearance_);
        
        declareParameter("robot_height", rclcpp::ParameterValue(0.5));
        node->get_parameter(name_ + "." + "robot_height", robot_height_);
        
        declareParameter("max_slope", rclcpp::ParameterValue(0.2));
        node->get_parameter(name_ + "." + "max_slope", max_slope_);

        declareParameter("robot_width", rclcpp::ParameterValue(0.8));
        node->get_parameter(name_ + "." + "robot_width", robot_width_);
        
        declareParameter("robot_length", rclcpp::ParameterValue(1.1));
        node->get_parameter(name_ + "." + "robot_length", robot_length_);
        
        declareParameter("draw_isodistance_each", rclcpp::ParameterValue(1.0));
        node->get_parameter(name_ + "." + "draw_isodistance_each", draw_isodistance_each_);

        declareParameter("enabled", rclcpp::ParameterValue(false));
        node->get_parameter(name_ + "." + "enabled", enabledLayer_);

        point_cloud_ = nullptr;
        traversabilityMap = std::make_shared<traversabilityGrid>(resolution_, Eigen::Vector2d(half_size_, half_size_), planeLMSPub_);
        globalTraversabilityMap_ = std::make_shared<GlobalTraversabilityMap>();
        // Change this if you change in polygon point publisher
        layered_costmap_->resizeMap((2 * squareHalfSideLength_) / resolution_, (2 * squareHalfSideLength_) / resolution_, layered_costmap_->getCostmap()->getResolution(), -squareHalfSideLength_, -squareHalfSideLength_);
        Costmap2D *master = layered_costmap_->getCostmap();
        
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                    master->getOriginX(), master->getOriginY());

        dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
            &TraversabilityLayer::dynamicParametersCallback,
            this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult TraversabilityLayer::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        rcl_interfaces::msg::SetParametersResult result;

        for (const rclcpp::Parameter& parameter : parameters) {
            const std::string& param_name = parameter.get_name();
            const rclcpp::ParameterType param_type = parameter.get_type();

            if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param_name == "square_half_side_length") {
                    squareHalfSideLength_ = parameter.as_double();
                } else if (param_name == "half_size") {
                    half_size_ = parameter.as_double();
                } else if (param_name == "security_distance") {
                    security_distance_ = parameter.as_double();
                } else if (param_name == "ground_clearance") {
                    ground_clearance_ = parameter.as_double();
                } else if (param_name == "robot_height") {
                    robot_height_ = parameter.as_double();
                } else if (param_name == "max_slope") {
                    max_slope_ = parameter.as_double();
                } else if (param_name == "robot_width") {
                    robot_width_ = parameter.as_double();
                } else if (param_name == "robot_length") {
                    robot_length_ = parameter.as_double();
                } else if (param_name == "draw_isodistance_each") {
                    draw_isodistance_each_ = parameter.as_double();
                }
            }
            if (param_type == rclcpp::ParameterType::PARAMETER_BOOL) {
                if (param_name == "enabled") {
                    enabledLayer_ = parameter.as_bool();
                }
            }
        }
        
        result.successful = true;
        return result;
    }


    sensor_msgs::msg::PointCloud2 TraversabilityLayer::transformPCL(sensor_msgs::msg::PointCloud2 msg, geometry_msgs::msg::TransformStamped transform)
    {
        // Transform the PCL with transform
        pcl::PCLPointCloud2 msg_cloud;
        // Convert message to pcl : out = msg_cloud
        pcl_conversions::toPCL(msg, msg_cloud);

        // Convert msg_cloud to pcl::PointCloud<T> : out = pcl_cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromPCLPointCloud2(msg_cloud, pcl_cloud);

        // Transform the pcl_cloud out: transfomed_cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl_ros::transformPointCloud(pcl_cloud, transformed_cloud, transform);

        // Convert tranformed_cloud to ros msg.
        sensor_msgs::msg::PointCloud2 transformed_msg;
        pcl::toROSMsg(transformed_cloud, transformed_msg);

        return transformed_msg;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr TraversabilityLayer::TransformPCLforAddition(geometry_msgs::msg::Pose pose_selected, sensor_msgs::msg::PointCloud2& pcl_ptr) 
    {
        geometry_msgs::msg::Vector3 translation;
        double ztranslation;

        // Calculate rotation (orientation) as a quaternion
        tf2::Quaternion rotation;
        geometry_msgs::msg::TransformStamped transform_stamped_tf;

        translation.x = std::round(pose_selected.position.x / resolution_) * resolution_;
        translation.y = std::round(pose_selected.position.y / resolution_) * resolution_;
        translation.z = 0.0;
        ztranslation = pose_selected.position.z;

        transform_stamped_tf.transform.translation = translation;

        // Publish transform to tree.
        transform_stamped_tf.header.frame_id = "map";
        transform_stamped_tf.header.stamp = pcl_ptr.header.stamp;
        transform_stamped_tf.child_frame_id = "posegraph_map";
        auto transform_stamped_tf_eig = traversabilityMap->computeTransformationMatrix(transform_stamped_tf.transform);
        transform_stamped_tf_eig = transform_stamped_tf_eig.inverse();

        sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg2;
        transformed_msg2 = nullptr;

        auto transform = traversabilityMap->computeGeometryMsgsTransform(transform_stamped_tf_eig);
        transform.transform.translation.z = -ztranslation;
        transform.header.frame_id = "posegraph_map";
        transform.child_frame_id = "map";
        transform.header.stamp = pcl_ptr.header.stamp;
        transformed_msg2 = std::make_shared<sensor_msgs::msg::PointCloud2>(transformPCL(pcl_ptr, transform));
        transformed_msg2->header.stamp = pcl_ptr.header.stamp;
        transformed_msg2->header.frame_id = "posegraph_map";

        return transformed_msg2;
    }

    void TraversabilityLayer::pointcloud_callback(const traversability_msgs::msg::PCL2WithNodeID::SharedPtr point_cloud_with_id)
    {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_layer"), "Callback!");
        auto start_time = std::chrono::high_resolution_clock::now();
        auto map_storage = globalTraversabilityMap_->getGridMapsWithIDPCL();
        for (size_t i = 0; i < point_cloud_with_id->graph.poses_id.size(); ++i)
        {
            auto it = map_storage.find(point_cloud_with_id->graph.poses_id[i]);
            if (it != map_storage.end()) {
                if(isPoseChanged(point_cloud_with_id->graph.poses[i], it->second->pose, 0.0)) {
                  // RCLCPP_ERROR(rclcpp::get_logger("traversability_layer"), "Pose changed");
                    auto transformed_pcl = TransformPCLforAddition(point_cloud_with_id->graph.poses[i], it->second->pcl);
                    if(transformed_pcl) {
                        TraversabilityLayer::publishtraversabilityMap(point_cloud_with_id->graph.poses_id[i], it->second->pcl, transformed_pcl, point_cloud_with_id->graph.poses[i]);
                    }
                }
            }
            else {
                geometry_msgs::msg::Pose pose_selected = point_cloud_with_id->pose;
                // RCLCPP_INFO_STREAM(rclcpp::get_logger("Traversability layer"), "Got latest pose of size: " << point_cloud_with_id->graph.poses.size());
                auto transformed_msg2 = TransformPCLforAddition(pose_selected, point_cloud_with_id->pcl);
                if(transformed_msg2) {
                    // RCLCPP_INFO_STREAM(rclcpp::get_logger("global_traversability"), "NODE ID OF CURRENT PCL: " << point_cloud_with_id->node_id);
                    TraversabilityLayer::publishtraversabilityMap(point_cloud_with_id->node_id, point_cloud_with_id->pcl, transformed_msg2, pose_selected);
                }
                else
                    RCLCPP_ERROR(rclcpp::get_logger("global_traversability"), "Transformed_msg2 is still Nullptr!");
                map_storage = globalTraversabilityMap_->getGridMapsWithIDPCL();
            }
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        double elapsed_seconds_decimal = elapsed_seconds.count();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Traversability layer"), "Time: " << elapsed_seconds_decimal);
    }

    void TraversabilityLayer::publishtraversabilityMap(int nodeid, sensor_msgs::msg::PointCloud2& pcl_map, std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_, geometry_msgs::msg::Pose pose_selected)
    {
        // Fill traversability structure
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_layer"), "Node ID of current pointcloud" << nodeid);
        traversabilityMap->reset();
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*pcl_, "x"); it != it.end(); ++it)
        {
            Eigen::Vector3d pt3(it[0], it[1], it[2]);
            if (((it[0] == 0) && (it[1] == 0)) || (it[2] > robot_height_))
            continue;
            traversabilityMap->insert_data(pt3);
        }

        // Publish as grid map
        // Create grid map.
        grid_map::GridMap map({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(2. * half_size_, 2. * half_size_), resolution_);
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
        {
            grid_map::Position position;
            map.getPosition(*it, position);
            Eigen::VectorXd haz = traversabilityMap->get_goodness_m(
                Eigen::Vector2d(position.x(), position.y()),
                security_distance_, ground_clearance_, max_slope_);

            if (haz(0) < 0.)
            continue;

            map.at("hazard", *it) = haz(0);
            map.at("step_haz", *it) = haz(1);
            map.at("roughness_haz", *it) = haz(2);
            map.at("slope_haz", *it) = haz(3);
            map.at("border_haz", *it) = haz(4);
            map.at("elevation", *it) = haz(5);
        }

        auto message = grid_map::GridMapRosConverter::toMessage(map);
        pubTraversability_->publish(std::move(message));

        // // convert grid map to CV image.
        // cv::Mat originalImage, destImage;
        // bool useTransparency = false;
        // if (useTransparency)
        // {
        //     // Note: The template parameters have to be set based on your encoding
        //     // of the image. For 8-bit images use `unsigned char`.
        //     grid_map::GridMapCvConverter::toImage<unsigned char, 3>(
        //         map, "hazard", CV_8UC3, 0.0, 1.0, originalImage);
        // }
        // else
        // {
        //     // TODO: 1 and 0 should be interchanged.
        //     grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
        //         map, "hazard", CV_8UC1, 0.0, 1.0, originalImage);
        // }

        // // Transform image for visualization
        // double scaling = 2;
        // cv::resize(originalImage, originalImage, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
        // cv::applyColorMap(originalImage, originalImage, cv::COLORMAP_JET);

        // // Draw robot footprint
        // cv::Point2i center = originalImage.size() / 2;
        // cv::Point2i half_rect_size = cv::Point2i(ceil(scaling * 0.5 * robot_width_ / resolution_), ceil(scaling * 0.5 * robot_length_ / resolution_));
        // cv::rectangle(originalImage, center - half_rect_size, center + half_rect_size, cv::Scalar(0, 0, 255), 1);

        // // Draw isodistances
        // if (draw_isodistance_each_ > 0)
        // {
        //     uint N = half_size_ * draw_isodistance_each_;
        //     for (uint i = 1; i < N; ++i)
        //     cv::circle(originalImage, center, i * scaling * draw_isodistance_each_ / resolution_, cv::Scalar(0, 0, 0));
        // }

        // auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", originalImage)
        //                 .toImageMsg();
        // pubImage_->publish(*msg_.get());

        // nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
        // grid_map::GridMapRosConverter::toOccupancyGrid(map, "hazard", 0., 1., occupancyGrid_msg);
        // pubOccupancy_->publish(occupancyGrid_msg);

        globalTraversabilityMap_->insertGridMapNodeIDPCL(map, nodeid, pcl_map, pose_selected);
    }

  void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
  {
    *min_x = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("traversability_layer"),
        "updateBounds: robot_x = " << robot_x << ", robot_y = " << robot_y << ", robot_yaw = " << robot_yaw << ", min_x = " << *min_x << ", min_y = " << *min_y << ", max_x = " << *max_x << ", max_y = " << *max_y << "Resolution = " << resolution_;);
  }

  void TraversabilityLayer::updateMasterCostmap(nav2_costmap_2d::Costmap2D &master_grid, double robot_pose_x, double robot_pose_y, std::map<std::string, double> valuesatposition, float positionx, float positiony)
  {
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int mx, my;
    if(master_grid.worldToMap(robot_pose_x + positionx + (resolution_ / 2), robot_pose_y + positiony - (resolution_ / 2), mx, my)) {
        int index = master_grid.getIndex(mx, my);
        std::vector<double> coord_values = {robot_pose_x + positionx + (resolution_ / 2), robot_pose_y + positiony - (resolution_ / 2)};
        // TODO:This wont work as the costmap is reset everytime. It will always show 255.
        if(static_cast<int>(master_array[index]) == 255) {
            master_array[index] = static_cast<unsigned char>(valuesatposition["hazard"] * 254);
            globalProperties_[coord_values] = valuesatposition;
        }
        if(static_cast<int>(master_array[index]) <= 254) {
            // std::map<std::string, double> resultkalman = KalmanFilter(globalProperties_[coord_values], valuesatposition);
            // master_array[index] = resultkalman["hazard"];
            master_array[index] = static_cast<unsigned char>(valuesatposition["hazard"] * 254);
            globalProperties_[coord_values] = valuesatposition;
        }
    }
  }


  void TraversabilityLayer::updateLayer(nav2_costmap_2d::Costmap2D &master_grid, double robot_pose_x, double robot_pose_y, std::map<std::string, double> valuesatposition, float positionx, float positiony)
  {
    // unsigned char *master_array = master_grid.getCharMap();
    unsigned int mx, my;
    if(master_grid.worldToMap(robot_pose_x + positionx + (resolution_ / 2), robot_pose_y + positiony - (resolution_ / 2), mx, my)) {
        int index = master_grid.getIndex(mx, my);
        std::vector<double> coord_values = {robot_pose_x + positionx + (resolution_ / 2), robot_pose_y + positiony - (resolution_ / 2)};
        // TODO:This wont work as the costmap is reset everytime. It will always show 255.
        if(static_cast<int>(costmap_[index]) == 255) {
            costmap_[index] = static_cast<unsigned char>(valuesatposition["hazard"] * 254);
            globalProperties_[coord_values] = valuesatposition;
        }
        if(static_cast<int>(costmap_[index]) <= 254) {
            // std::map<std::string, double> resultkalman = KalmanFilter(globalProperties_[coord_values], valuesatposition);
            // master_array[index] = resultkalman["hazard"];
            costmap_[index] = static_cast<unsigned char>(valuesatposition["hazard"] * 254);
            globalProperties_[coord_values] = valuesatposition;
        }
    }
  }


  void TraversabilityLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  { 
    auto start_time = std::chrono::high_resolution_clock::now();
    auto map_storage = globalTraversabilityMap_->getGridMapsWithIDPCL();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    double elapsed_seconds_decimal = elapsed_seconds.count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Traversability layer"), "Time retrieve data is: " << elapsed_seconds_decimal);
    if (map_storage.size() > 0)
    {
      for (auto it = map_storage.begin(); it != map_storage.end(); ++it)
      {
        auto grid_map_selected = it->second->gridMsg;
        auto pose_selected = it->second->pose;
        for (grid_map::GridMapIterator it2(grid_map_selected); !it2.isPastEnd(); ++it2)
        {
          grid_map::Position position;
          grid_map_selected.getPosition(*it2, position);
          std::map<std::string, double> valuesatposition;
        //   for (const std::string& layer : grid_map_selected.getLayers()) {
        //     valuesatposition[layer] = grid_map_selected.atPosition(layer, position);
        //   }
          valuesatposition["hazard"] = grid_map_selected.atPosition("hazard", position);
          if (std::isnan(valuesatposition["hazard"]) == false) {
            if(enabledLayer_ == true) {
                updateMasterCostmap(master_grid, pose_selected.position.x, pose_selected.position.y, valuesatposition, position.x(), position.y());
            }
            else {
                updateLayer(master_grid, pose_selected.position.x, pose_selected.position.y, valuesatposition, position.x(), position.y());
            }
          }
        }
      }
    }
    RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("traversability_layer"),
        "updateCosts: "
            << ", min_i = " << min_i << ", min_j = " << min_j << ", max_i = " << max_i << ", max_j = " << max_j << "Resolution = " << resolution_;);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end_time - start_time;
    elapsed_seconds_decimal = elapsed_seconds.count();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Traversability layer"), "Time taken to update global map is: " << elapsed_seconds_decimal);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(traversability_gridmap::TraversabilityLayer, nav2_costmap_2d::Layer)