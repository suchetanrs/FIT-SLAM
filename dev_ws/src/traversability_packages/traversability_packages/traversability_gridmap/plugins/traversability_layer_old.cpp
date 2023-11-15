#include <traversability_gridmap/traversability_layer.hpp>

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace traversability_gridmap
{
  TraversabilityLayer::TraversabilityLayer()
  {
  }

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
    subscription_ = node->create_subscription<traversability_msgs::msg::PCL2WithNodeID>("/velodyne_transformed_points_nodeid", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&TraversabilityLayer::pointcloud_callback, this, std::placeholders::_1));

    // subscriptionPoseArray_ = node->create_subscription<rtabmap_msgs::msg::Path>
    //         ("/pose_graph", 10,
    //         std::bind(&TraversabilityLayer::poseArrayCallback, this, std::placeholders::_1));

    pubTraversability_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "/RTQuadtree_struct", rclcpp::QoS(1).transient_local());

    pubTraversability_->on_activate();

    pubImage_ =
        node->create_publisher<sensor_msgs::msg::Image>("RTQuadtree_image", rclcpp::QoS(1).transient_local());

    pubImage_->on_activate();

    pubOccupancy_ =
        node->create_publisher<nav_msgs::msg::OccupancyGrid>("RTQuadtree_occupancyGrid", rclcpp::QoS(1).transient_local());

    pubOccupancy_->on_activate();

    publisherpcl_ =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_transformed_points", rclcpp::QoS(1).transient_local());

    publisherpcl_->on_activate();


    node->declare_parameter("half_size", rclcpp::ParameterValue(7.0));
    node->get_parameter("half_size", half_size_);
    // node->declare_parameter("resolution", rclcpp::ParameterValue(0.25));
    node->get_parameter("resolution", resolution_);
    // resolution_ = 0.25;

    node->declare_parameter("security_distance", rclcpp::ParameterValue(0.6));
    node->get_parameter("security_distance", security_distance_);
    node->declare_parameter("ground_clearance", rclcpp::ParameterValue(0.2));
    node->get_parameter("ground_clearance", ground_clearance_);
    node->declare_parameter("robot_height", rclcpp::ParameterValue(0.5));
    node->get_parameter("robot_height", robot_height_);
    node->declare_parameter("max_slope", rclcpp::ParameterValue(0.4));
    node->get_parameter("max_slope", max_slope_);
    // node->declare_parameter("frame_id", rclcpp::ParameterValue("base_footprint"));
    // node->get_parameter("frame_id", frame_id_);

    node->declare_parameter("robot_width", rclcpp::ParameterValue(0.8));
    node->get_parameter("robot_width", robot_width_);
    node->declare_parameter("robot_length", rclcpp::ParameterValue(1.1));
    node->get_parameter("robot_length", robot_length_);
    node->declare_parameter("draw_isodistance_each", rclcpp::ParameterValue(1.));
    node->get_parameter("draw_isodistance_each", draw_isodistance_each_);

    node->declare_parameter("loop_closure_correction", rclcpp::ParameterValue(false));
    node->get_parameter("loop_closure_correction", loopClosureCorrection_);

    oldPoseArray = nullptr;
    point_cloud_ = nullptr;
    // half_size_ = 7.5;
    // resolution_ = 0.25;
    traversabilityMap = std::make_shared<traversabilityGrid>(resolution_,
                                                             Eigen::Vector2d(half_size_, half_size_));

    globalTraversabilityMap_ = std::make_shared<GlobalTraversabilityMap>();

    // Change this if you change in polygon point publisher
    layered_costmap_->resizeMap(40 / resolution_, 40 / resolution_, layered_costmap_->getCostmap()->getResolution(), -20, -20);
    Costmap2D *master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
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

  sensor_msgs::msg::PointCloud2::SharedPtr TraversabilityLayer::TransformPCLforAddition(geometry_msgs::msg::Pose pose_selected, sensor_msgs::msg::PointCloud2::SharedPtr pcl_ptr) {
    geometry_msgs::msg::Vector3 translation;
    double ztranslation;

    // Calculate rotation (orientation) as a quaternion
    double resolution = 0.25;
    tf2::Quaternion rotation;
    geometry_msgs::msg::TransformStamped transform_stamped_tf;

    translation.x = std::round(pose_selected.position.x / resolution) * resolution;
    translation.y = std::round(pose_selected.position.y / resolution) * resolution;
    translation.z = 0.0;
    ztranslation = pose_selected.position.z;

    transform_stamped_tf.transform.translation = translation;

    // Publish transform to tree.
    transform_stamped_tf.header.frame_id = "map";
    transform_stamped_tf.header.stamp = pcl_ptr->header.stamp;
    transform_stamped_tf.child_frame_id = "posegraph_map";
    auto transform_stamped_tf_eig = traversabilityMap->computeTransformationMatrix(transform_stamped_tf.transform);
    transform_stamped_tf_eig = transform_stamped_tf_eig.inverse();

    sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg2;
    transformed_msg2 = nullptr;

    auto transform = traversabilityMap->computeGeometryMsgsTransform(transform_stamped_tf_eig);
    transform.transform.translation.z = -ztranslation;
    transform.header.frame_id = "posegraph_map";
    transform.child_frame_id = "map";
    transform.header.stamp = pcl_ptr->header.stamp;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Traversability layer"), "Transform2 AVAILABLE");
    transformed_msg2 = std::make_shared<sensor_msgs::msg::PointCloud2>(transformPCL(*pcl_ptr, transform));
    transformed_msg2->header.stamp = pcl_ptr->header.stamp;
    transformed_msg2->header.frame_id = "posegraph_map";
    publisherpcl_->publish(*transformed_msg2);
    // auto tf_buffer2_ = std::make_shared<tf2_ros::Buffer>(buffer_node_->get_clock());
    // auto tf_listener2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer2_);
    // auto tf_broadcaster2_ = std::make_shared<tf2_ros::TransformBroadcaster>(buffer_node_);
    // tf_broadcaster2_->sendTransform(transform);
    return transformed_msg2;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr TraversabilityLayer::TransformPCLforCorrection(geometry_msgs::msg::Pose pose_selected, sensor_msgs::msg::PointCloud2::SharedPtr pcl_ptr, std_msgs::msg::Header headernow) {
    geometry_msgs::msg::Vector3 translation;
    double ztranslation;

    // Calculate rotation (orientation) as a quaternion
    double resolution = 0.25;
    tf2::Quaternion rotation;
    geometry_msgs::msg::TransformStamped transform_stamped_tf;

    translation.x = std::round(pose_selected.position.x / resolution) * resolution;
    translation.y = std::round(pose_selected.position.y / resolution) * resolution;
    translation.z = 0.0;
    ztranslation = pose_selected.position.z;

    // rotation.setX(pose_selected.orientation.x);
    // rotation.setY(pose_selected.orientation.y);
    // rotation.setZ(pose_selected.orientation.z);
    // rotation.setW(pose_selected.orientation.w);

    // transform_stamped_tf.transform.rotation.x = rotation.x();
    // transform_stamped_tf.transform.rotation.y = rotation.y();
    // transform_stamped_tf.transform.rotation.z = rotation.z();
    // transform_stamped_tf.transform.rotation.w = rotation.w();

    transform_stamped_tf.transform.translation = translation;

    // Publish transform to tree.
    transform_stamped_tf.header.frame_id = "map";
    transform_stamped_tf.header.stamp = headernow.stamp;
    transform_stamped_tf.child_frame_id = "posegraph_map_correction";
    auto tf_buffer2_ = std::make_shared<tf2_ros::Buffer>(buffer_node_->get_clock());
    auto tf_listener2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer2_);
    auto tf_broadcaster2_ = std::make_shared<tf2_ros::TransformBroadcaster>(buffer_node_);
    tf_broadcaster2_->sendTransform(transform_stamped_tf);

    sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg2;
    transformed_msg2 = nullptr;

    auto rate = rclcpp::Rate(20.0);
    while(!tf_buffer2_->canTransform("posegraph_map_correction", "map", tf2::TimePointZero))
    {
      rate.sleep();
      RCLCPP_ERROR(rclcpp::get_logger("Traversability layer"), "Transform2 unavailable");
    }
    auto transform = tf_buffer2_->lookupTransform("posegraph_map_correction", "map", tf2::TimePointZero);
    transform.transform.translation.z = -ztranslation;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Traversability layer"), "Transform2 AVAILABLE");
    transformed_msg2 = std::make_shared<sensor_msgs::msg::PointCloud2>(transformPCL(*pcl_ptr, transform));
    transformed_msg2->header.stamp = pcl_ptr->header.stamp;
    transformed_msg2->header.frame_id = "posegraph_map";
    return transformed_msg2;
  }


  void TraversabilityLayer::pointcloud_callback(const traversability_msgs::msg::PCL2WithNodeID::SharedPtr point_cloud_with_id)
  {
    point_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(point_cloud_with_id->pcl);
    auto posegraph = point_cloud_with_id->path;
    geometry_msgs::msg::Pose pose_selected;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Traversability layer"), "Got latest pose of size: " << posegraph.poses.size());
    for (size_t i = 0; i < posegraph.poses.size(); i++)
    {
      if (posegraph.node_ids[i] == point_cloud_with_id->node_id)
      {
        pose_selected = posegraph.poses[i];
      }
    }
    auto transformed_msg2 = TransformPCLforAddition(pose_selected, point_cloud_);
    if(transformed_msg2) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("global_traversability"), "NODE ID OF CURRENT PCL: " << point_cloud_with_id->node_id);
      TraversabilityLayer::publishtraversabilityMap(point_cloud_with_id->node_id, point_cloud_, transformed_msg2, pose_selected, true);
    }
    else
      RCLCPP_INFO_STREAM(rclcpp::get_logger("global_traversability"), "Transformed_msg2 is still Nullptr!");
    
    // TODO: Correcting the map can take sometime. 
    // Here we assume that there are no new loop closures in the time the map is being corrected.
    if(loopClosureCorrection_ == true) {
      if(correctionComplete == true) {
        correctionComplete = false;
        if(oldPoseArray) {
          std::thread{std::bind(&TraversabilityLayer::correctChangedPoses, this, std::placeholders::_1, std::placeholders::_2),*oldPoseArray, point_cloud_with_id->path}.detach();
        }
      }
    }
    oldPoseArray = std::make_shared<rtabmap_msgs::msg::Path>(point_cloud_with_id->path);
    globalTraversabilityMap_->setPoseArray(point_cloud_with_id->path);
  }

  void TraversabilityLayer::publishtraversabilityMap(int nodeid, std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_map, std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_, geometry_msgs::msg::Pose pose_selected, bool addnew)
  {
    std::lock_guard<std::mutex> lock(publishMapGaurd_);
    // Fill traversability structure
    RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_layer"), "Node ID of current pointcloud" << nodeid);
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
    grid_map::GridMap map({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation",
                           "meanx", "meany", "meanz", "cov00", "cov01", "cov02", "cov10", "cov11", "cov12", "cov20", "cov21", "cov22"});
    map.setFrameId("posegraph_map");
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

      map.at("meanx", *it) = haz(6);
      map.at("meany", *it) = haz(7);
      map.at("meanz", *it) = haz(8);

      for (int i = 0; i < 9; ++i)
      {
        map.at("cov" + std::to_string(i / 3) + std::to_string(i % 3), *it) = haz(9 + i);
      }
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("traversability_node"), "\n Covariances of grid [" << haz(9) << ", " << haz(10) << ", " << haz(11) << "] \n"
      //                                                           << "[" << haz(12) << ", " << haz(13) << ", " << haz(14) << "] \n"
      //                                                           << "[" << haz(15) << ", " << haz(16) << ", " << haz(17) << "] \n \n \n");
    }

    auto message = grid_map::GridMapRosConverter::toMessage(map);
    pubTraversability_->publish(std::move(message));

    // convert grid map to CV image.
    cv::Mat originalImage, destImage;
    bool useTransparency = false;
    if (useTransparency)
    {
      // Note: The template parameters have to be set based on your encoding
      // of the image. For 8-bit images use `unsigned char`.
      grid_map::GridMapCvConverter::toImage<unsigned char, 3>(
          map, "hazard", CV_8UC3, 0.0, 1.0, originalImage);
    }
    else
    {
      // TODO: 1 and 0 should be interchanged.
      grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
          map, "hazard", CV_8UC1, 0.0, 1.0, originalImage);
    }

    // Transform image for visualization
    double scaling = 2;
    cv::resize(originalImage, originalImage, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
    cv::applyColorMap(originalImage, originalImage, cv::COLORMAP_JET);

    // Draw robot footprint
    cv::Point2i center = originalImage.size() / 2;
    cv::Point2i half_rect_size = cv::Point2i(ceil(scaling * 0.5 * robot_width_ / resolution_), ceil(scaling * 0.5 * robot_length_ / resolution_));
    cv::rectangle(originalImage, center - half_rect_size, center + half_rect_size, cv::Scalar(0, 0, 255), 1);

    // Draw isodistances
    if (draw_isodistance_each_ > 0)
    {
      uint N = half_size_ * draw_isodistance_each_;
      for (uint i = 1; i < N; ++i)
        cv::circle(originalImage, center, i * scaling * draw_isodistance_each_ / resolution_, cv::Scalar(0, 0, 0));
    }

    auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", originalImage)
                    .toImageMsg();
    pubImage_->publish(*msg_.get());

    nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "hazard", 0., 1., occupancyGrid_msg);
    pubOccupancy_->publish(occupancyGrid_msg);

    if(addnew == true)
      globalTraversabilityMap_->insertGridMapNodeIDPCL(map, nodeid, *pcl_map, pose_selected);
    if(addnew == false) {
      // TODO: If loop closure detection parameter is off, there is no need to store the point clouds in the storage class.
      globalTraversabilityMap_->deleteGridMapNodeIDPCL(nodeid);
      globalTraversabilityMap_->insertGridMapNodeIDPCL(map, nodeid, *pcl_map, pose_selected);
    }
  }

  void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
  {

    // *min_x = *min_x;
    // *min_y = *min_y;
    // *max_x = *max_x;
    // *max_y = *max_y;
    *min_x = -300.0;
    *min_y = -300.0;
    *max_x = 300.0;
    *max_y = 300.0;
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("traversability_layer"),
        "updateBounds: robot_x = " << robot_x << ", robot_y = " << robot_y << ", robot_yaw = " << robot_yaw << ", min_x = " << *min_x << ", min_y = " << *min_y << ", max_x = " << *max_x << ", max_y = " << *max_y << "Resolution = " << resolution_;);
  }

  void TraversabilityLayer::correctChangedPoses(rtabmap_msgs::msg::Path oldposegraph, rtabmap_msgs::msg::Path newposegraph)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("global_traversability"), "Correct Poses function Called");
    std::map<int, geometry_msgs::msg::Pose> oldposemap;
    std::map<int, geometry_msgs::msg::Pose> newposemap;
    for (size_t i=0; i < oldposegraph.poses.size(); i++) {
      oldposemap[oldposegraph.node_ids[i]] = oldposegraph.poses[i];
    }
    for (size_t i=0; i < newposegraph.poses.size(); i++) {
      newposemap[newposegraph.node_ids[i]] = newposegraph.poses[i];
    }
    auto rate = rclcpp::Rate(10);
    for (const auto& pair : oldposemap) {
      auto nodeidselected = pair.first;
      if(oldposemap[nodeidselected] != newposemap[nodeidselected]) {
        for (auto storage : globalTraversabilityMap_->getGridMapsWithIDPCL()) {
          if (storage.node_id == nodeidselected) {
            auto pcl_selected = std::make_shared<sensor_msgs::msg::PointCloud2>(storage.pcl);
            RCLCPP_ERROR(rclcpp::get_logger("global_traversability"), "Identified modified pose");
            auto transformed_msg2 = TransformPCLforCorrection(newposemap[nodeidselected], pcl_selected, point_cloud_->header);
            if(transformed_msg2) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger("global_traversability"), "NODE ID OF CURRENT PCL MODIFIED: " << nodeidselected);
              TraversabilityLayer::publishtraversabilityMap(nodeidselected, pcl_selected, transformed_msg2, newposemap[nodeidselected], false);
              // rate.sleep();
            }
          }
        }
      }
    }
    correctionComplete = true;
  }

  // std::map<std::string, double> TraversabilityLayer::KalmanFilter(std::map<std::string, double> predValueAtPosition, std::map<std::string, double> measValueAtPosition) {
  //   // Prediction
  //   Eigen::Vector3d predmean = Eigen::Vector3d::Zero();
  //   predmean(0) = predValueAtPosition["meanx"];
  //   predmean(1) = predValueAtPosition["meany"];
  //   predmean(2) = predValueAtPosition["meanz"];
    
  //   Eigen::Matrix3d predCov = Eigen::Matrix3d::Zero();
  //   for (int i = 0; i < 9; ++i)
  //   {
  //     predCov(i / 3 , i % 3) = predValueAtPosition["cov" + std::to_string(i / 3) + std::to_string(i % 3)];
  //   }

  //   // Correction
  //   Eigen::Vector3d measmean = Eigen::Vector3d::Zero();
  //   measmean(0) = measValueAtPosition["meanx"];
  //   measmean(1) = measValueAtPosition["meany"];
  //   measmean(2) = measValueAtPosition["meanz"];
    
  //   Eigen::Matrix3d measCov = Eigen::Matrix3d::Zero();
  //   for (int i = 0; i < 9; ++i)
  //   {
  //     measCov(i / 3 , i % 3) = measValueAtPosition["cov" + std::to_string(i / 3) + std::to_string(i % 3)];
  //   }

    



  // }

  void TraversabilityLayer::updateLayer(nav2_costmap_2d::Costmap2D &master_grid, double robot_pose_x, double robot_pose_y, std::map<std::string, double> valuesatposition, float positionx, float positiony)
  {
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int mx, my;
    master_grid.worldToMap(robot_pose_x + positionx + 0.125, robot_pose_y + positiony - 0.125, mx, my);
    int index = master_grid.getIndex(mx, my);
    std::vector<double> coord_values = {robot_pose_x + positionx + 0.125, robot_pose_y + positiony - 0.125};
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

  void TraversabilityLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("traversability_layer"),
        "updateCosts: "
            << ", min_i = " << min_i << ", min_j = " << min_j << ", max_i = " << max_i << ", max_j = " << max_j << "Resolution = " << resolution_;);

    auto returnval = globalTraversabilityMap_->getGridMapsWithIDPCL();
    if (returnval.size() > 0)
    {
      RCLCPP_INFO_STREAM(
          rclcpp::get_logger("traversability_layer"), "Update costs: Node ID" << returnval[returnval.size() - 1].node_id);
      for (size_t i = 0; i < returnval.size(); i++)
      {
        auto grid_map_selected = returnval[i].gridMsg;
        auto pose_selected = returnval[i].pose;
        for (grid_map::GridMapIterator it(grid_map_selected); !it.isPastEnd(); ++it)
        {
          grid_map::Position position;
          grid_map_selected.getPosition(*it, position);
          std::map<std::string, double> valuesatposition;
          for (const std::string& layer : grid_map_selected.getLayers()) {
            valuesatposition[layer] = grid_map_selected.atPosition(layer, position);
          }
          if (std::isnan(valuesatposition["hazard"]) == false)
          {
            updateLayer(master_grid, pose_selected.position.x, pose_selected.position.y, valuesatposition, position.x(), position.y());
          }
        }
      }
    }





    // unsigned int mx, my;
    // master_grid.worldToMap(-1.0,-1.0,mx,my);
    // RCLCPP_INFO_STREAM(
    // rclcpp::get_logger("traversability_layer"), "The mx and my are: " << mx << " , " << my);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(traversability_gridmap::TraversabilityLayer, nav2_costmap_2d::Layer)