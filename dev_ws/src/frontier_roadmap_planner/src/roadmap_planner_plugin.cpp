#include "frontier_roadmap_planner/roadmap_planner_plugin.hpp"

namespace frontier_roadmap_planner
{
void FrontierRoadmapPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  global_frame_ = costmap_ros->getGlobalFrameID();
  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "frontier_roadmap_nav2_plan", rclcpp::QoS(10),
    std::bind(&FrontierRoadmapPlanner::planCallback, this, std::placeholders::_1));
}

// Callback function for plan messages
void FrontierRoadmapPlanner::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(latest_plan_mutex_);
  latest_plan_ = *msg;
}

void FrontierRoadmapPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type frontier_roadmap_planner_planner", name_.c_str());
}

void FrontierRoadmapPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type frontier_roadmap_planner_planner", name_.c_str());
}

void FrontierRoadmapPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type frontier_roadmap_planner_planner", name_.c_str());
}

nav_msgs::msg::Path FrontierRoadmapPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  (void)start;
  (void)goal;
  std::lock_guard<std::mutex> lock(latest_plan_mutex_);
  nav_msgs::msg::Path plan;
  if (latest_plan_.poses.size() == 0) {
    RCLCPP_WARN(logger_, "No plan received");
    return plan;
  }
  plan = linearInterpolation(latest_plan_, 0.3);
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;
  return plan;
}

nav_msgs::msg::Path FrontierRoadmapPlanner::linearInterpolation(
  const nav_msgs::msg::Path & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.poses.size() - 1; j++) {
    auto pt1 = raw_path.poses[j];
    p1 = pt1;
    pa.poses.push_back(p1);

    auto pt2 = raw_path.poses[j + 1];
    double distance = std::hypot(pt2.pose.position.x - pt1.pose.position.x, pt2.pose.position.y - pt1.pose.position.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.pose.position.y - pt1.pose.position.y) / distance;
    double cos_alpha = (pt2.pose.position.x - pt1.pose.position.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.pose.position.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.pose.position.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  return pa;
}

}  // namespace frontier_roadmap_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(frontier_roadmap_planner::FrontierRoadmapPlanner, nav2_core::GlobalPlanner)