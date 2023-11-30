#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/srv/get_allocated_goal.hpp"
#include <iostream>

class GetAllocatedGoalServer : public rclcpp::Node
{
public:
  GetAllocatedGoalServer()
    : Node("goal_allocation_server")
  {
    RCLCPP_INFO(logger_, "Goal Allocation server initializing");
    service_ = this->create_service<frontier_msgs::srv::GetAllocatedGoal>(
      "/get_allocated_goal", std::bind(&GetAllocatedGoalServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    

    robot_namespaces_ = {"scout_1", "scout_2"};
    this->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
    this->get_parameter("robot_namespaces", robot_namespaces_);
    int count = 0;
    for(auto ns : robot_namespaces_) {
      robot_namespaces_id_[ns] = count;
      ++count;
    }
  }

private:
  void handle_service_request(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Request> request,
    const std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Response> response)
  {
    response->success = true;
    RCLCPP_INFO_STREAM(logger_, "Frontiers list len: " << request->frontiers.size());
    RCLCPP_INFO_STREAM(logger_, "Costs list len: " << request->costs.size());

    // Once you're done, send the response
    RCLCPP_INFO(logger_, "Service request received and processed successfully");
  }

  rclcpp::Service<frontier_msgs::srv::GetAllocatedGoal>::SharedPtr service_;
  rclcpp::Logger logger_ = rclcpp::get_logger("goal_allocation_server");
  std::vector<std::string> robot_namespaces_;
  std::map<std::string, int> robot_namespaces_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetAllocatedGoalServer>());
  rclcpp::shutdown();
  return 0;
}