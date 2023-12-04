#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/srv/get_allocated_goal.hpp"
#include "frontier_msgs/srv/load_frontier_costs.hpp"
#include <iostream>
#include <chrono>

class GetAllocatedGoalServer : public rclcpp::Node
{
public:
  GetAllocatedGoalServer()
    : Node("goal_allocation_server")
  {
    RCLCPP_INFO(logger_, "Goal Allocation server initializing");
    service_get_goal_ = this->create_service<frontier_msgs::srv::GetAllocatedGoal>(
      "/get_allocated_goal", std::bind(&GetAllocatedGoalServer::handle_service_get_goal_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    service_load_cost_data_ = this->create_service<frontier_msgs::srv::LoadFrontierCosts>(
      "/send_latest_frontier_costs", std::bind(&GetAllocatedGoalServer::handle_load_cost_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    robot_namespaces_ = {"/scout_1", "/scout_2"};
    this->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
    this->get_parameter("robot_namespaces", robot_namespaces_);
    int count = 0;
    for(auto ns : robot_namespaces_) {
      robot_namespaces_id_[ns] = count;
      ++count;
      robot_recieved_latest_data_[ns] = false;
      RCLCPP_INFO_STREAM(logger_, "Robot with namespace " << ns << " initialized.");
    }
  }

private:
  void handle_load_cost_data(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<frontier_msgs::srv::LoadFrontierCosts::Request> request,
    std::shared_ptr<frontier_msgs::srv::LoadFrontierCosts::Response> response)
  {
    robot_recieved_latest_data_[request->robot_namespace] = true;
    response->success = true;
    RCLCPP_INFO_STREAM(logger_, "RECEIVED DATA UPDATE FOR ROBOT: " << request->robot_namespace);
    RCLCPP_INFO_STREAM(logger_, "Frontiers list len: " << request->frontiers.size());
    RCLCPP_INFO_STREAM(logger_, "Costs list len: " << request->costs.size());
    for (auto pr : request->costs) {
      RCLCPP_INFO_STREAM(logger_, "The frontier cost: " << pr);
    }
    RCLCPP_INFO(logger_, "*********************");
    service_load_cost_data_->send_response(*request_header, *response);
  }


  void handle_service_get_goal_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Request> request,
    std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Response> response)
  {
    bool send_allocation_data_ = false;
    while(send_allocation_data_ == false && rclcpp::ok()) {
      send_allocation_data_ = true;
      for(auto ns : robot_namespaces_) {
        if(robot_recieved_latest_data_[ns] == false) {
          RCLCPP_INFO_STREAM(logger_, "Waiting for information from robot: " << ns);
          send_allocation_data_ = false;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    for(auto ns : robot_namespaces_) {
      RCLCPP_INFO_STREAM(logger_, "The bool value for " << ns << " is: " << robot_recieved_latest_data_[ns]);
    }
    // robot_recieved_latest_data_[request->robot_namespace] = false;

    response->success = true;
    RCLCPP_INFO_STREAM(logger_, "Returned the allocated goal for robot: " << request->robot_namespace);
    RCLCPP_INFO(logger_, "==============================");
    service_get_goal_->send_response(*request_header, *response);
  }

  rclcpp::Service<frontier_msgs::srv::GetAllocatedGoal>::SharedPtr service_get_goal_;
  rclcpp::Service<frontier_msgs::srv::LoadFrontierCosts>::SharedPtr service_load_cost_data_;
  rclcpp::Logger logger_ = rclcpp::get_logger("goal_allocation_server");
  std::vector<std::string> robot_namespaces_;
  std::map<std::string, int> robot_namespaces_id_;
  std::map<std::string, bool> robot_recieved_latest_data_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetAllocatedGoalServer>());
  rclcpp::shutdown();
  return 0;
}