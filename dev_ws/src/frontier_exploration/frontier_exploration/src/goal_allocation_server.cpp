#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/srv/get_allocated_goal.hpp"
#include "frontier_msgs/msg/load_frontier_costs.hpp"
#include <iostream>
#include <chrono>

class GetAllocatedGoalServer : public rclcpp::Node
{
public:
  GetAllocatedGoalServer()
    : Node("goal_allocation_server")
  {
    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;

    RCLCPP_INFO(logger_, "Goal Allocation server initializing");
    service_get_goal_ = this->create_service<frontier_msgs::srv::GetAllocatedGoal>(
      "/get_allocated_goal", std::bind(&GetAllocatedGoalServer::handle_service_get_goal_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    sub_load_costs_data_ = this->create_subscription<frontier_msgs::msg::LoadFrontierCosts>(
      "/send_latest_frontier_costs", rclcpp::QoS(10), std::bind(&GetAllocatedGoalServer::handle_load_cost_data, this, std::placeholders::_1), sub1_opt);
    
    robot_namespaces_ = {"/scout_2"};
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
  void handle_load_cost_data(frontier_msgs::msg::LoadFrontierCosts::SharedPtr cost_data)
  {
    RCLCPP_INFO_STREAM(logger_, "The thread ID for this function call is cost data:: " << std::this_thread::get_id());
    robot_recieved_latest_data_[cost_data->robot_namespace] = true;
    RCLCPP_INFO_STREAM(logger_, "RECEIVED DATA UPDATE FOR ROBOT: " << cost_data->robot_namespace);
    RCLCPP_INFO_STREAM(logger_, "Frontiers list len: " << cost_data->frontiers.size());
    RCLCPP_INFO_STREAM(logger_, "Costs list len: " << cost_data->costs.size());
    for (auto pr : cost_data->costs) {
      RCLCPP_INFO_STREAM(logger_, "The frontier cost: " << pr);
    }
    RCLCPP_INFO(logger_, "*********************");
  }

  void handle_service_get_goal_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Request> request,
    std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Response> response)
  {
    RCLCPP_INFO_STREAM(logger_, "The thread ID for this function call is goal allocation:: " << std::this_thread::get_id());
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
    robot_recieved_latest_data_[request->robot_namespace] = false;

    response->success = true;
    RCLCPP_INFO_STREAM(logger_, "Returned the allocated goal for robot: " << request->robot_namespace);
    RCLCPP_INFO(logger_, "==============================");
    service_get_goal_->send_response(*request_header, *response);
  }

  rclcpp::Service<frontier_msgs::srv::GetAllocatedGoal>::SharedPtr service_get_goal_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::Subscription<frontier_msgs::msg::LoadFrontierCosts>::SharedPtr sub_load_costs_data_;
  rclcpp::Logger logger_ = rclcpp::get_logger("goal_allocation_server");
  std::vector<std::string> robot_namespaces_;
  std::map<std::string, int> robot_namespaces_id_;
  std::map<std::string, bool> robot_recieved_latest_data_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto rosnode = std::make_shared<GetAllocatedGoalServer>();
  executor.add_node(rosnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}




        // // PASS THE FRONTIER LIST WITH COSTS TO THE MULTI ROBOT GOAL ALLOCATOR
        
        // frontier_msgs::msg::LoadFrontierCosts frontier_costs_msg_;
        // std::vector<frontier_msgs::msg::Frontier> frontiers_list;
        // std::vector<double> frontier_costs;
        // for (auto pair : selection_result.frontier_costs) {
        //     // Extract key and value
        //     auto key = pair.first.frontier_; // frontier with meta data
        //     auto value = pair.second; // cost.

        //     // Push them into respective vectors
        //     frontiers_list.push_back(key);
        //     frontier_costs.push_back(value);
        // }
        // frontier_costs_msg_.robot_namespace = std::string{standard_node_->get_namespace()};
        // frontier_costs_msg_.frontiers = frontiers_list;
        // frontier_costs_msg_.costs = frontier_costs;

        // pub_load_frontier_costs_->publish(frontier_costs_msg_);


        // // GET THE ALLOCATED GOAL

        // auto request_get_allocated_goal = std::make_shared<frontier_msgs::srv::GetAllocatedGoal::Request>();
        // request_get_allocated_goal->robot_namespace = std::string{standard_node_->get_namespace()};

        // while (!client_get_allocated_goal_->wait_for_service(std::chrono::seconds(1))) {
        //     if (!rclcpp::ok()) {
        //         RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
        //         rclcpp::shutdown();
        //     }
        //     RCLCPP_INFO(logger_, "get goal service not available, waiting again...");
        // }

        // auto result_get_allocated_goal = client_get_allocated_goal_->async_send_request(request_get_allocated_goal);
        // std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal_Response> get_allocated_goal_srv_res;
        // if (rclcpp::spin_until_future_complete(standard_node_, result_get_allocated_goal) == rclcpp::FutureReturnCode::SUCCESS) {
        //     get_allocated_goal_srv_res = result_get_allocated_goal.get();
        //     if (get_allocated_goal_srv_res->success == false) {
        //         RCLCPP_INFO(standard_node_->get_logger(), "Get Allocation failed");
        //     } else {
        //         // Process the received poses as needed
        //         RCLCPP_INFO_STREAM(standard_node_->get_logger(), "Response of getting allocated goal: " << get_allocated_goal_srv_res->success);
        //     }
        // } else {
        // RCLCPP_ERROR(standard_node_->get_logger(), "Failed to call the service /get_allocated_goal");
        // }