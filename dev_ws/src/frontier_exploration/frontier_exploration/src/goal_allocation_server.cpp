#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/srv/get_allocated_goal.hpp"

class GetAllocatedGoalServer : public rclcpp::Node
{
public:
  GetAllocatedGoalServer()
    : Node("get_allocated_goal_server")
  {
    // Create the service
    service_ = this->create_service<frontier_msgs::srv::GetAllocatedGoal>(
      "get_allocated_goal", std::bind(&GetAllocatedGoalServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void handle_service_request(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Request> request,
    const std::shared_ptr<frontier_msgs::srv::GetAllocatedGoal::Response> response)
  {
    // Handle the service request and fill the response
    // For example, you can do some processing and set the response values
    response->success = true;  // Set to true for simplicity

    // You can access request data like this:
    // request->robot_namespace
    // request->frontier_list
    // request->costs

    // Perform your processing here and assign values to the response

    // Once you're done, send the response
    RCLCPP_INFO(get_logger(), "Service request received and processed successfully");
  }

  rclcpp::Service<frontier_msgs::srv::GetAllocatedGoal>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetAllocatedGoalServer>());
  rclcpp::shutdown();
  return 0;
}