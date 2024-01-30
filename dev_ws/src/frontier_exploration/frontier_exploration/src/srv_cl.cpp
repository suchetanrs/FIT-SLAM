#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <thread>

using namespace std::placeholders;

class MinimalServerClient : public rclcpp::Node
{
public:
    MinimalServerClient() : Node("minimal_server_client")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                            std::bind(&MinimalServerClient::handle_service_request, this, _1, _2, _3));

        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        // Wait for the server to be ready before sending requests
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server not available, waiting...");
        }

        // Send requests to the server
        // send_request(1, 2);
        // send_request(5, 3);
        // send_request(10, 20);
        // std::thread t1(&MinimalServerClient::send_request, this);
        std::thread{std::bind(&MinimalServerClient::send_request, this)}.detach();
        // while (currently_processing_ == true)
        // {
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));
        // }
        std::thread{std::bind(&MinimalServerClient::send_request2, this)}.detach();
        // t1.join();
        // t2.join();
        // send_request();
        // send_request2();
    }

private:
    void send_request()
    {
        int a = 1;
        int b = 2;
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        RCLCPP_INFO_STREAM(this->get_logger(), "Sent request via thread: " << std::this_thread::get_id());
        auto future_result = client_->async_send_request(request);
        std::unique_lock<std::mutex> lock(processing_lock_);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, result->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response");
        }
        return;
    }

    void send_request2()
    {
        int a = 3;
        int b = 4;
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        RCLCPP_INFO_STREAM(this->get_logger(), "Sent request2 via thread: " << std::this_thread::get_id());
        std::unique_lock<std::mutex> lock(processing_lock_);
        auto future_result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(2)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, result->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response");
        }
        return;
    }

    void handle_service_request(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: %d + %d = %d", request->a, request->b, response->sum);
        rclcpp::sleep_for(std::chrono::seconds(5));
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    bool currently_processing_ = true;
    std::mutex processing_lock_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalServerClient>());
    rclcpp::shutdown();
    return 0;
}