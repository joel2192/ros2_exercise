#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class AddClient : public rclcpp::Node
{
public:
    AddClient()
    : Node("add_client")
    {
        client_ =
            this->create_client<example_interfaces::srv::AddTwoInts>(
                "add_two_ints"
            );
    }

    void send_request(int64_t a, int64_t b)
    {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Interrupted while waiting for service."
                );
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for service..."
            );
        }

        auto request =
            std::make_shared<
                example_interfaces::srv::AddTwoInts::Request>();

        request->a = a;
        request->b = b;

        auto future = client_->async_send_request(request);

        auto result = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            future
        );

        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(
                this->get_logger(),
                "Result: %ld + %ld = %ld",
                a,
                b,
                future.get()->sum
            );
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to call service."
            );
        }
    }

private:
    rclcpp::Client<
        example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddClient>();

    node->send_request(3, 5);

    rclcpp::shutdown();

    return 0;
}