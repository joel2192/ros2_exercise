#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddServer : public rclcpp::Node
{
public:
    AddServer()
    : Node("add_server")
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(
                &AddServer::add,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(), "Service is ready.");
    }

private:
    void add(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;

        RCLCPP_INFO(
            this->get_logger(),
            "%ld + %ld = %ld",
            request->a,
            request->b,
            response->sum
        );
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}