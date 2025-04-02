#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "arduinobot_msgs/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<arduinobot_msgs::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Service server has been started...");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> request,
                         const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> response)
    {   
        RCLCPP_INFO(this->get_logger(), "New Request Received a: %ld, b: %ld", request->a, request->b);
        
        response->sum = request->a + request->b;

        RCLCPP_INFO(this->get_logger(), "Returning sum: %ld", response->sum);
    }

};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
