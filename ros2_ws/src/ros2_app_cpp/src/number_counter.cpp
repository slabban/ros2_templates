#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "my_robot_interfaces/srv/reset_counter.hpp"


class NumberCounterNode : public rclcpp::Node 
{
    public:
        NumberCounterNode() : Node("number_counter") 
        {
            number_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::recvNumber, this, std::placeholders::_1));
            reset_counter_service_ = this->create_service<my_robot_interfaces::srv::ResetCounter>(
            "reset_counter",
            std::bind(&NumberCounterNode::srvResetCounter, this, std::placeholders::_1, std::placeholders::_2)   
            );

        }

    private:

        int counter = 0;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr number_subscriber_;
        rclcpp::Service<my_robot_interfaces::srv::ResetCounter>::SharedPtr reset_counter_service_;

        // Defining our subscriber callback function
        void recvNumber(const std_msgs::msg::Int64::SharedPtr msg)
        {
            counter += msg->data;
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter);
        }

        // Defining our service callback function
        void srvResetCounter(const my_robot_interfaces::srv::ResetCounter::Request::SharedPtr request,
        const my_robot_interfaces::srv::ResetCounter::Response::SharedPtr response)
        {
            if (request->reset_value >= 0){
                counter = request->reset_value;
                RCLCPP_INFO(this->get_logger(), "Counter %d", counter);
                response->success = true;
            }
            else
            {
                response->success = false;
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}