#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node 
{
    public:
        NumberCounterNode() : Node("number_counter") 
        {
            number_subscriber_ = this->create_subscription<std_msgs::msg::Int64>
            ("number", 10, std::bind(&NumberCounterNode::recvNumber, this, std::placeholders::_1));
        }

    private:

        int counter = 0;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr number_subscriber_;

        void recvNumber(const std_msgs::msg::Int64::SharedPtr msg)
        {
            counter += msg->data;
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter);
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