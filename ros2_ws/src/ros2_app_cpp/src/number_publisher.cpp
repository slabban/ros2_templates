#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node{
    public:
        NumberPublisherNode() : Node("number_publisher"), number_(1)
        {
            number_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::publishNumber, this));
            number_publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 1);
            RCLCPP_INFO(this->get_logger(), "Number Publisher Has Started!");
        }
    
    private:
        int number_ = 1;
        rclcpp::TimerBase::SharedPtr number_timer_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr number_publisher_;


        void publishNumber()
        {
            auto msg = std_msgs::msg::Int64();
            msg.data = number_;
            number_publisher_->publish(msg);
        }

}
;

int main(int argc,char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





