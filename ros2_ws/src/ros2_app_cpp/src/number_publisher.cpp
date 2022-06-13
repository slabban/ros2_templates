#include "rclcpp/rclcpp.hpp"

class NumberPublisherNode : public rclcpp::Node{
    public:
        NumberPublisherNode() : Node("number_publisher"), number_(0)
        {
            number_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::print_number, this));
            RCLCPP_INFO(this->get_logger(), "Number Publisher Has Started!");
        }
    
    private:
        int number_ = 0;
        rclcpp::TimerBase::SharedPtr number_timer_;

        void print_number()
        {
            RCLCPP_INFO(this->get_logger(), "%d", number_);
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





