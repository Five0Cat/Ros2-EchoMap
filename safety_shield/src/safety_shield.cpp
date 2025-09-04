#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleNode : public rclcpp::Node{
    public:
        SimpleNode():Node("simple_node"){
            RCLCPP_INFO(this->get_logger(),">>> Constructor: Node is created!");
        timer_ = create_wall_timer(std::chrono::seconds(1),
          [this]{ RCLCPP_INFO(get_logger(), ">>> onTimer tick"); 
      }
        sub_ = create_subscription<std_msgs::msg::String>(
        "/chatter", 10,
        [this](std_msgs::msg::String::SharedPtr msg){
            this->onMsg(msg);   // 直接调用类里的成员函数
        });
);
    }
    private:
         rclcpp::TimerBase::SharedPtr timer_;
  };
int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleNode>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0 ;
}