#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PubNode : public rclcpp::Node {
public:
  PubNode() : Node("pub_node") {
    // 创建 publisher
    pub_ = create_publisher<std_msgs::msg::String>("/chatter", 10);

    // 创建定时器，每 1 秒发布一次
    timer_ = create_wall_timer(std::chrono::seconds(1),
      [this]() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from publisher!";
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published: '%s'", msg.data.c_str());
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubNode>());
  rclcpp::shutdown();
  return 0;
}