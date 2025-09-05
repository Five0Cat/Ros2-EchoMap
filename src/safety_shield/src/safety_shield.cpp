#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <optional>
#include <utility>
#include "local_mapper/utils.hpp"
class SafetyShield : public rclcpp::Node {
public:
  SafetyShield() : Node("safety_shield") {
    // Declare parameters
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/cmd_vel_safe_marker", 10);
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_safe", 10,
        std::bind(&SafetyShield::onCmd, this, std::placeholders::_1));
    grid_topic_   = this->declare_parameter<std::string>("grid_topic", "/local_grid");
    in_cmd_topic_ = this->declare_parameter<std::string>("in_cmd", "/cmd_vel");
    out_cmd_topic_= this->declare_parameter<std::string>("out_cmd", "/cmd_vel_safe");

    stop_distance_m_ = this->declare_parameter<double>("stop_distance", 0.8);
    lateral_half_m_  = this->declare_parameter<double>("lateral_half", 0.3);
    occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 50);
    verbose_ = this->declare_parameter<bool>("verbose", false);

    pub_safe_ = this->create_publisher<geometry_msgs::msg::Twist>(out_cmd_topic_, 10);

    sub_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&SafetyShield::onGrid, this, std::placeholders::_1));

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      in_cmd_topic_, 10,
      std::bind(&SafetyShield::onCmd, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "SafetyShield up: grid=%s, in=%s, out=%s, stop=%.2fm, width=±%.2fm, thr=%d",
      grid_topic_.c_str(), in_cmd_topic_.c_str(), out_cmd_topic_.c_str(),
      stop_distance_m_, lateral_half_m_, occupied_threshold_);
  }

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    grid_ = *msg;
    gi_.res = grid_.info.resolution;
    gi_.w   = grid_.info.width;
    gi_.h   = grid_.info.height;
    gi_.ox  = grid_.info.origin.position.x;
    gi_.oy  = grid_.info.origin.position.y;
    grid_ready_ = (gi_.w > 0 && gi_.h > 0 && gi_.res > 0.0);
  }

  void onCmd(const geometry_msgs::msg::Twist::SharedPtr cmd) {
    geometry_msgs::msg::Twist out = *cmd;
    bool obstacle = false;
     visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // 确保和你实际使用的坐标系一致
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cmd_vel_safe";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置箭头的起点
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置箭头的方向（根据 linear.x 和 angular.z）
    marker.scale.x = 0.1;  // 箭头的粗细
    marker.scale.y = 0.2;  // 箭头的宽度
    marker.scale.z = 0.0;  // 不需要 Z 方向的扩展

    marker.color.a = 1.0;  // 不透明
    marker.color.r = 0.0;  // 红色
    marker.color.g = 1.0;  // 绿色
    marker.color.b = 0.0;  // 蓝色

    // 设置箭头的方向（基于速度的 linear.x 和 angular.z）
    marker.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(cmd->angular.z / 2), cos(cmd->angular.z / 2)));
    marker.pose.position.x = cmd->linear.x;  // 前进方向
    marker.pose.position.y = 0.0;           // 保持原地不动

    pub_marker_->publish(marker);
    if (grid_ready_) {
      const double step = std::max(gi_.res, 0.05);
      for (double x = 0.0; x <= stop_distance_m_; x += step) {
        for (double y = -lateral_half_m_; y <= lateral_half_m_; y += step) {
          auto cell = lm::worldToCell(x, y, gi_);
          if (!cell) continue;
          int idx = lm::cellIndex(cell->first, cell->second, gi_.w);
          int v = grid_.data[idx];
          if (v >= occupied_threshold_) { obstacle = true; break; }
        }
        if (obstacle) break;
      }
    }

    if (obstacle && cmd->linear.x > 0.0) {
      out.linear.x = 0.0;
      if (verbose_) {
        RCLCPP_WARN(this->get_logger(),
          "Shield activated: obstacle within %.2fm ahead, stopping.",
          stop_distance_m_);
      }
    }

    pub_safe_->publish(out);
  }

  // Members
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_safe_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;

  nav_msgs::msg::OccupancyGrid grid_;
  lm::GridInfo gi_;
  bool grid_ready_{false};

  std::string grid_topic_, in_cmd_topic_, out_cmd_topic_;
  double stop_distance_m_{0.8}, lateral_half_m_{0.3};
  int occupied_threshold_{50};
  bool verbose_{false};
};

int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SafetyShield>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0 ;
}