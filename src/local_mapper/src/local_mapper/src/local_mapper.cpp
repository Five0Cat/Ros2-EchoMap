#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "local_mapper/utils.hpp"

class LocalMapper : public rclcpp::Node {
public:
  LocalMapper() : rclcpp::Node("local_mapper") {
    // 1) 读取参数（可从 YAML 载入）
    // declare_parameter + get_parameter，或直接硬编码第一版
    frame_id_ = this->declare_parameter<std::string>("grid.frame_id", "map");
    res_      = this->declare_parameter<double>("grid.resolution", 0.05);
    w_        = this->declare_parameter<int>("grid.width", 200);
    h_        = this->declare_parameter<int>("grid.height", 200);
    ox_       = this->declare_parameter<double>("grid.origin_x", -5.0);
    oy_       = this->declare_parameter<double>("grid.origin_y", -5.0);

    max_range_ = this->declare_parameter<double>("sensor.max_range", 6.0);
    mark_val_  = this->declare_parameter<int>("sensor.mark_value", 100);

    setupGridMsg_();

    pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_grid", 1);

    // 2) 订阅 /scan（SensorDataQoS）
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->declare_parameter<std::string>("sensor.topic", "/scan"),
      rclcpp::SensorDataQoS(),
      std::bind(&LocalMapper::onScan, this, std::placeholders::_1));

    // 3) 定时发布（把当前 grid_ 发出去；你也可在 onScan 里直接发）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalMapper::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "LocalMapper D2 up: res=%.2f w=%d h=%d origin=(%.2f,%.2f)",
                res_, w_, h_, ox_, oy_);
  }

private:
  // 初始化 OccupancyGrid 头和元信息
  void setupGridMsg_() {
    grid_.header.frame_id = frame_id_;
    grid_.info.resolution = res_;
    grid_.info.width = w_;
    grid_.info.height = h_;
    grid_.info.origin.position.x = ox_;
    grid_.info.origin.position.y = oy_;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(w_*h_, -1); // unknown
  }

  // 把一条 scan 写进 grid_.data（核心逻辑：你来填）
  void rasterizeScan_(const sensor_msgs::msg::LaserScan& scan) {
    const float r_max = std::min<float>(scan.range_max,static_cast<float>(max_range_));
    const float r_min = std::max<float>(0.f,scan.range_min);
    const int N = static_cast<int>(scan.ranges.size());
    
    if(N <= 0) return ;

    for(int i = 0 ; i < N ; ++i){
      float r = scan.ranges[i];
      if(!std::isfinite(r)) continue;
      if(r < r_min || r > r_max) continue;

      const float theta = scan.angle_min + i * scan.angle_increment;
        // Sensor assumed at (0,0) in grid frame_id_
    const double x = static_cast<double>(r) * std::cos(theta);
    const double y = static_cast<double>(r) * std::sin(theta);

    if (auto ij = lm::worldToCell(x, y, {res_, w_, h_, ox_, oy_})){
  const int idx = lm::cellIndex(ij->first, ij->second, w_);
      if (idx >= 0 && idx < static_cast<int>(grid_.data.size())) {
        grid_.data[idx] = static_cast<int8_t>(mark_val_); // e.g. 100 = occupied
      }
    }
    }

  }

  // 激光回调：更新内部 grid_，刷新时间戳
 void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //    -1 = unknown, 0 = free, 100 = occupied
  std::fill(grid_.data.begin(), grid_.data.end(), -1);

  rasterizeScan_(*msg);

  // 3) 更新时间戳和 frame_id
  last_stamp_ = msg->header.stamp;
  grid_.header.stamp = last_stamp_;
  grid_.header.frame_id = frame_id_;   

  // 4) 发布 OccupancyGrid
  pub_grid_->publish(grid_);
}

  void onTimer() {
    grid_.header.stamp = (last_stamp_.nanoseconds()==0) ? this->now() : last_stamp_;
    pub_grid_->publish(grid_);
  }

  // 成员
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid grid_;

  // 参数
  std::string frame_id_;
  double res_{0.05};
  int w_{165}, h_{165};
  double ox_{-5.0}, oy_{-5.0};
  double max_range_{6.0};
  int mark_val_{100};

  rclcpp::Time last_stamp_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMapper>());
  rclcpp::shutdown();
  return 0;
}
