#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher()
    : Node("grid_map_publisher")
  {
    publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GridMapPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = nav_msgs::msg::OccupancyGrid();
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();

    msg.info.width = 100;
    msg.info.height = 100;
    msg.info.resolution = 0.1;
    msg.info.origin.position.x = -5.0;
    msg.info.origin.position.y = -5.0;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(msg.info.width * msg.info.height);
    for(auto& cell : msg.data)
    {
      cell = rand() % 100; // 随机填充地图数据
    }

    RCLCPP_INFO(this->get_logger(), "Publishing grid map");
    publisher_->publish(msg);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapPublisher>());
  rclcpp::shutdown();
  return 0;
}