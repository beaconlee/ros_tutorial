#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>



class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher()
    : Node("grid_map_publisher")
  {
    publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);

    traj_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("dp_traj", 10);

    PublishMap();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GridMapPublisher::PublishTraj, this));

    traj_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GridMapPublisher::PublishTraj, this));
  }

private:
  void PublishTraj()
  {
    // sphere 领域   line strip 线条
    visualization_msgs::msg::Marker sphere;
    sphere.header.stamp = this->get_clock()->now();
    sphere.header.frame_id = "map";

    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::msg::Marker::ADD;

    sphere.pose.orientation.set__w(0.2);
    sphere.color.set__r(4.);
    sphere.color.set__g(255);
    sphere.color.set__b(4.);
    sphere.color.set__a(1.);

    sphere.scale.x = 0.01;
    sphere.scale.y = 0.01;
    sphere.scale.z = 0.01;

    static double y = 1;
    static double x = 2;

    if(y > 2)
    {
      y = 1;
      x = 2;
    }

    geometry_msgs::msg::Point point;
    for(size_t idx = 0; idx < 100; ++idx)
    {
      point.set__x(idx * 0.01 * x);
      point.set__y(idx * 0.01 * y);
      point.set__z(0.1);
      sphere.points.push_back(point);
    }

    traj_publisher_->publish(sphere);
    x += 0.3;
    y += 0.3;
  }

  void PublishMap()
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

    // Initialize map data
    msg.data.resize(msg.info.width * msg.info.height, 0);

    // Draw some rectangles
    draw_rectangle(msg.data, msg.info.width, 20, 20, 30, 30, 200);
    draw_rectangle(msg.data, msg.info.width, 50, 50, 70, 70, 200);

    // Draw a road
    draw_road(msg.data, msg.info.width, 10, 80, 40, 50, 0);

    RCLCPP_INFO(this->get_logger(), "Publishing grid map");
    publisher_->publish(msg);
  }

  void draw_rectangle(std::vector<int8_t>& data,
                      int width,
                      int x0,
                      int y0,
                      int x1,
                      int y1,
                      int value)
  {
    for(int y = y0; y <= y1; ++y)
    {
      for(int x = x0; x <= x1; ++x)
      {
        if(x >= 0 && x < width && y >= 0 && y < width) // Check bounds
        {
          data[y * width + x] = value;
        }
      }
    }
  }

  void draw_road(std::vector<int8_t>& data,
                 int width,
                 int x0,
                 int x1,
                 int y0,
                 int y1,
                 int value)
  {
    for(int y = y0; y <= y1; ++y)
    {
      for(int x = x0; x <= x1; ++x)
      {
        if(x >= 0 && x < width && y >= 0 && y < width) // Check bounds
        {
          data[y * width + x] = value;
        }
      }
    }
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr traj_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapPublisher>());
  rclcpp::shutdown();
  return 0;
}