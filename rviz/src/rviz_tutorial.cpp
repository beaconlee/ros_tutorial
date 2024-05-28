#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include <string>


using iString = std_msgs::msg::String;

class RvizNode : public rclcpp::Node
{
public:
  RvizNode(const std::string& node_name)
    : rclcpp::Node(node_name)
  {
    publisher_ = this->create_publisher<iString>("string_topic", 10);
  }

private:
  rclcpp::Publisher<iString>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RvizNode>("rviz_tutorial");

  rclcpp::executors::SingleThreadedExecutor executors;

  executors.add_node(node);
  executors.spin();
  rclcpp::Time now = rclcpp::Clock().now();
  rclcpp::Time last = rclcpp::Clock().now();

  nav_msgs::msg::Path path;


  printf("hello world rviz package\n");
  rclcpp::shutdown();

  return 0;
}
