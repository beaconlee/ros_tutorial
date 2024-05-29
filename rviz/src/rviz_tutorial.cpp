// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <iostream>
// #include "nav_msgs/msg/path.hpp"
// #include <string>


// using iString = std_msgs::msg::String;

// class RvizNode : public rclcpp::Node
// {
// public:
//   RvizNode(const std::string& node_name)
//     : rclcpp::Node(node_name)
//   {
//     publisher_ = this->create_publisher<iString>("string_topic", 10);
//   }

// private:
//   rclcpp::Publisher<iString>::SharedPtr publisher_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<RvizNode>("rviz_tutorial");

//   rclcpp::executors::SingleThreadedExecutor executors;

//   executors.add_node(node);
//   executors.spin();
//   rclcpp::Time now = rclcpp::Clock().now();
//   rclcpp::Time last = rclcpp::Clock().now();

//   nav_msgs::msg::Path path;


//   printf("hello world rviz package\n");
//   rclcpp::shutdown();

//   return 0;
// }

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"

// class PathPublisher : public rclcpp::Node
// {
// public:
//   PathPublisher()
//     : Node("path_publisher")
//   {
//     publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
//     timer_ =
//         this->create_wall_timer(std::chrono::milliseconds(500),
//                                 std::bind(&PathPublisher::publish_path, this));
//   }

// private:
//   void publish_path()
//   {
//     auto path = nav_msgs::msg::Path();
//     path.header.stamp = this->now();
//     path.header.frame_id = "odom";

//     // 生成一些假设的路径点
//     for(int i = 0; i < 10; ++i)
//     {
//       geometry_msgs::msg::PoseStamped pose;
//       pose.header.stamp = this->now();
//       pose.header.frame_id = "odom";
//       pose.pose.position.x = i * 0.5;
//       pose.pose.position.y = i * 0.5;
//       pose.pose.position.z = 0.0;
//       pose.pose.orientation.w = 1.0;

//       path.poses.push_back(pose);
//     }

//     std::cout << "fabushuju" << std::endl;


//     publisher_->publish(path);
//   }

//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PathPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
    : Node("path_publisher")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&PathPublisher::publish_path, this));
  }

private:
  void publish_path()
  {
    auto path = nav_msgs::msg::Path();
    path.header.stamp = this->now();
    path.header.frame_id = "map";

    static int j = 1;
    ++j;
    if(j > 4)
    {
      j = 1;
    }
    for(int i = 0; i < 10; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = i * 0.5 * j;
      pose.pose.position.y = i * 0.5 * j;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    publisher_->publish(path);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  RCLCPP_INFO(node->get_logger(), "wudi");


  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
