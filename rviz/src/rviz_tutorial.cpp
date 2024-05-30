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
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz/quintic_polynomial.h"
#include "rviz/osqp/piecewise_jerk_problem.h"


class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
    : Node("path_publisher")
  {
    Init();
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    publisher_polygon_ =
        this->create_publisher<visualization_msgs::msg::Marker>("rectangle",
                                                                10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000),
                                std::bind(&PathPublisher::publish_path, this));

    timer_polygon_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PathPublisher::publish_rectangle, this));
  }

private:
  void Init()
  {
    beacon::QuinticCoefficients xcoff{1, 1, 1, 1, 1, 1};
    qpx_.SetCoff(xcoff);
  }
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

    Eigen::VectorXd path2;

    int size = path2.size() / 3;
    pjp_.Optimize(&path2);

    std::cout << "osqp result:" << path2 << "\n";

    static int k = -1;
    k *= -1;


    for(int i = 0; i < size; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      double t = i * 0.01;
      pose.pose.position.x = t * k;
      pose.pose.position.y = path2[i];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    publisher_->publish(path);
  }

  void publish_rectangle()
  {
    auto rectangel = visualization_msgs::msg::Marker();
    rectangel.header.stamp = this->now();
    rectangel.header.frame_id = "map";

    // 之前没有加下面的这段内容，矩阵没有显示出来
    rectangel.ns = "polygen";
    rectangel.type = visualization_msgs::msg::Marker::LINE_STRIP;
    rectangel.action = visualization_msgs::msg::Marker::ADD;

    rectangel.pose.orientation.w = 1.0;
    rectangel.scale.x = 0.1;
    rectangel.color.r = 1.0;
    rectangel.color.a = 1.0;

    std::vector<std::pair<double, double>> pose2{{
                                                     1,
                                                     1,
                                                 },
                                                 {5, 1},
                                                 {5, 5},
                                                 {1, 5}};

    for(int i = 0; i < 4; ++i)
    {
      geometry_msgs::msg::Point pose;
      pose.set__x(pose2[i].first);
      pose.set__y(pose2[i].second);
      pose.set__z(0.);
      rectangel.points.push_back(pose);
    }

    // error 这里又忘了将矩阵围成一个圈了
    geometry_msgs::msg::Point pose;
    pose.set__x(pose2.begin()->first);
    pose.set__y(pose2.begin()->second);
    pose.set__z(0);
    rectangel.points.push_back(pose);

    publisher_polygon_->publish(rectangel);
  }


  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      publisher_polygon_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_polygon_;
  beacon::QuinticPolynomial qpx_;
  beacon::QuinticPolynomial qpy_;

  PiecewiseJerkProblem pjp_;
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

/*
在 ROS 2 中，`path.header.frame_id` 的作用是指定路径数据所属的坐标系（坐标框架）。具体来说，它是一个字符串，用来标识路径数据在什么坐标框架下进行定义和解释。理解这个字段的作用有助于确保在多坐标系环境中正确地解读和使用路径数据。

### 详细解释：

1. **坐标系定义**：
   - ROS 使用坐标系（frame）来描述机器人的位置、姿态以及感知数据等信息。不同的传感器数据和机器人状态信息可能处于不同的坐标系下。
   - 例如，激光扫描仪的数据可能在激光坐标系（`laser` frame）下，而机器人的全局位置可能在地图坐标系（`map` frame）下。

2. **header.frame_id 的作用**：
   - `path.header.frame_id` 指定了路径数据所参考的坐标系。例如，如果 `frame_id` 被设置为 `"map"`，则表示路径点的坐标是在全局地图坐标系下定义的。
   - 这样，使用该路径数据的其他节点就知道如何解释这些坐标，并可以进行必要的坐标变换，以便在不同的坐标系之间进行数据融合和转换。

3. **使用场景**：
   - 当路径数据从一个节点发布到另一个节点时，接收节点可以使用 `frame_id` 来确定如何将该路径与其他数据进行整合。
   - 在路径规划、导航、和传感器数据处理等应用中，正确设置和使用 `frame_id` 是至关重要的，因为它确保了所有数据的一致性和正确性。

### 示例代码

在你提供的代码示例中：
```cpp
auto path = nav_msgs::msg::Path();
path.header.stamp = this->now();
path.header.frame_id = "map";
```
- `path.header.stamp = this->now();`：为路径消息设置时间戳，这通常是消息生成的时间。
- `path.header.frame_id = "map";`：指定路径的坐标系为 `"map"`，表示路径点的坐标是在全局地图坐标系下定义的。

通过设置 `frame_id`，你确保了该路径数据可以被其他依赖于这个信息的节点正确地解读和使用。这在复杂的机器人系统中是至关重要的，因为不同的传感器和算法可能需要在不同的坐标系之间进行转换和对齐。
*/