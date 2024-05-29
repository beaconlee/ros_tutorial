在ROS2中使用C++发布栅格地图并在RViz中显示，需要创建一个C++节点发布OccupancyGrid消息。以下是详细步骤：

### 1. 安装必要的软件包
确保你已经安装了ROS2和RViz。你可以使用以下命令来安装RViz：

```bash
sudo apt update
sudo apt install ros-<ros2-distro>-rviz2
```
请将`<ros2-distro>`替换为你的ROS2版本，比如`foxy`、`galactic`或`humble`。

### 2. 创建ROS2包
创建一个新的ROS2包来发布栅格地图。你可以使用以下命令：

```bash
ros2 pkg create --build-type ament_cmake grid_map_publisher
```

### 3. 编写节点发布栅格地图
在包中创建一个C++源文件来发布栅格地图。在`src`目录下创建一个名为`grid_map_publisher.cpp`的文件，并添加以下代码：

```cpp
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
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
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
        for (auto& cell : msg.data) {
            cell = rand() % 100;  // 随机填充地图数据
        }

        RCLCPP_INFO(this->get_logger(), "Publishing grid map");
        publisher_->publish(msg);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### 4. 设置CMakeLists.txt
编辑`CMakeLists.txt`以包含必要的构建指令。确保它看起来像这样：

```cmake
cmake_minimum_required(VERSION 3.5)
project(grid_map_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(grid_map_publisher src/grid_map_publisher.cpp)
ament_target_dependencies(grid_map_publisher rclcpp nav_msgs)

install(TARGETS
  grid_map_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### 5. 构建并运行包
构建你的ROS2包并运行：

```bash
colcon build
source install/setup.bash
ros2 run grid_map_publisher grid_map_publisher
```

### 6. 在RViz中显示栅格地图
启动RViz并添加一个新的显示类型`Map`。在`Topic`字段中输入`/grid_map`。你应该能够看到RViz中显示的栅格地图。

```bash
rviz2
```

在RViz中：
- 添加一个`Map`显示插件。
- 设置`Topic`为`/grid_map`。
- 设置`Frame`为`map`。

按照以上步骤，你应该能够在RViz中显示由ROS2节点发布的栅格地图。如果遇到任何问题，请检查节点日志，并确保所有配置正确。

# 如何将这个地图更改为比较清晰具体的地图？比如地方上有几个矩形，有一些道路

为了生成一个更具体和清晰的地图，我们可以手动定义地图的数据，而不是随机生成。下面是一个示例，展示如何在栅格地图中定义一些矩形区域和道路：

### 1. 修改 `grid_map_publisher.cpp`
我们将更新C++代码来手动定义一个包含矩形和道路的栅格地图。

```cpp
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
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
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

        // Initialize map data
        msg.data.resize(msg.info.width * msg.info.height, 0);

        // Draw some rectangles
        draw_rectangle(msg.data, msg.info.width, 20, 20, 30, 30, 100);
        draw_rectangle(msg.data, msg.info.width, 50, 50, 70, 70, 100);

        // Draw a road
        draw_road(msg.data, msg.info.width, 10, 80, 40, 50, 0);

        RCLCPP_INFO(this->get_logger(), "Publishing grid map");
        publisher_->publish(msg);
    }

    void draw_rectangle(std::vector<int8_t>& data, int width, int x0, int y0, int x1, int y1, int value)
    {
        for (int y = y0; y <= y1; ++y)
        {
            for (int x = x0; x <= x1; ++x)
            {
                data[y * width + x] = value;
            }
        }
    }

    void draw_road(std::vector<int8_t>& data, int width, int x0, int x1, int y0, int y1, int value)
    {
        for (int y = y0; y <= y1; ++y)
        {
            for (int x = x0; x <= x1; ++x)
            {
                data[y * width + x] = value;
            }
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### 2. 重新构建并运行包
构建你的ROS2包并运行：

```bash
colcon build
source install/setup.bash
ros2 run grid_map_publisher grid_map_publisher
```

### 3. 在RViz中显示栅格地图
启动RViz并添加一个新的显示类型`Map`。在`Topic`字段中输入`/grid_map`。你应该能够看到RViz中显示的具体地图。

```bash
rviz2
```

在RViz中：
- 添加一个`Map`显示插件。
- 设置`Topic`为`/grid_map`。
- 设置`Frame`为`map`。

### 解释
- `draw_rectangle` 函数用于在地图上绘制一个矩形。你可以指定矩形的左上角和右下角的坐标，以及矩形的值（这里设置为100表示完全占用）。
- `draw_road` 函数用于在地图上绘制一条道路。你可以指定道路的左上角和右下角的坐标，以及道路的值（这里设置为0表示空闲）。

通过这种方式，你可以生成一个更具体和清晰的栅格地图，在RViz中显示矩形建筑和道路。你可以根据需要调整矩形和道路的坐标，以生成你需要的地图布局。


# ERROR
uan@beacon:~/workspace/ros_tutorial$ rviz2
qt.qpa.plugin: Could not find the Qt platform plugin "wayland" in ""
[INFO] [1716955018.239446947] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [1716955018.239539252] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[INFO] [1716955018.250581203] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [1716955026.853962874] [rviz2]: Trying to create a map of size 100 x 100 using 1 swatches
[ERROR] [1716955026.885782940] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
active samplers with a different type refer to the same texture image unit



这个错误通常是由于在RViz中加载的着色器程序不兼容或存在冲突。我们可以尝试通过以下方法来解决这个问题：

### 1. 更新系统和软件包
首先，确保系统和所有相关软件包是最新的：

```bash
sudo apt update
sudo apt upgrade
```

### 2. 检查图形驱动程序
确保你使用的是最新的图形驱动程序。如果你使用的是NVIDIA显卡，可以使用以下命令更新驱动：

```bash
sudo ubuntu-drivers autoinstall
```

### 3. 使用X11而非Wayland
错误信息中提到了`Qt platform plugin "wayland"`，你可以尝试使用X11而不是Wayland来启动RViz。你可以在登录界面选择`Ubuntu on Xorg`，然后重新启动RViz。

### 4. 修改RViz配置文件
你可以尝试通过修改RViz配置文件来避免着色器冲突。找到RViz配置文件（通常位于`~/.rviz2/default.rviz`），并尝试重置或修改其中的设置。

### 5. 手动指定Qt平台
尝试手动指定使用X11而不是Wayland：

```bash
export QT_QPA_PLATFORM=xcb
rviz2
```

### 6. 确保正确的OpenGL版本
确保系统支持并正确配置了OpenGL。你可以使用以下命令查看OpenGL版本：

```bash
glxinfo | grep "OpenGL version"
```

### 7. 安装mesa-utils（如适用）
如果未安装mesa-utils，可以通过以下命令安装：

```bash
sudo apt install mesa-utils
```

### 8. 检查RViz日志
查看RViz的详细日志，以获取更多信息。日志文件通常位于`~/.ros/log`目录下。

### 9. 调整栅格地图生成代码
虽然这不太可能是引起GLSL错误的原因，但为了确保发布的栅格地图数据没有问题，可以再检查一下代码是否有误。

### 示例：`grid_map_publisher.cpp`

确保生成的地图数据是正确的。例如：

```cpp
void draw_rectangle(std::vector<int8_t>& data, int width, int x0, int y0, int x1, int y1, int value)
{
    for (int y = y0; y <= y1; ++y)
    {
        for (int x = x0; x <= x1; ++x)
        {
            if (x >= 0 && x < width && y >= 0 && y < width) // Check bounds
            {
                data[y * width + x] = value;
            }
        }
    }
}

void draw_road(std::vector<int8_t>& data, int width, int x0, int x1, int y0, int y1, int value)
{
    for (int y = y0; y <= y1; ++y)
    {
        for (int x = x0; x <= x1; ++x)
        {
            if (x >= 0 && x < width && y >= 0 && y < width) // Check bounds
            {
                data[y * width + x] = value;
            }
        }
    }
}
```

### 总结
以上方法应该能够帮助解决RViz2启动时的着色器错误问题。如果问题仍然存在，请提供更多详细信息，例如系统版本、ROS2发行版版本等，以便进一步排查。





# 发布事件触发 node

在 ROS 2 中使用 C++ 创建一个节点，该节点只有在特定事件触发时才发布消息，而不是定时发布消息，可以通过订阅和发布的机制来实现。下面是一个详细的示例代码：

首先，确保你已经安装了 ROS 2，并创建了一个工作空间。然后，在工作空间中创建一个新的包：

```sh
ros2 pkg create --build-type ament_cmake event_trigger_node
```

在 `src` 目录下创建一个新的 C++ 文件，例如 `event_trigger_node.cpp`：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EventTriggerNode : public rclcpp::Node
{
public:
    EventTriggerNode() : Node("event_trigger_node")
    {
        // 创建订阅者，监听 /trigger_event 主题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/trigger_event", 10,
            std::bind(&EventTriggerNode::trigger_callback, this, std::placeholders::_1));

        // 创建发布者，用于发布消息到 /output 主题
        publisher_ = this->create_publisher<std_msgs::msg::String>("/output", 10);
    }

private:
    void trigger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received trigger event: '%s'", msg->data.c_str());

        // 创建要发布的消息
        auto output_msg = std::make_shared<std_msgs::msg::String>();
        output_msg->data = "Triggered message!";

        // 发布消息
        publisher_->publish(*output_msg);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", output_msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
```

接下来，修改 `CMakeLists.txt` 文件以确保脚本可以正确编译和运行：

```cmake
cmake_minimum_required(VERSION 3.5)
project(event_trigger_node)

# 找到依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(event_trigger_node src/event_trigger_node.cpp)

# 链接依赖库
ament_target_dependencies(event_trigger_node rclcpp std_msgs)

# 安装可执行文件
install(TARGETS
  event_trigger_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

现在你可以构建并运行这个节点：

```sh
colcon build
source install/setup.bash
ros2 run event_trigger_node event_trigger_node
```

此时，你可以通过发布消息到 `/trigger_event` 主题来触发该节点发布消息。例如，可以使用 `ros2 topic pub` 命令来测试：

```sh
ros2 topic pub /trigger_event std_msgs/msg/String "data: 'Hello'"
```

这个示例展示了如何在事件触发时发布消息，而不是定时发布。你可以根据实际应用的需求调整订阅和发布的逻辑。