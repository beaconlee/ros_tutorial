cmake_minimum_required(VERSION 3.8)
project(rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找系统中的依赖项(库)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# 生成可执行的二进制文件
add_executable(rviz_tutorial
               src/rviz_tutorial.cpp
               src/quintic_polynomial.cc
               src/piecewise_jerk_problem.cc)


target_include_directories(rviz_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(rviz_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

LINK_DIRECTORIES("/usr/local/lib")

# 链接库
# 这里去连接外部库，之前这里写的是：OsqpEigen，然后失败了
# 写 OsqpEigen::OsqpEigen 就编译成功了
target_link_libraries(rviz_tutorial
  OsqpEigen::OsqpEigen
)

# 依赖于其他的目标文件，确保其他目标已经被构建 
# ament_target_dependencies(nodeName packageName)
ament_target_dependencies(
  rviz_tutorial
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "visualization_msgs"
  "osqp"
  "OsqpEigen"
)

# 生成可安装文件
# 该语句的最终目的就是让编译器安装该节点，使得节点可以运行，参数只需要动中间的，
install(TARGETS rviz_tutorial
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成功能包
ament_package()


#自定义消息类型接口
# rosidl_generate_interfaces()   