# 加载 urdf 文件，并在 rviz2 中显示机器人模型

# 1. 启动 robot_publisher 节点，该节点要以参数的方式加载 urdf 文件内容
# 2. 启动 rviz2 节点


# 优化： 1. 添加 joint_state_publisher 节点，当机器人有非固定关节时，必须包含该节点
#       2. 设置 rviz2 的默认配置文件 
#       3. 动态的传入 urdf 文件，把 urdf 封装为参数


from launch import LaunchDescription
from launch_ros.actions import Node

# 封装终端指令相关类
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

# 参数声明与获取
from launch.actions   import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 分组相关
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

# 事件相关
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo

from launch_ros.parameter_descriptions import ParameterValue
# 专门用来封装指令执行的
from launch.substitutions import Command

# 获取功能包下 share 目录路径
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  #error 这里如果不加 value_type=str 会报错
  # p_value = ParameterValue(Command(["xacro ", get_package_share_directory("urdf_demo")+"/urdf/urdf/beacon.urdf"]))
  robot_model_arg = DeclareLaunchArgument(
    name="robot_model", 
    default_value = get_package_share_directory("urdf_demo")+"/urdf/urdf/beacon.urdf",
    description='Path to the URDF file'
  )

  # 调用格式  ros2 launch urdf_demo  display.launch.py robot_model:=`ros2 pkg prefix --share urdf_demo`/urdf/urdf/beacon2.urdf

  dir_value = LaunchConfiguration('robot_model')
  print(str(dir_value))
  p_value = ParameterValue(
    Command(["xacro ", LaunchConfiguration('robot_model')]),
    value_type=str)

  robot_state_pub = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description":p_value}],
    output='screen'
  )


  joint_state_pub = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    output='screen'
  )
  

  rviz_value = get_package_share_directory("urdf_demo")+"/rviz/urdf.rviz"
  rviz2 = Node(
    package="rviz2", 
    executable="rviz2",
    arguments=["-d", rviz_value],
    output='screen'
    )

  return LaunchDescription([robot_model_arg,robot_state_pub, joint_state_pub, rviz2])

