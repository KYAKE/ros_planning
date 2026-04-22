import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("astar_demo")
    default_map = os.path.join(package_share, "maps", "warehouse", "warehouse.yaml")
    xacro_file = os.path.join(
        package_share, "urdf", "turtlebot3_waffle", "turtlebot3_waffle.xacro"
    )
    rviz_config = os.path.join(package_share, "rviz", "demo.rviz")
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    rosbridge_port_arg = DeclareLaunchArgument("rosbridge_port", default_value="9090")
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value="/camera/rgb/image_raw"
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/rgb/camera_info"
    )
    camera_width_arg = DeclareLaunchArgument("camera_width", default_value="320")
    camera_height_arg = DeclareLaunchArgument("camera_height", default_value="240")
    camera_publish_rate_arg = DeclareLaunchArgument("camera_publish_rate", default_value="5.0")

    robot_description = Command(["xacro ", xacro_file])

    demo_node = Node(
        package="astar_demo",
        executable="demo_server",
        name="astar_demo_server",
        output="screen",
        parameters=[
            {
                "map_yaml": LaunchConfiguration("map"),
                "camera_topic": LaunchConfiguration("camera_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_width": ParameterValue(LaunchConfiguration("camera_width"), value_type=int),
                "camera_height": ParameterValue(LaunchConfiguration("camera_height"), value_type=int),
                "camera_publish_rate": ParameterValue(
                    LaunchConfiguration("camera_publish_rate"), value_type=float
                ),
            }
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch),
        launch_arguments={
            "port": LaunchConfiguration("rosbridge_port"),
            "address": "0.0.0.0",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            map_arg,
            use_rviz_arg,
            rosbridge_port_arg,
            camera_topic_arg,
            camera_info_topic_arg,
            camera_width_arg,
            camera_height_arg,
            camera_publish_rate_arg,
            demo_node,
            robot_state_publisher,
            joint_state_publisher,
            rosbridge,
            rviz,
        ]
    )
