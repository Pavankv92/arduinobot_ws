from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():

    # urdf_path = os.path.join(get_package_share_path('arduinobot_description'), 'urdf', 'arduinobot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('arduinobot_bringup'), 'rviz', 'arduinobot_config.rviz')

    # create a model path as launch argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), 'urdf', 'arduinobot.urdf.xacro'),
        description="Path to Robot URDF model"
    )

    # Param, LaunchConfiguration gets the value of the param
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]))

    robot_state_publisher_node =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node =  Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # -d : directory
    rviz2_node =  Node(
        package='rviz2',
        executable='rviz2',
        output="screen",
        arguments=['-d', rviz_config_path]

    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])