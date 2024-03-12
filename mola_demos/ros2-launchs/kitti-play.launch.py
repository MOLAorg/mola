
# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_demos")

    # args that can be set from the command line or a default will be used
    kitti_sequence_arg = DeclareLaunchArgument(
        "kitti_sequence", default_value="00")

    set_seq_env_var = SetEnvironmentVariable(
        name='KITTI_SEQ', value=LaunchConfiguration('kitti_sequence'))

    mola_cli_node = Node(
        package='mola_launcher',
        executable='mola-cli',
        output='screen',
        arguments=[
                os.path.join(myDir, 'mola-cli-launchs', 'kitti_just_replay_to_ros2.yaml')]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
                '-d', [os.path.join(myDir, 'rviz2', 'kitti.rviz')]]
    )

    return LaunchDescription([
        kitti_sequence_arg,
        set_seq_env_var,
        mola_cli_node,
        rviz2_node
    ])
