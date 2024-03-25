# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_demos")

    pf_params_file = os.path.join(myDir, 'ros2-launchs', 'mrpt-pf.yaml')

    pointcloud_pipeline_file = os.path.join(
        myDir, 'ros2-launchs', 'point-cloud-pipeline.yaml')

    # Launch for pf_localization:
    mrpt_pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            # 'log_level_core': 'DEBUG',
            'topic_sensors_point_clouds': '/pf_input_points',
            'topic_gnns': '/gps',
            'pf_params_file': pf_params_file,
        }.items()
    )

    # Launch for MULRAN dataset:
    mulran_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            myDir, 'ros2-launchs', 'ros-mulran-play.launch.py')]),
        launch_arguments={
            # 'xxx': xxx,
        }.items()
    )

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'pipeline_yaml_file': pointcloud_pipeline_file,
            'points_topic_name': '/lidar_points',
            'filter_output_layer_name': 'output_for_pf',
            'filter_output_topic_name': '/pf_input_points',
            'time_window': '0.20',
            'show_gui': 'False',
            'one_observation_per_topic': 'True',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    return LaunchDescription([
        mrpt_pf_localization_launch,
        mulran_launch,
        pointcloud_pipeline_launch,
    ])
