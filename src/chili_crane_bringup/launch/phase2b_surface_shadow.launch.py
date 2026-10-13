"""Phase2B perception/proposal only. No hardware, task, safety or actuation nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=get_package_share_directory('chili_crane_bringup')
    params={'config_file':LaunchConfiguration('config_file'),
            'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=share+'/config/perception/phase2b_surface.template.yaml'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='chili_crane_perception',executable='pit_surface_shadow_node.py',namespace='crane_01',parameters=[params]),
        Node(package='chili_crane_perception',executable='grasp_proposal_shadow_node.py',namespace='crane_01',parameters=[params]),
    ])
