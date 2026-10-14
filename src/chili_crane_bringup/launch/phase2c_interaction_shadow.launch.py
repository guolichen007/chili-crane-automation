"""Evidence only; no TaskCycles replacement, controller or hardware writers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=get_package_share_directory('chili_crane_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='crane_01'),
        DeclareLaunchArgument('config_file',default_value=share+'/config/algorithms/phase2c_interaction.yaml'),
        Node(package='chili_crane_control',executable='grasp_interaction_shadow_node.py',
             namespace=LaunchConfiguration('namespace'),parameters=[{'config_file':LaunchConfiguration('config_file')}]),
    ])
