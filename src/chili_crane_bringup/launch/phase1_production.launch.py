from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share + "/launch/phase1_runtime.launch.py"),
            launch_arguments={"config_root": LaunchConfiguration("config_root"), "profile": "production"}.items()),
    ])
