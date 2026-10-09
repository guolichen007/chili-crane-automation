"""mock_system: installed config with an explicit override."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("chili_crane_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("crane_id", default_value="crane_01"),
        DeclareLaunchArgument("config_root", default_value=share + "/config"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("physical_output_enabled", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + "/launch/base_system.launch.py"),
            launch_arguments={
                "crane_id": LaunchConfiguration("crane_id"),
                "config_root": LaunchConfiguration("config_root"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "physical_output_enabled": LaunchConfiguration("physical_output_enabled"),
                "runtime_mode": "localization",
                "entry_mode": "mock",
            }.items()),
    ])
