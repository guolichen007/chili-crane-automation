"""Full B/C evidence stack. Optional console is a read-only monitor by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=get_package_share_directory('chili_crane_bringup')
    namespace=LaunchConfiguration('namespace')
    surface=LaunchConfiguration('surface_config_file')
    interaction=LaunchConfiguration('interaction_config_file')
    clock={'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='crane_01'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('surface_config_file',default_value=share+'/config/perception/phase2b_surface.template.yaml'),
        DeclareLaunchArgument('interaction_config_file',default_value=share+'/config/algorithms/phase2c_interaction.yaml'),
        DeclareLaunchArgument('commissioning',default_value='false'),
        DeclareLaunchArgument('commissioning_config_file',default_value=share+'/config/commissioning/console.yaml'),
        DeclareLaunchArgument('git_sha',default_value='NOT_CONFIGURED'),
        DeclareLaunchArgument('runs_directory',default_value='runs'),
        DeclareLaunchArgument('commissioning_port',default_value='8080'),
        DeclareLaunchArgument('commissioning_run_id',default_value=''),
        DeclareLaunchArgument('recording_config_file',default_value=share+'/config/recording/rosbag2_profile.yaml'),
        Node(package='chili_crane_perception',executable='pit_surface_shadow_node.py',namespace=namespace,
             output='screen',
             parameters=[dict(clock,config_file=surface,interaction_config_file=interaction)]),
        Node(package='chili_crane_perception',executable='grasp_proposal_shadow_node.py',namespace=namespace,
             output='screen',
             parameters=[dict(clock,config_file=surface)]),
        Node(package='chili_crane_control',executable='grasp_interaction_shadow_node.py',namespace=namespace,
             output='screen',
             parameters=[dict(clock,config_file=interaction)]),
        Node(package='chili_crane_commissioning',executable='commissioning_console.py',namespace=namespace,
             output='screen',
             condition=IfCondition(LaunchConfiguration('commissioning')),
             arguments=['--config',LaunchConfiguration('commissioning_config_file'),'--git-sha',LaunchConfiguration('git_sha'),
                        '--run-id',LaunchConfiguration('commissioning_run_id'),
                        '--runs-directory',LaunchConfiguration('runs_directory'),'--crane-namespace',namespace,
                        '--recording-config',LaunchConfiguration('recording_config_file'),'--host','127.0.0.1',
                        '--port',LaunchConfiguration('commissioning_port')],
             parameters=[clock]),
    ])
