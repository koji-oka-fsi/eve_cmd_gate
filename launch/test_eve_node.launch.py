
# eve_cmd_gate/launch/bringup_ad_managers.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    moved_threshold = LaunchConfiguration('moved_threshold')
    timer_hz = LaunchConfiguration('timer_hz')
    use_overridable_vehicle = LaunchConfiguration('use_overridable_vehicle')

    container = ComposableNodeContainer(
        name='ad_managers_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ad_sound_manager',
                plugin='ad_sound_manager::AdSoundManager',
                name='ad_sound_manager',
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            ComposableNode(
                package='ad_status_lamp_manager',
                plugin='ad_status_lamp_manager::AdStatusLampManager',
                name='ad_status_lamp_manager',
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            ComposableNode(
                package='in_parking_state_manager',
                plugin='in_parking_state_manager::InParkingStateManager',
                name='in_parking_state_manager',
                parameters=[{'moved_threshold': moved_threshold, 'timer_hz': timer_hz}],
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            ComposableNode(
                package='warning_lamp_manager',
                plugin='warning_lamp_manager::WarningLampManager',
                name='warning_lamp_manager',
                parameters=[{'use_overridable_vehicle': use_overridable_vehicle}],
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('moved_threshold', default_value='1.0'),
        DeclareLaunchArgument('timer_hz', default_value='10.0'),
        DeclareLaunchArgument('use_overridable_vehicle', default_value='true'),
        container
    ])
