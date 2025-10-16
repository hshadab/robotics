from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('zkml_guard')
    config_path = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    guard_params_path = os.path.join(pkg_share, 'config', 'zkml_guard.params.yaml')

    camera_topic = LaunchConfiguration('camera_topic', default='/image')
    gating_mode = LaunchConfiguration('gating_mode', default='argmax')
    threshold = LaunchConfiguration('threshold', default='0.6')
    target_label = LaunchConfiguration('target_label', default='')
    target_label_id = LaunchConfiguration('target_label_id', default='-1')
    require_proof = LaunchConfiguration('require_proof', default='true')
    prove_on = LaunchConfiguration('prove_on', default='rising_edge')
    unlock_hold_ms = LaunchConfiguration('unlock_hold_ms', default='3000')
    reprove_on_label_change = LaunchConfiguration('reprove_on_label_change', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value=camera_topic),
        DeclareLaunchArgument('gating_mode', default_value=gating_mode),
        DeclareLaunchArgument('threshold', default_value=threshold),
        DeclareLaunchArgument('target_label', default_value=target_label),
        DeclareLaunchArgument('target_label_id', default_value=target_label_id),
        DeclareLaunchArgument('require_proof', default_value=require_proof),
        DeclareLaunchArgument('prove_on', default_value=prove_on),
        DeclareLaunchArgument('unlock_hold_ms', default_value=unlock_hold_ms),
        DeclareLaunchArgument('reprove_on_label_change', default_value=reprove_on_label_change),

        # twist_mux with lock on /zkml/stop
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[config_path],
        ),

        # ZKML guard
        Node(
            package='zkml_guard',
            executable='zkml_guard',
            name='zkml_guard',
            output='screen',
            parameters=[
                guard_params_path,
                {'camera_topic': camera_topic},
                {'gating_mode': gating_mode},
                {'threshold': threshold},
                {'target_label': target_label},
                {'target_label_id': target_label_id},
                {'require_proof': require_proof},
                {'prove_on': prove_on},
                {'unlock_hold_ms': unlock_hold_ms},
                {'reprove_on_label_change': reprove_on_label_change},
            ],
        ),
    ])
