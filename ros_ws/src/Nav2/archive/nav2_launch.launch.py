from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # params_file = LaunchConfiguration('params_file')
    params_file = '/workspace/nav2_params.yaml'
    autostart = LaunchConfiguration('autostart')

    use_respawn = LaunchConfiguration('use_respawn')
    # Nav2 BT XML for FollowPath.
    # If this file name differs on your install, adjust it after checking the folder:
    # `ros2 pkg prefix nav2_bt_navigator` then look in share/nav2_bt_navigator/behavior_trees
    #follow_path_bt_xml = PathJoinSubstitution([
    #    FindPackageShare('nav2_bt_navigator'),
    #    'behavior_trees',
    #    'follow_path.xml',
    #])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'params_file',
        #     default_value=PathJoinSubstitution([
        #         FindPackageShare('my_nav2_config'),
        #         'config',
        #         'nav2_local_followpath.yaml'
        #     ]),
        #     description='Full path to the Nav2 parameters file'
        # ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 lifecycle nodes'
        ),

        # Local costmap server (rolling window voxel+inflation)
#        Node(
#            package='nav2_costmap_2d',
#            executable='costmap_server',
#            name='local_costmap',
#            output='screen',
#            parameters=[params_file],
#        ),
#
     Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Controller server (RPP FollowPath)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        # Behavior server (DriveOnHeading / Wait, etc)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        # BT Navigator (exposes /follow_path action if BT XML supports it)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {'default_bt_xml_filename': follow_path_bt_xml},
            ],
        ), 
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        # Lifecycle manager to bring everything up
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': autostart,
                'node_names': [
                    'local_costmap',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }],
        ),
    ])
