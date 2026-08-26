import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Directories
    bringup_dir = '/ros2_ws/src/flatland_nav2_bringup'

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch rviz2')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    world_path_arg = DeclareLaunchArgument(
        'world_path', default_value='/ros2_ws/worlds/sample.world.yaml',
        description='Path to flatland world file')
    map_path_arg = DeclareLaunchArgument(
        'map_path', default_value='/ros2_ws/maps/sample_map.yaml',
        description='Path to map yaml file')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 parameters file')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world_path')
    map_path = LaunchConfiguration('map_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # Rewrite params to inject use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    # Flatland server node
    flatland_server = Node(
        package='flatland_server',
        executable='flatland_server',
        name='flatland_server',
        output='screen',
        parameters=[{
            'world_path': world_path,
            # 50 Hz physics is plenty for a diff-drive robot (nav2 controller runs at
            # 20 Hz). The world rate also sets the /clock publish rate, and every
            # use_sim_time node wakes per /clock message — at 200 Hz the whole nav2
            # stack burned CPU just processing time updates.
            'update_rate': 50.0,
            'step_size': 0.02,
            'show_viz': 0.0,
            'viz_pub_rate': 30.0,
            'use_sim_time': False,  # flatland is the time source
        }],
    )

    # Map server (lifecycle node)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_path}],
    )

    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
    )

    # Lifecycle manager for map_server and amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # --- Nav2 navigation nodes (launched directly, without docking_server) ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
    )

    # Republishes selected simulation data (battery, ETA) to /inorbit/custom_data
    republisher = Node(
        package='republisher',
        executable='republisher',
        name='republisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Diagnostics watcher node — publishes /diagnostics for sim health.
    # use_sim_time is intentionally left off: the watcher's freshness and
    # presence checks only need wall-clock time, and subscribing to /clock
    # at the sim step rate is the dominant CPU cost for this node.
    diagnostics_watcher = Node(
        package='diagnostics_watcher',
        executable='watcher',
        name='diagnostics_watcher',
        output='screen',
    )

    # Aggregator — republishes /diagnostics_agg grouped by Sensors/Power/Navigation
    diagnostics_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='analyzers',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'config', 'diagnostics_aggregator.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # rosbridge websocket on :9090 — used by the ISO 21423 agent sidecar (iso-agent/)
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'port': 9090}],
    )

    # Lifecycle manager for navigation nodes
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    # Rviz2 (conditional)
    rviz_config = os.path.join(bringup_dir, 'config', 'flatland_rviz.rviz')
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        world_path_arg,
        map_path_arg,
        nav2_params_arg,
        flatland_server,
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        smoother_server,
        lifecycle_manager_navigation,
        republisher,
        diagnostics_watcher,
        diagnostics_aggregator,
        rosbridge,
        rviz_node,
    ])
