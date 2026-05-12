import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    NAVIGATION launch file — FastLIO2 + Nav2 MPPI
    ===============================================
    Uses FastLIO2 for real-time localisation against saved map.
    Nav2 handles path planning and control.

    Launch command:
        ros2 launch lynx_quanta robot_fastlio.launch.py \
            map:=/home/sutd/lynx_ws/src/lynx_quanta/maps/my_map.yaml

    Prerequisites:
        1. Run fastlio_mapping.launch.py to build the map
        2. Save map:
               ros2 run nav2_map_server map_saver_cli -f ~/my_map \
                   --ros-args -p map_topic:=/projected_map
           Copy my_map.pgm + my_map.yaml → lynx_quanta/maps/
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('lynx_quanta'), 'maps', 'my_map.yaml'),
        description='Path to saved occupancy grid .yaml')

    pkg_lynx       = get_package_share_directory('lynx_quanta')
    urdf           = os.path.join(pkg_lynx, 'urdf', 'm20_with_arm', 'm20_with_arm_v2.urdf')
    ctrl_yaml      = os.path.join(pkg_lynx, 'config', 'm20_with_arm_controller.yaml')
    fastlio_config = os.path.join(pkg_lynx, 'config', 'm20_fastlio.yaml')
    nav2_params    = os.path.join(pkg_lynx, 'config', 'nav2_fastlio_params.yaml')
    rviz_config    = os.path.join(pkg_lynx, 'config', 'm20_sensors.rviz')

    robot_desc = ParameterValue(
        Command(['xacro ', urdf, ' ', 'ros2_control_yaml:=', ctrl_yaml]),
        value_type=str)

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    gz_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'm20_with_arm', '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.6',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Bridge ────────────────────────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gz_ros2_bridge', output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_rear/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_rear/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_desc,
            'publish_frequency': 30.0,
            'ignore_timestamp':  True,
        }],
    )

    # ── ros2_control spawners ─────────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
    )
    leg_pose_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['leg_pose_controller'],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
    )
    wheel_velocity_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_velocity_controller'], output='screen',
    )

    brain = Node(
        package='lynx_quanta', executable='lynx_brain', name='lynx_brain',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── FastLIO2 (localization mode) ──────────────────────────────────────────
    # In nav mode: FastLIO2 still runs in real-time mapping mode
    # but the pre-built map is served separately by Nav2 map_server.
    # FastLIO2 provides the odom→base_link TF continuously.
    # ── Dual LiDAR merger ────────────────────────────────────────────────────
    # Transforms /lidar_rear/points into lidar_front_link frame and
    # concatenates with /lidar_front/points → /lidar_merged/points
    # This gives full 360° coverage matching the real M20 dual-LiDAR setup.
    lidar_merger = Node(
        package='lynx_quanta',
        executable='lidar_merger',
        name='lidar_merger',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'target_frame':  'lidar_front_link',
        }],
    )

    fastlio = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fastlio_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/lidar', '/lidar_merged/points'),  # merged front+rear
            ('/imu',   '/imu/data'),
        ],
    )

    # ── Frame aliases for FastLIO2 ────────────────────────────────────────────
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_init_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────────
    # Uses navigation_launch.py (no AMCL) — FastLIO2 provides localisation TF
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map':          map_yaml,
            'params_file':  nav2_params,
        }.items()
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)
    ld.add_action(rsp)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(leg_pose_controller)
    ld.add_action(wheel_velocity_controller)
    ld.add_action(brain)
    ld.add_action(static_tf_map_odom) # map → odom (connects FastLIO2 + Gazebo trees)
    ld.add_action(lidar_merger)       # merge front+rear LiDAR clouds
    ld.add_action(fastlio)
    ld.add_action(nav2)
    ld.add_action(rviz)
    return ld