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
    MAPPING launch file — FastLIO2 + octomap_server
    =================================================
    Pipeline:
      Gazebo sensors
          ↓
      ros_gz_bridge  (/lidar_front/points + /imu/data)
          ↓
      FastLIO2  →  /Odometry (odom→base_link TF)
               →  /cloud_registered (world-frame 3D cloud)
          ↓
      octomap_server  →  /projected_map (2D occupancy grid)
                      →  /octomap_full  (3D map)
          ↓
      RViz2 (visualise map building)

    Save map when done:
        # Save 2D map for Nav2
        ros2 run nav2_map_server map_saver_cli -f ~/my_map \
            --ros-args -p map_topic:=/projected_map

        # Save 3D PCD map (set pcd_save_en: true in m20_fastlio.yaml first)
        # FastLIO2 saves automatically on shutdown

    Requires:
        bash ~/Downloads/install_fastlio.sh   (run once)
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    pkg_lynx       = get_package_share_directory('lynx_quanta')
    pkg_fastlio    = get_package_share_directory('spark_fast_lio')

    urdf           = os.path.join(pkg_lynx, 'urdf', 'm20_with_arm', 'm20_with_arm_v2.urdf')
    ctrl_yaml      = os.path.join(pkg_lynx, 'config', 'm20_with_arm_controller.yaml')
    fastlio_config = os.path.join(pkg_lynx, 'config', 'm20_fastlio.yaml')
    octomap_config = os.path.join(pkg_lynx, 'config', 'octomap.yaml')
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

    # ── FastLIO2 ──────────────────────────────────────────────────────────────
    # Subscribes to: /lidar_front/points + /imu/data
    # Publishes:
    #   /Odometry           → robot pose in world frame
    #   /cloud_registered   → accumulated 3D map cloud
    #   /path               → trajectory
    # Also broadcasts TF:  camera_init → body (remapped to map → base_link)
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
        # spark-fast-lio subscribes to /lidar and /imu by default.
        # Remap to our actual Gazebo bridge topics.
        remappings=[
            ('/lidar', '/lidar_merged/points'),  # merged front+rear
            ('/imu',   '/imu/data'),
        ],
    )

    # ── Static TF: map → odom ────────────────────────────────────────────────
    # FastLIO2 publishes: map → body  (world frame = map, robot frame = body)
    # Gazebo publishes:   odom → base_footprint → base_link → [joints]
    #
    # These are TWO disconnected trees. Connecting map → odom bridges them:
    #   map → odom → base_footprint → base_link → [joints]   (for RViz + Nav2)
    #   map → body                                            (FastLIO2 tracking)
    #
    # body and base_link are close but not identical — body is the LiDAR-inertial
    # estimate; base_link is the Gazebo physics body. Acceptable for simulation.
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── octomap_server ────────────────────────────────────────────────────────
    # Converts /cloud_registered (3D) → /projected_map (2D occupancy grid)
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            octomap_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/cloud_registered'),
        ],
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
    ld.add_action(fastlio)            # LiDAR-inertial odometry + mapping
    ld.add_action(octomap)            # 3D cloud → 2D occupancy grid
    ld.add_action(rviz)
    return ld