"""
glim_mapping.launch.py — Lynx M20 + GLIM 3-D LiDAR-inertial mapping
=====================================================================

Root-cause fixes applied
------------------------
1.  GLIM crash (InvalidTopicNameError):
      glim_ros/image_topic defaults to "" which GLIM tries to subscribe to,
      producing an empty-topic exception.  Fixed by supplying a real topic name
      in config_ros.json.

2.  Config files not being read by GLIM:
      After copying updated files to src/, rebuild with --symlink-install so
      the install/ tree picks them up.

3.  RViz "Frame [map] does not exist":
      GLIM must be running to publish the map→odom TF.  RViz default fixed
      frame is now "odom" so the robot is visible immediately; it switches to
      "map" automatically once GLIM publishes that frame.

TF tree
-------
  map ──(GLIM)──► odom ──(GLIM/TF)──► base_footprint ──► base_link ──► all joints

Deploy steps
------------
  1.  Copy all files from outputs/ into your source tree:
        cp -r <outputs>/config/glim/*  ~/lynx_ws/src/lynx_quanta/config/glim/
        cp <outputs>/launch/glim_mapping.launch.py  ~/lynx_ws/src/lynx_quanta/launch/
  2.  Rebuild:
        cd ~/lynx_ws && colcon build --symlink-install --packages-select lynx_quanta
        source install/setup.bash
  3.  Launch:
        ros2 launch lynx_quanta glim_mapping.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_lynx      = get_package_share_directory('lynx_quanta')
    urdf          = os.path.join(pkg_lynx, 'urdf', 'm20_with_arm', 'm20_with_piper_v3.urdf')
    ctrl_yaml     = os.path.join(pkg_lynx, 'config', 'm20_with_piper_controller.yaml')
    ekf_yaml      = os.path.join(pkg_lynx, 'config', 'ekf.yaml')
    glim_cfg_path = os.path.join(pkg_lynx, 'config', 'glim')

    # glim_ros ships its own RViz config; use it if present
    try:
        pkg_glim_ros = get_package_share_directory('glim_ros')
        rviz_cfg = os.path.join(pkg_glim_ros, 'rviz', 'glim_ros.rviz')
        if not os.path.exists(rviz_cfg):
            rviz_cfg = ''
    except Exception:
        rviz_cfg = ''

    robot_desc = ParameterValue(
        Command(['xacro ', urdf, ' ', 'ros2_control_yaml:=', ctrl_yaml]),
        value_type=str)

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    gz_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description',
                   '-name',  'm20_with_arm', '-allow_renaming', 'true',
                   '-x', '0.0', '-y', '0.0', '-z', '0.6'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
            'ignore_timestamp':  True,
        }],
    )

    # ── 3. Gazebo ↔ ROS 2 bridge ──────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gz_ros2_bridge', output='screen',
        arguments=[
            # Command velocity must remain bidirectional.
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # Simulation time.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Sensors required for GLIM front-LiDAR mapping.
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Rear LiDAR is bridged for inspection only; GLIM currently does not use it.
            # '/lidar_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Camera image only. Depth images and camera point clouds are not needed for GLIM mapping.
            '/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/camera_rear/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera_rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 4. ros2_control spawners ──────────────────────────────────────────────
    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name],
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        )

    # ── 5. Dual LiDAR merger → /lidar_merged_points ───────────────────────────
    lidar_merger_node = Node(
        package='lynx_quanta', executable='lidar_merger', name='lidar_merger',
        output='screen',
        parameters=[{
            'use_sim_time':       use_sim_time,
            'target_frame':       'base_link',
            'front_topic':        '/lidar_front/points',
            'rear_topic':         '/lidar_rear/points',
            'merged_cloud_topic': '/lidar_merged_points',
            'merged_scan_topic':  '/scan',
            'min_height':         -0.05,
            'max_height':         2.0,
            'angle_min':          -3.14159,
            'angle_max':           3.14159,
            'angle_increment':     0.00349,
            'range_min':           0.15,
            'range_max':           50.0,
            'publish_rate':        10.0,
        }],
    )

    # ── 6. EKF — wheel odometry + IMU ─────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
    )

    # ── 7. GLIM — 3-D LiDAR-inertial mapping / localisation ──────────────────
    #
    #  Critical settings for Gazebo simulation:
    #
    #  a) use_sim_time=true  — passed as a ROS param so GLIM uses /clock.
    #
    #  b) config_sensors.json → global_shutter_lidar=true
    #     Gazebo's virtual LiDAR doesn't embed per-point timestamps.
    #     This tells GLIM to use the PointCloud2 header.stamp instead of
    #     trying to read per-point time fields (fixes "points=3.5 frozen").
    #
    #  c) config_ros.json → image_topic="/camera_front/image"
    #     GLIM crashes with InvalidTopicNameError if image_topic is an
    #     empty string (the built-in default).  A valid topic name avoids
    #     the exception even if no camera subscriber is active.
    #
    glim_node = Node(
        package='glim_ros', executable='glim_rosnode',
        name='glim_rosnode', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_path':  glim_cfg_path},
        ],
    )

    # ── 8. RViz2 ──────────────────────────────────────────────────────────────
    #  Default fixed frame is "odom" so the robot is visible immediately.
    #  Change to "map" in RViz once GLIM starts publishing that frame.
    rviz_args = ['-d', rviz_cfg] if rviz_cfg and os.path.exists(rviz_cfg) else []
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 9. Lynx Brain ─────────────────────────────────────────────────────────
    brain = Node(
        package='lynx_quanta', executable='lynx_brain', name='lynx_brain',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    ld.add_action(gz_sim)
    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)
    ld.add_action(rsp)

    ld.add_action(spawner('joint_state_broadcaster'))
    ld.add_action(spawner('leg_pose_controller'))
    ld.add_action(spawner('wheel_velocity_controller'))
    ld.add_action(spawner('arm_controller'))
    ld.add_action(spawner('gripper_controller'))

    # ld.add_action(lidar_merger_node)
    # ld.add_action(ekf_node)

    ld.add_action(glim_node)
    ld.add_action(rviz_node)
    ld.add_action(brain)

    return ld
