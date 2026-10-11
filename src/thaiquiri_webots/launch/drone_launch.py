import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer, SetRemap, SetParameter, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    use_rtabmap = LaunchConfiguration('rtabmap', default=True)

    package_dir = get_package_share_directory('thaiquiri_webots')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'thaiquiri.urdf')

    with open(robot_description_path, 'r') as desc_file:
        robot_description = desc_file.read()

    footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=[robot_description_path]
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )

    rtabmap_launch_path = get_package_share_directory('rtabmap_launch')
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_path, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            # 'use_sim_time': 'true',
            'rtabmap_args': '--delete_db_on_start',

            'stereo': 'true',
            'left_image_topic': '/drone/left/image_rect_color',
            'right_image_topic': '/drone/right/image_rect',
            'left_camera_info_topic': '/drone/left/camera_info',
            'right_camera_info_topic': '/drone/right/camera_info',

            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/drone/lidar_livox/point_cloud',

            'qos': '2',
            'qos_scan': '2',

            'visual_odometry': 'false',
            'icp_odometry': 'true',

            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            'approx_sync': 'True',
            'approx_sinc_max_interval': '0.05',
            'rviz': 'False',

            "queue_size": "20",
        }.items(),
        condition=IfCondition(use_rtabmap)
    )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'webots_world',
                           'worlds', 'mine_thaiquiri.wbt')
    )

    drone_driver = WebotsController(
        robot_name='drone',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    fix_camera_info = Node(
        package='thaiquiri_webots',
        executable='fix_camera_info',
        name='fix_camera_info',
        output='screen',
        namespace='drone',
    )

    bgra_to_bgr_node = Node(
        package='thaiquiri_webots',
        executable='bgra_to_bgr',
        name='bgra_to_bgr_node',
        output='screen',
        namespace='drone',
    )

    odom_to_ardupilot_node = Node(
        package='thaiquiri_webots',
        executable='odom_to_ardupilot',
        name='odom_to_ardupilot_node',
        output='screen',
        namespace='drone',
        parameters=[
            {'odom_topic': '/drone/odom'},
            {'tf_topic': '/ap/tf'},
            {'parent_frame': 'odom'},
            {'child_frame': 'base_footprint'},
            {'use_sim_time': True},
        ],
        remappings=[
            ('/clock', '/ap/clock')
        ],
    )

    gps_pose_publisher_node = Node(
        package='thaiquiri_webots',
        executable='gps_pose_publisher',
        name='gps_pose_publisher_node',
        output='screen',
        namespace='drone',
        parameters=[
            {'use_sim_time': True},
        ],
        remappings=[
            ('/clock', '/ap/clock')
        ],
    )

    imu_merger_node = Node(
        package='thaiquiri_webots',
        executable='imu_merger_node',
        name='imu_merger_node',
        output='screen',
        namespace='drone',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
                {'use_mag': False},  # Webots no suele tener magnetómetro por defecto
                # Si quieres que publique el frame transform
                {'publish_tf': False},
                {'world_frame': 'enu'}  # East-North-Up, el estándar de ROS
        ],
        remappings=[
            # Se suscribe al tópico que genera nuestro merger
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )

    stereo_proc_share = get_package_share_directory('stereo_image_proc')
    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            stereo_proc_share, 'launch', 'stereo_image_proc.launch.py')),
        launch_arguments={
            'namespace': 'drone',
            'approximate_sync': 'True',
            'approximate_sync_tolerance_seconds': '0.05',
            'disparity_only': 'True',
        }.items()
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=drone_driver,
        nodes_to_start=[rtabmap]
    )

    return LaunchDescription([
        webots,
        drone_driver,
        footprint,
        robot_state_publisher,
        fix_camera_info,
        bgra_to_bgr_node,
        # odom_to_ardupilot_node,
        # gps_pose_publisher_node,
        stereo_proc,
        # joint_state_publisher,
        imu_merger_node,
        imu_filter_node,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
