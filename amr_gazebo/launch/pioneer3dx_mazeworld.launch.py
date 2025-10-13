from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def launch_setup(context, *args, **kwargs):
    """Executed at runtime when the launch context exists."""

    # === Package paths ===
    desc_pkg = get_package_share_directory('amr_robots_description')
    sim_pkg = get_package_share_directory('amr_gazebo')

    # === Launch arguments ===
    urdf_gazebo_file = LaunchConfiguration('urdf_gazebo').perform(context)
    urdf_rviz_file = LaunchConfiguration('urdf_rviz').perform(context)
    world_file = LaunchConfiguration('world').perform(context)
    rviz_config_file = LaunchConfiguration('rviz_config').perform(context)

    # === Process the URDF/Xacro ===
    if urdf_gazebo_file.endswith('.xacro'):
        robot_gazebo_description = xacro.process_file(urdf_gazebo_file).toxml()
    else:
        with open(urdf_gazebo_file, 'r') as f:
            robot_gazebo_description = f.read()

    if urdf_rviz_file.endswith('.xacro'):
        robot_rviz_description = xacro.process_file(urdf_rviz_file).toxml()
    else:
        with open(urdf_rviz_file, 'r') as f:
            robot_rviz_description = f.read()


    # === Gazebo resource path ===
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{desc_pkg}:{os.environ.get('GZ_SIM_RESOURCE_PATH','')}"
    )

    # === Launch Gazebo Ignition ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # === Robot State Publisher ===
    robot_state_publisher_rviz = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_rviz',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_rviz_description},
        ],
    )

    # === Robot State Publisher ===
    robot_state_publisher_gazebo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_gazebo',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_gazebo_description},
        ],
    )

    # === Spawn robot in Gazebo ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'pioneer3dx', '-topic', 'robot_gazebo_description'],
        output='screen',
    )

    # === ROS–Gazebo bridge for cmd_vel ===
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_cmdvel',
        arguments=[
            '/model/pioneer3dx/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        output='screen',
    )

    # === RViz2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return [
        gz_resource_path,
        gazebo_launch,
        robot_state_publisher_rviz,
        robot_state_publisher_gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz_node,
    ]


def generate_launch_description():
    """Main entry point for launch."""
    desc_pkg = get_package_share_directory('amr_robots_description')
    sim_pkg = get_package_share_directory('amr_gazebo')

    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_pkg, 'worlds', 'maze.world'),
        description='Path to Gazebo Ignition world file',
    )

    urdf_gazebo_arg = DeclareLaunchArgument(
        'urdf_gazebo',
        default_value=os.path.join(desc_pkg, 'urdf', 'pioneer3dx_gazebo.urdf.xacro'),
        description='Path to robot URDF or Xacro file',
    )

    urdf_rviz_arg = DeclareLaunchArgument(
        'urdf_rviz',
        default_value=os.path.join(desc_pkg, 'urdf', 'pioneer3dx.urdf.xacro'),
        description='Path to robot URDF or Xacro file',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(sim_pkg, 'rviz', 'pioneer3dx_default.rviz'),
        description='Path to RViz configuration file',
    )

    return LaunchDescription([
        world_arg,
        urdf_gazebo_arg,
        urdf_rviz_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
