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

    desc_pkg = get_package_share_directory('amr_robots_description')
    sim_pkg = get_package_share_directory('amr_gazebo')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    # === Resolve paths from launch arguments ===
    urdf_file = LaunchConfiguration('urdf').perform(context)
    world_file = LaunchConfiguration('world').perform(context)

    # === Process the URDF/Xacro ===
    if urdf_file.endswith('.xacro'):
        robot_description = xacro.process_file(urdf_file).toxml()
    else:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()


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
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ],
    )

    # === Spawn the robot in Ignition ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'pioneer3dx', '-topic', 'robot_description'],
        output='screen',
    )



    return [
        #set_ignition_env,
        gz_resource_path,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
    ]


def generate_launch_description():
    desc_pkg = get_package_share_directory('amr_robots_description')
    sim_pkg = get_package_share_directory('amr_gazebo')

    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_pkg, 'worlds', 'empty.world'),
        description='Path to Gazebo Ignition world file',
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=os.path.join(desc_pkg, 'urdf', 'pioneer3dx_gazebo.urdf.xacro'),
        description='Path to robot URDF or Xacro file',
    )

    return LaunchDescription([
        world_arg,
        urdf_arg,
        OpaqueFunction(function=launch_setup),
    ])
