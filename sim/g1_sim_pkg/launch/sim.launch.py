import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('g1_sim')
    g1desc = get_package_share_directory('g1_description')
    gz = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg, 'worlds', 'room.world')
    with open(os.path.join(g1desc, 'urdf', 'g1_nav.urdf')) as f:
        robot_desc = f.read()

    gui = LaunchConfiguration('gui')

    # gzserver always (headless-capable). CPU 'ray' lidar publishes /scan without OpenGL.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    # gzclient (GUI) only when gui:=true  ->  set gui:=false on headless/SSH servers
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'g1', '-z', '0.05'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to false for headless servers (no Gazebo window)'),
        gzserver, gzclient, rsp, spawn,
    ])
