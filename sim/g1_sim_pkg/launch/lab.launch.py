import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Simulacion del LAB REAL: mundo generado de los mapas del robot (lab.world, frame G1).
    Mismos waypoints que el robot real (A/B/C de waypoints.json) y misma puerta
    (G1_DOOR_X/Y/AXIS = -3.90/1.25/135). Spawn en el waypoint A con su yaw real."""
    pkg = get_package_share_directory('g1_sim')
    gz = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg, 'worlds', 'lab.world')

    # MODELO: 'box' (DEFECTO, g1_base.urdf = caja+lidar, fisica planar LIMPIA) | 'full' (g1_nav
    # completo, bonito para demos). MEDIDO 2026-07-03 (run sim 140803, 10 colisiones en espacio
    # ABIERTO): el modelo completo lleva las piernas rigidas sin controlador, los pies penetran
    # ... DIAGNOSTICO FINAL: ambos modelos VOLCABAN (CG alto + inercia pequena vs planar_move);
    # ademas el full no tenia NINGUNA colision (fantasma). Arreglado en el URDF (anti-vuelco +
    # caja de colision unica + mu=0): el modelo FULL vuelve a ser el defecto, ya es estable.
    model = os.environ.get('G1_SIM_MODEL', 'full').strip().lower()
    if model == 'full':
        g1desc = get_package_share_directory('g1_description')
        urdf_path = os.path.join(g1desc, 'urdf', 'g1_nav.urdf')
    else:
        urdf_path = os.path.join(pkg, 'urdf', 'g1_base.urdf')
    with open(urdf_path) as f:
        robot_desc = f.read()

    gui = LaunchConfiguration('gui')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])

    # spawn en el waypoint A real: (0.99, 0.57) yaw -120.3 grados = -2.10 rad
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'g1',
                            '-x', '0.99', '-y', '0.57', '-z', '0.05', '-Y', '-2.10',
                            '-timeout', '120'])   # el mundo del lab tarda en cargar: 30s se quedaba corto

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='true para abrir la GUI de Gazebo (necesita pantalla/VNC)'),
        gzserver, gzclient, rsp, spawn,
    ])
