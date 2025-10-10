from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Caminho para o mundo ===
    # world_file = '/root/ros2_ws/src/bug_algorithms/worlds/bug1_world/bug1_world.world'
    
    tb3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    world_file = os.path.join(tb3_gazebo_path, 'worlds', 'turtlebot3_house.world')

    pkg_path = get_package_share_directory('bug_algorithms')

    # === Ajusta o GAZEBO_MODEL_PATH para incluir mundos e modelos TurtleBot3 ===
    tb3_models_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )
    os.environ["GAZEBO_MODEL_PATH"] = f"/root/ros2_ws/src/bug_algorithms/worlds:{tb3_models_path}"

    # === Caminho do modelo do TurtleBot3 Waffle ===
    turtlebot3_model_path = os.path.join(tb3_models_path, 'turtlebot3_waffle', 'model.sdf')

    # === Caminho do mapa YAML ===
    map_file = os.path.join(pkg_path, 'maps', 'turtlebot3_house.yaml')

    # === Inicia Gazebo com plugin ROS Factory ===
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_factory.so',
            '--pause'
        ],
        output='screen'
    )

    # === Spawna o TurtleBot3 com delay de 5s ===
    spawn_turtlebot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'wafflebot',
                    '-file', turtlebot3_model_path,
                    '-x', '-6.5',
                    '-y', '-2.0',
                    '-z', '0.5',
                    '-Y', '1.57'
                ],
                output='screen'
            )
        ]
    )

    # === Nó do map_server (publica o mapa no tópico /map) ===
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # === Nodo do algoritmo Potential Fields ===
    potential_fields_node = Node(
        package='bug_algorithms',
        executable='potential_fields',
        name='potential_fields',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot,
        map_server,
        # potential_fields_node
    ])
