from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Caminho para o mundo
    world_file = '/root/ros2_ws/src/bug_algorithms/worlds/bug1_world/bug1_world.world'

    # Configura o GAZEBO_MODEL_PATH incluindo seus modelos e TurtleBot3
    os.environ["GAZEBO_MODEL_PATH"] = "/root/ros2_ws/src/bug_algorithms/worlds:/opt/ros/humble/share/turtlebot3_gazebo/models"

    # Ajusta o GAZEBO_MODEL_PATH para incluir seus mundos e os modelos do TurtleBot3
    tb3_models_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )
    os.environ["GAZEBO_MODEL_PATH"] = (
        "/root/ros2_ws/src/bug_algorithms/worlds:" + tb3_models_path
    )

    # Caminho do modelo do waffle
    turtlebot3_model_path = os.path.join(
        tb3_models_path, 'turtlebot3_waffle', 'model.sdf'
    )

    # Comando para iniciar Gazebo com factory plugin
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_factory.so',
            '--pause'
        ],
        output='screen'
    )

    # Spawna o turtlebot com delay de 5s
    spawn_turtlebot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'wafflebot',
                    '-file', turtlebot3_model_path,
                    '-x', '1.540100',
                    '-y', '-7.914520',
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    bug1_node = Node(
        package='bug_algorithms',
        executable='bug1',   # tem que estar no setup.py ou CMakeLists como entry_point
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_turtlebot,
        bug1_node
    ])