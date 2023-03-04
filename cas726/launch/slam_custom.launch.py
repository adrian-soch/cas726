# Launch a mapper node
# @author Todor Stoyanov


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_cas726 = get_package_share_directory('cas726')

    slam_config = PathJoinSubstitution(
        [pkg_cas726, 'config', 'slam_custom.yaml'])

    slam = Node(
            package='cas726',
            executable='mapper',
            name='custom_slam',
            output='screen',
            parameters=[
              slam_config,
              {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
#            prefix=['xterm -e gdb --args'],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    return ld
