
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Enable GUI')

    # Get the world file path
    bookstore_world_pkg = get_package_share_directory('bookstore_world')
    world_file = os.path.join(bookstore_world_pkg, 'worlds', 'bookstore.world')  # Change from .sdf to .world

    # Launch Ignition Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        ign_gazebo
    ])
