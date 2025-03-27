import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Enable GUI')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    # Get the world file path
    bookstore_world_pkg = get_package_share_directory('bookstore_world')
    world_file = os.path.join(bookstore_world_pkg, 'worlds', 'bookstore.world')

    # Define TurtleBot3 model SDF file
    turtlebot3_pkg = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_model = os.path.join(turtlebot3_pkg, 'models', 'turtlebot3_waffle', 'model.sdf')

    # Spawn position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Launch Ignition Gazebo with the world file
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # Spawn TurtleBot3 using `ign service`
    spawn_turtlebot = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/world/bookstore/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', (
                'sdf_filename: "' + turtlebot3_model + '", '
                'name: "turtlebot3", '
                'pose: { position: { x: ' + x_pose.perform({}) + 
                ', y: ' + y_pose.perform({}) + 
                ', z: ' + z_pose.perform({}) + ' } }'
            )
        ],
        output='screen'
    )


    return LaunchDescription([
        gui_arg,
        use_sim_time,
        ign_gazebo,
        spawn_turtlebot
    ])
