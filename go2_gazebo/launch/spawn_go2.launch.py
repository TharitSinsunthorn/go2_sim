import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():
    
    world_file_name = LaunchConfiguration('world_file_name')
    urdf_file = LaunchConfiguration('urdf_file')

    world_file_name_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='empty.world'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='go2.xacro'
    )

    # Position and orientation
    # [X, Y, Z]
    position = [0.06, 0.0, 0.6]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_name = "GO2"
    
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_gazebo'), 'launch'),
            '/start_world.launch.py']),
    launch_arguments={'world_file_name': world_file_name}.items(),
    )
    
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   robot_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_gazebo'), 'launch'),
            '/jtc_go2.launch.py'])
    )

    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_description'), 'launch', 'go2_visualize.launch.py')
            ]),
    launch_arguments={'use_joint_state_gui': 'False',
                      'use_sim_time': "True",
                      'urdf_file': urdf_file}.items(),
    )
    
    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_teleop'), 'launch', 'joystick.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )
    
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_navigation'), 'launch', 'nav_core.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )


    # create and return launch description object
    return LaunchDescription(
        [
            world_file_name_arg,
            urdf_file_arg,
            start_world,
            spawn_robot,
            launch_ros2_control,
            visualize_robot,
            # joystick, 
            # navigation
        ]
    )