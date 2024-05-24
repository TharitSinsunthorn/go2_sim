from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    
    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    FR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FRposition_trajectory_controller", "-c", "/controller_manager"],
    )

    FL_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FLposition_trajectory_controller", "-c", "/controller_manager"],
    )

    RR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RRposition_trajectory_controller", "-c", "/controller_manager"],
    )
    
    RL_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RLposition_trajectory_controller", "-c", "/controller_manager"],
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            
            RegisterEventHandler(
                event_handler=OnProcessExit(
                  target_action=joint_state_broadcaster_spawner,
                  on_exit=[FR_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=FR_robot_controller_spawner,
                  on_start=[FL_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=FL_robot_controller_spawner,
                  on_start=[RR_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=RR_robot_controller_spawner,
                  on_start=[RL_robot_controller_spawner],
                )
            ),
            
            # imu_broadcaster_spawner
        ]
    )