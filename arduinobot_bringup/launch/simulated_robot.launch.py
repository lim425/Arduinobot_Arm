import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Step1: Gazebo simulation
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    
    # Step2: Ros2 controll for robotics arm
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    # Step3: Moveit path planning
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    # Step4: Alexa Integration
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py"
            )
            
        )
    
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface,
    ])