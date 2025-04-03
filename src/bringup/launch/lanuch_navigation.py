import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Paths to commands
    turtlebot3_house_cmd = "ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py"
    navigation_launch_cmd = "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:='/home/subnt/nav2_aruco/src/control/config/glob_costmap.yaml'"
    slam_toolbox_cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
    rviz_cmd = "ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

    return LaunchDescription([
        ExecuteProcess(
            cmd=[turtlebot3_house_cmd],
            shell=True
        ),
        ExecuteProcess(
            cmd=[navigation_launch_cmd],
            shell=True
        ),
        ExecuteProcess(
            cmd=[slam_toolbox_cmd],
            shell=True
        ),
        ExecuteProcess(
            cmd=[rviz_cmd],
            shell=True
        )
    ])
