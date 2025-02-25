import os

import launch
import launch_ros.actions


def generate_launch_description():
    params_file = os.path.join(os.getcwd(),
                               "src/KISS-Matcher/ros/config/params.yaml")

    return launch.LaunchDescription([
        launch_ros.actions.Node(package='pointcloud_visualizer',
                                executable='pointcloud_visualizer',
                                name='pointcloud_visualizer',
                                parameters=[params_file]),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                os.path.join(os.getcwd(),
                             "src/KISS-Matcher/ros/rviz/km_rviz.rviz")
            ],
            output='screen')
    ])
