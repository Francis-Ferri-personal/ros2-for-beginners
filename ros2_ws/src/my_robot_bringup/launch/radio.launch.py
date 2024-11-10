from launch import LaunchDescription
from launch_ros.actions import Node


# Mandatory fucntion
def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["giskard", "bb8", "daneel","lander", "c3po"]

    # Robots
    for name in robot_names:

        station= Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name=f"robot_news_station_{name}",
            parameters=[{"robot_name": name}],
        )

        ld.add_action(station)

    # Smartphone
    smartphone = Node(package="my_cpp_pkg", executable="smartphone")

    ld.add_action(smartphone)

    return ld
