from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Turtlesim
    turtlesim = Node(package="turtlesim", executable="turtlesim_node")

    # Spawner
    turtle_spawner = Node(
        package="turtlesim_catch_them_all",
        executable="turtle_spawner",
        parameters=[{"spawn_frequency": 0.25}, {"turtle_name_prefix": ""}],
    )

    # Controller
    turtle_controller = Node(
        package="turtlesim_catch_them_all",
        executable="turtle_controller",
        parameters=[{"catch_closest_turtle_first": True}],
    )

    ld.add_action(turtlesim)
    ld.add_action(turtle_spawner)
    ld.add_action(turtle_controller)

    return ld
