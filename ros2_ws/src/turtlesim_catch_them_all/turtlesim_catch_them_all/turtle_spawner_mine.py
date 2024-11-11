#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import random
from functools import partial

# ros2 interface show turtlesim/srv/Spawn
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from my_robot_interfaces.msg import Turtle, TurtleList
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 0.1)
        self.declare_parameter("turtle_name_prefix", "")

        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value

        self.turtle_idx = 0
        self.turtles = {}

        self.publiher_ = self.create_publisher(TurtleList, "alive_turtles", 10)

        self.server_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        self.timer_spawn_ = self.create_timer(
            1.0 / self.spawn_frequency, self.call_spawn_turtle
        )

    def call_spawn_turtle(self):
        client = self.create_client(Spawn, "spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Spawn Turtle...")

        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 360.0)
        name = f"{self.turtle_name_prefix}turtle_{self.turtle_idx}"

        request = Spawn.Request()
        request.x, request.y, request.theta, request.name = x, y, theta, name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, x=x, y=y))
        self.turtle_idx += 1

    def callback_spawn_turtle(self, future: Future, x, y):
        try:
            response: Spawn.Response = future.result()
            name = response.name
            if name:
                self.turtles[name] = (x, y)
                self.get_logger().info(
                    f"New turtle '{name}'spawned at x:{str(x)} + y:{str(y)}"
                )
                self.publish_turtle_list()

            else:
                self.get_logger().info(
                    f"Impossible to spawn turtle spawned at x:{str(x)} + y:{str(y)}"
                )

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_kill_turtle(self, turtle_name):
        client = self.create_client(Kill, "kill")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Kill Turtle...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle, name=turtle_name))

    def callback_kill_turtle(self, future: Future, name):
        try:
            future.result()
            del self.turtles[name]
            self.publish_turtle_list()
            self.get_logger().info(f"Turtle '{name}' was killed.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def publish_turtle_list(self):
        # Create the alive turtle list
        turtle_list = [
            Turtle(name=name, x=x, y=y) for name, (x, y) in self.turtles.items()
        ]
        msg = TurtleList(turtles=turtle_list)

        self.publiher_.publish(msg)

    def callback_catch_turtle(
        self, request: CatchTurtle.Request, response: CatchTurtle.Response
    ):
        turtle_name = request.name
        self.call_kill_turtle(turtle_name)
        # TODO: Improve this to handle theexceptions correctly
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
