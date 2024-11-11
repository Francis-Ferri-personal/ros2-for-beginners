#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import random
import math
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_robot_interfaces.msg import Turtle, TurtleList
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.declare_parameter("catch_closest_turtle_first", True)

        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first"
        ).value
        self.turtle1_pose_ = None
        self.turtles_ = {}
        self.control_gain_ = 2.5  # P-controller gain, adjust as necessary
        self.tolerance_ = 0.25

        # Turtle 1 position
        self.turtle1_pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle1_pose, 10
        )
        self.turtle1_pose_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10
        )

        # Turtles alive
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleList, "alive_turtles" ,self.callback_alive_turtles, 10
        )

        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Serve Catch turtle...")

    def callback_turtle1_pose(self, msg: Pose):
        self.turtle1_pose_ = msg

        if len(self.turtles_) == 0:
            return

        # Find the chosen turtle based on the flag
        if self.catch_closest_turtle_first_:
            # Use the min function to find the closest turtle by distance
            chosen_turtle = min(
                self.turtles_.items(),  # Iterate over key-value pairs (name, turtle)
                key=lambda item: math.sqrt(
                    (item[1][0] - self.turtle1_pose_.x) ** 2
                    + (item[1][1] - self.turtle1_pose_.y) ** 2
                ),
            )
        else:
            # Select the first turtle in the dictionary (name, turtle)
            chosen_turtle = next(iter(self.turtles_.items()))

        self.move_turtle_to_target(self.turtle1_pose_, chosen_turtle)

    def move_turtle_to_target(self, turtle1_pose, chosen_turtle):
        turtle_name, turtle = chosen_turtle
        self.get_logger().info(f"Moving master turtle to {turtle_name}")

        # Calculate the difference in position (x, y)
        delta_x = turtle[0] - turtle1_pose.x
        delta_y = turtle[1] - turtle1_pose.y

        # Calculate the distance to the target
        distance = math.sqrt(delta_x**2 + delta_y**2)

        if distance < self.tolerance_:
            self.catch_turtle(turtle_name)
            return

        # Calculate the angle to the target
        target_angle = math.atan2(delta_y, delta_x)

        # Calculate the error in angle (angular difference)
        angle_error = target_angle - turtle1_pose.theta

        # Normalize the angle error to be between -pi and pi
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Proportional control for linear velocity (move towards the target)
        linear_velocity = self.control_gain_ * distance

        # Proportional control for angular velocity (rotate towards the target)
        angular_velocity = self.control_gain_ * angle_error

        # Create a Twist message for controlling turtle1
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        self.turtle1_pose_publisher_.publish(twist)

    def callback_alive_turtles(self, msg: TurtleList):

        turtles: TurtleList = msg.turtles

        for turtle in turtles:
            turtle: Turtle
            self.turtles_[turtle.name] = (turtle.x, turtle.y, turtle.theta)

    def catch_turtle(self, turtle_name):
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle, turtle_name=turtle_name)
        )
        del self.turtles_[turtle_name]

    def callback_catch_turtle(self, future: Future, turtle_name):
        try:
            response: CatchTurtle.Response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f"Turtle {turtle_name} caught.")
            else:
                self.get_logger().info(f"Problem to catch Turtle {turtle_name}.")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
