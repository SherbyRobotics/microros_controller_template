import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([

        Node(
                package="joy",
                executable="joy_node",
                name="joy",
                parameters = [
                    {'deadzone':0.0},
                    {'dev': "/dev/input/js0"},
                    ],
                ),

        Node(
		package="microros_controller_template",
		executable="controller",
		name="controller",
		),
    ])
