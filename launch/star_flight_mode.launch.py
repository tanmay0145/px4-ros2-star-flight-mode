#!/usr/bin/env python3

"""Launch file for the PX4 Python Star Flight Mode."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for star flight mode."""
    return LaunchDescription([
        Node(
            package='px4_python_star_flight_mode',
            executable='star_flight_mode.py',
            name='star_flight_mode',
            output='screen',
            emulate_tty=True,
            parameters=[
                # You can add parameters here if needed
            ]
        )
    ])