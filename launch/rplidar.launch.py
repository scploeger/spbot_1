import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',

            }]
        )
    ])




"""
ros2 run rplidar_ros rplidar_node --ros-args
     -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
     -p serial_baudrate:=115200
     -p frame_id:=laser_frame
     -p angle_compensate:=true
     -p scan_mode:=Standard
"""