# launch/two_nodes.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_onnx_ros2',      
            executable='AI_inference',                
            name='AI_inference',
            output='screen',                    
        ),
        Node(
            package='yolo_onnx_ros2',      
            executable='serial_writer',                
            name='serial_writer',
            output='screen',
        ),
    ])