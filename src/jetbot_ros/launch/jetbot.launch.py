from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_ros',
            # namespace='cam',
            executable='cam_pub_new',
            # name='burger'
        ),        
        Node(
            package='test_ros_websock',
            # namespace='cam',
            executable='test_subscriber',
            # name='burger'
        ),        

    ])