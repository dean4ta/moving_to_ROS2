from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='ros1_bridge',
      node_executable='dynamic_bridge',
      node_name='bridge'
    ),
    Node(
      package='cpp_pubsub',
      node_executable='talker',
      node_name='talker'
    )
  ])