from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  odom_handler_dir = get_package_share_directory('odom_handler')

  ld = LaunchDescription([
    Node(
      package='odom_handler', 
      executable='odom_handler', 
      name='odom_handler',
      output='screen',
      parameters=[odom_handler_dir+"/../../command_logs/data_location.yaml"])
  ])
  return ld