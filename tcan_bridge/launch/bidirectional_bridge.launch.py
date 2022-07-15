import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('tcan_bridge'),
      'config',
      'bidirectional.yaml'
      )

   return LaunchDescription([
      Node(
         package='tcan_bridge',
         executable='bidirectional_bridge',
         name='tcan_bridge',
         parameters=[config]
      )
   ])