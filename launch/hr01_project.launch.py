import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   realsense2_camera_d435 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'),'launch'),
         '/rs_color_launch.py'])
      )
      
   apriltag_41h12 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('apriltag_ros'), 'launch'),
         '/tag_41h12_all.launch.py'])
      )

   
#   apriltag_to_visual_odometry_node = IncludeLaunchDescription(
#      PythonLaunchDescriptionSource([os.path.join(
#         get_package_share_directory('apriltag_to_visual_odometry'), 'launch'),
#         '/apriltag_to_visual_odometry.launch.py'])
#      )


   return LaunchDescription([
      realsense2_camera_d435,
      apriltag_41h12,
#      apriltag_to_visual_odometry_node
   ])
