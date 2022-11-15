# apriltag_to_visual_odometry

This ROS2 nodes use the Apriltag ROS2 detections to send the visual odometry pose estimation to PX4.

apriltag_to_visual_odometry merges the pose informations from multiple AprilTag detections to improve the estimation accuracy. Different algorithms are implemented for filtering and averaging the transforms published by apriltag_ros.

## Topics
### Subscriptions:
- `/tf_vio` - where apriltag_ros publish the transforms of the detected AprilTags