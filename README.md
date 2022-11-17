# apriltag_to_visual_odometry

This ROS2 node uses the Apriltag ROS2 detections to send the visual odometry pose estimation to PX4.

apriltag_to_visual_odometry merges the pose informations from multiple AprilTag detections to improve the estimation accuracy. Different algorithms are implemented for filtering and averaging the transforms published by apriltag_ros.

## Topics
### Subscriptions:
- `/tf_vio` - where apriltag_ros publishes the transforms of the detected AprilTags;
- `/fmu/vehicle_odometry/out` - where PX4 publishes the visual odometry pose after the EKF computation;

### Publisher:
- `/tf` - where apriltag_to_visual_odometry publishes a vector of transforms to visualize a simulation of the environments;
- `/fmu/vehicle visual odometry/in` - where apriltag_to_visual_odometry publishes the visual odometry pose, which PX4 merges with EKF.

## Configuration
The node is configured via a yaml configurations file.

The configuration file `apriltag_virtual_map.yaml` has the format:
```yaml
apriltag_to_visual_odometry:
ros__parameters:
debug: false    # To see more feedback on the terminal
graphics_on: true   # To see the apriltag-map, position estimated from Apriltag(drone) and position estim>
#Camera parameters
# camera_frame: "camera_color_optical_frame" # ***HR01***
camera_frame:  "camera"
roll_cam: 0      # [deg]
pitch_cam: 0     # [deg]
#yaw_cam: -90     # [deg] ***HR01***
yaw_cam: 90     # [deg] ***QR01***
#offset_camera_body: [ 0.035, 0.15, -0.1 ]       # [m]   ***HR01***
#offset_body_camera: [ 0.15, -0.035, 0.1 ]       # [m]   ***HR01***
offset_camera_body: [ 0, -0.105, 0 ]       # [m] ***QR01***
offset_body_camera: [ 0.105, 0, 0 ]       # [m]  ***QR01***

# filtering parameters
just_bigger_one: false          # consider only the bigger frame 
euc_dist_filter: false          # compute euclidean distance between a frame location and the past pose to delete the outliers
iqr_filter: true                # weighted iqr filter to delete the outliers
just_two_size: false           # consider only the two bigger sizes of frames seen 
euc_dist_max: 0.05             # euclidean distance limit for deleting outliers
euc_dist_to_increase: 0.01     # increase the euclidean distance limit if too frames are deleted
euc_outlier_ratio:  0.3        # ratio between the number of frames that have not been deleted and all the frames seen
fir: true                
fir_weight: [1, 1, 1, 1, 1]

#Specify the apriltag_map. Example:
#tag_ids: [first_id_XL, ... , last_id_XL, first_id_L, ... ,last_id_L, first_id_M, ... , last_id_M, first_id_S, ... , last_id_S]
#tags_locations_<size_tags>: [x_first, y_first, ... , x_last, y_last]      # tag_<ID>-xy_location relative world

# Apriltag division: 
# XL(0 --> 99) 
# L(100 --> 399) 
# M(400 --> 999) 
# S(1000 --> ...) 

tag_ids: [0, 1, 100, 101, 400, 401, 1000, 1001]
tags_locations_XL: [0.256, 0.256, 1.716, 0.256]
tags_locations_L: [0.621, 0.256, 0.986, 0.256]
tags_locations_M: [0.6205, 0.073, 0.803, 0.073]
tags_locations_S: [0.53, 0.0735, 0.7125, 0.0735]
```

The best accuracy has been guaranteed using the iqr_filter and the fir, as specified in the example configuration file.

## Dependencies
This ROS2 node needs the following packages to run:
- [apriltag](https://github.com/C-square-unipd/apriltag);
- [apriltag_ros](https://github.com/C-square-unipd/apriltag_ros);
- apriltag_viz (da capire se serve davvero)
- [apriltag_msgs](https://github.com/C-square-unipd/apriltag_msgs);
- [px4_msgs](https://github.com/C-square-unipd/px4_msgs);
- [px4_ros_com](https://github.com/C-square-unipd/px4_ros_com);
- [Eigen3 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page).
