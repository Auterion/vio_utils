# vio_utils
ROS package that holds various utility nodes that are used to process, augment or visualize data for visual inertial odometry

## butterworth_imu
Subscribe to mavros/imu/data_raw and apply a second order butterworth to the data.
Note: The use of this node was deprecated, applying a digital low-pass directly in the IMU driver showed much better performance

## path_broadcaster
Subscribe to mavros/local_position/pose and vio pose and add incoming poses to a nav_msgs::Path. 
The paths are published and can be visualized in rviz

## tf_broadcaster
Subscribe to mavros/local_position/pose and vio pose and determine a pose offset based on the first received pose messages.
Publish a transform::tf to substract this offset for any subsequent poses. This allows to align GPS and VIO path for visualization.
