unique_identifier_msgs/UUID uuid
float32 score # detection confidence
geometry_msgs/Point position # center
# Rotation of the box from the header frame's axes. size is measured along the
# box's own axes after this rotation.
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float64[9] position_covariance
float64[9] velocity_covariance
