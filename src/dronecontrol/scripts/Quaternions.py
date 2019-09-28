from geometry_msgs.msg import Quaternion
# Create a list of floats, which is compatible with tf
# quaternion methods
quat_tf = [0, 1, 0, 0]
quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

ROS_INFO_STREAM(quat_msg)  
