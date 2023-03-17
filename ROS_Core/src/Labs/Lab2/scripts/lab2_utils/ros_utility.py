import rospy
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_ros_param(param_name: str, default):
    '''
    Read a parameter from the ROS parameter server. If the parameter does not exist, return the default value.
    Args:
        param_name: string, name of the parameter
        default: default value
    Return:
        value of the parameter
    '''
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    else:
        # try seach parameter
        if param_name[0] == '~':
            search_param_name = rospy.search_param(param_name[1:])
        else:
            search_param_name = rospy.search_param(param_name)

        if search_param_name is not None:
            rospy.loginfo('Parameter %s not found, search found %s, using it', param_name, search_param_name)
            return rospy.get_param(search_param_name)
        else:
            rospy.logwarn("Parameter '%s' not found, using default: %s", param_name, default)
            return default


def pose2T(pose: Pose):
    '''
    Convert a geometry_msgs/Pose message to a SE(3) matrix
    Args:
        pose: geometry_msgs/Pose message
    Return:
        T: a 4x4 numpy array
    '''
    orientation = pose.orientation
    position = pose.position
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    trans = [position.x, position.y, position.z]
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = trans
    return T

