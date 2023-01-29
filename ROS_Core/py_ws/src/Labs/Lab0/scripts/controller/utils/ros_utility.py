import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def get_ros_param(param_name, default):
    '''
    Read a parameter from the ROS parameter server. If the parameter does not exist, return the default value.
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
        

def state_to_pose_stamped(state, t, frame_id='map'):
    '''
    Convert a State Vector object to a PoseStamped message
    state: [x,y,v,yaw,delta]
    t: float time in seconds
    frame_id: string
    '''

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp =rospy.Time.from_sec(t)
    pose.pose.position.x = state[0]
    pose.pose.position.y = state[1]
    pose.pose.position.z = 0.0
    
    q = quaternion_from_euler(0.0, 0.0, state[3])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose