import rospy

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

