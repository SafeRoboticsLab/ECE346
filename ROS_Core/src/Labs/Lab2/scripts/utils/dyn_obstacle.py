import numpy as np
import rospy
from racecar_obs_detection.srv import GetFRS, GetFRSResponse
from visualization_msgs.msg import Marker, MarkerArray


def frs_to_obstacle(frs_respond: GetFRSResponse)->list:
    '''
    This function converts the response from the FRS service to a list of obstacles
    Parameters:
        frs_respond: GetFRSResponse, the response from the FRS service
    Returns:
        obstacles_list: list of obstacles that can be used by ILQR
    '''
    obstacles_list = []
    for frs in frs_respond.FRS: # A list of SetArray
        vertices_list = []
        for frs_t in frs.set_list: # A list of polygon in a setarry
            polygon = []
            for points in frs_t.points:
                polygon.append([points.x, points.y])
            vertices_list.append(np.array(polygon))
        obstacles_list.append(vertices_list)
    return obstacles_list

def frs_to_msg(frs_respond: GetFRSResponse)->MarkerArray:
    '''
    This function converts the response from the FRS service to a MarkerArray for visualization
    '''
    marker_array = MarkerArray()
    
    if frs_respond.FRS is None:
        return marker_array
    
    for i, frs in enumerate(frs_respond.FRS): # A list of SetArray
        for t, frs_t in enumerate(frs.set_list): # A list of polygon in a setarry
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "frs_" + str(i)
            marker.id = t
            marker.action = 0
            marker.type = 4 # line strip
            
            marker.color.r = 204.0/255.0
            marker.color.g = 102.0/255.0
            marker.color.b = 0.0/255.0
            marker.color.a = 0.5
            
            marker.points = frs_t.points
            marker.points.append(frs_t.points[0])
            
            marker.scale.x = 0.01
            
            marker.lifetime = rospy.Duration(0.2)
            
            marker_array.markers.append(marker)
    return marker_array