from quickzonoreach.zono import get_zonotope_reachset
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import pickle
from tf.transformations import euler_from_quaternion

class FRS():
    def __init__(self, map_file = None):
        # State-space model  
        # State: [x,y, vx, vy, vx_ref]]    
        self.a_mat = np.array([[0, 0, 1, 0, 0], 
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0]])
        self.b_mat = np.array([[0, 0],
                [0, 0],
                [1,0],
                [0,1],
                [0,0]])
        
        # Get 
        if map_file is None:
            map_file = rospy.get_param("~map_file")
        with open(map_file, 'rb') as f:
            self.lanelet_map = pickle.load(f)
        self.lanelet_map.build_graph(0)

    def get(self, state: Odometry, t_list, K_vx, K_y, K_vy, dx, dy, v_ref = None, allow_lane_change=True):
        '''
        Get the zonotope reachable set for given time steps
        Parameters:
            t_list: [N,] np.ndarray, time to calculate the reachable set [s] 
                Note: this is the absolute wall time in your ROS system
            K_vx: float, gain for longitudinal velocity
            K_y: float, gain for lateral state
            K_vy: float, gain for lateral velocity
            v_ref: float, reference velocity
            dx: float, disturbance in x direction [m/s^2]
            dy: float, disturbance in y direction [m/s^2]
            allow_lane_change: bool, whether to allow lane change
                Note: if True, the reachable set will be calculated for 
                    1) the closest lanelet
                    2) the neighboring lanelets
                    If False, the reachable set will be calculated for
                    1) the closest lanelet
                    2) the neighboring lanelets if the vehicle cross the lanelet boundary
        Returns:
            time_varying_vertices: list of N reachable sets,
                    each reachable set is list of T polygons. Each polygon is a (P,2) np.ndarray
        '''
        # Initialize
        if not isinstance(t_list, np.ndarray):
            t_list = np.array(t_list)
            
        reachable_sets_list = []
        
        # get pose of the vehicle
        x = state.pose.pose.position.x
        y = state.pose.pose.position.y
        
        q = [state.pose.pose.orientation.x, state.pose.pose.orientation.y, 
                state.pose.pose.orientation.z, state.pose.pose.orientation.w]
        psi = euler_from_quaternion(q)[-1]
        pose = np.array([x, y, psi])

        # add reference lane into the list
        ref_lanelet_list = []
        
        # get lateral distance to the lane
        closest_lanelet, s_start = self.lanelet_map.get_closest_lanelet(pose, check_psi=True)
        d_lateral, _  = closest_lanelet.distance_to_centerline(pose[:2])
        ref_lanelet_list.append((closest_lanelet.id, s_start, d_lateral))

        if allow_lane_change:
            for left in closest_lanelet.left:
                left_lanelet = self.lanelet_map.get_lanelet(left)
                d_lateral, s_start  = left_lanelet.distance_to_centerline(pose[:2])
                ref_lanelet_list.append((left, s_start, d_lateral))
            for right in closest_lanelet.right:
                right_lanelet = self.lanelet_map.get_lanelet(right)
                d_lateral, s_start  = right_lanelet.distance_to_centerline(pose[:2])
                ref_lanelet_list.append((right, s_start, d_lateral))
        elif d_lateral > 0.05: # if vehicle overleap the left lane boundary
            for right in closest_lanelet.right:
                right_lanelet = self.lanelet_map.get_lanelet(right)
                d_lateral, s_start  = right_lanelet.distance_to_centerline(pose[:2])
                ref_lanelet_list.append((right, s_start, d_lateral))
        elif d_lateral < -0.05: #if vehicle overleap the left lane boundary
            for left in closest_lanelet.left:
                left_lanelet = self.lanelet_map.get_lanelet(left)
                d_lateral, s_start  = left_lanelet.distance_to_centerline(pose[:2])
                ref_lanelet_list.append((left, s_start, d_lateral))


        for (start_id, s_start, d_lateral) in ref_lanelet_list:
            # construct the closed loop state space model
            k_mat = np.array([[0, 0, K_vx, 0, -K_vx],
                    [0, K_y, 0, K_vy, 0]])

            a_hat_mat = self.a_mat - self.b_mat@k_mat
            
            # construct the disturbance matrix
            input_box = [[-dx, dx], [-dy, dy]]
            
            # contruct the initial box
            vx_cur = state.twist.twist.linear.x # current longitudinal velocity
            vy_cur = state.twist.twist.linear.y # current lateral velocity
            if v_ref is None:# if no reference velocity is given, use current velocity
                v_ref = vx_cur 
            init_box = [[-1e-3, 1e-3], 
                        [d_lateral-1e-3, d_lateral+1e-3], 
                        [vx_cur,vx_cur], 
                        [vy_cur,vy_cur], 
                        [v_ref, v_ref]]
            
            a_mat_list = []
            b_mat_list = []
            input_box_list = []

            dt_list = (t_list - state.header.stamp.to_sec()).tolist()
            dt_list[1:] = np.diff(dt_list)
            # print("dt_list: ", dt_list)
            for _ in range(len(dt_list)):
                a_mat_list.append(a_hat_mat)
                b_mat_list.append(self.b_mat)
                input_box_list.append(input_box)
            
            # get the zonotope in frenet frame
            zonotopes = get_zonotope_reachset(init_box, a_mat_list, b_mat_list, input_box_list, dt_list, quick=True)
            
            # max in x
            d_max = np.max(np.array(zonotopes[-1].verts())[:,0])+0.3
            reachable_routes = self.lanelet_map.get_reachable_path(start_id, s_start, d_max, False)
            
            # Map back to the global frame

            for route in reachable_routes:
                reachable_sets = []
                for zonotope in zonotopes[1:]:
                    # Reachable set is rectangular
                    x_min = np.min(np.array(zonotope.verts())[:,0])-0.1
                    x_max = np.max(np.array(zonotope.verts())[:,0])+0.3
                    y_min = np.min(np.array(zonotope.verts())[:,1])-0.1
                    y_max = np.max(np.array(zonotope.verts())[:,1])+0.1
                
                    s_min = x_min / route.length
                    s_max = x_max / route.length
                    
                    s_samples = np.linspace(s_min, s_max, 10)
                    
                    sampled_pt = route.getValue(s_samples) #[N,2]
                    deri = route.getDerivative(s_samples)
                    slope = np.arctan2(deri[1], deri[0]) # [N,]
                    
                    right_bound = np.array([sampled_pt[:,0] + np.sin(slope)*y_min,
                            sampled_pt[:,1] - np.cos(slope)*y_min]).T #[N,2]
                    
                    left_bound = np.flip(np.array([sampled_pt[:,0] + np.sin(slope)*y_max,
                                        sampled_pt[:,1] - np.cos(slope)*y_max]).T, axis = 0) #[N,2]
                    
                    reachable_sets.append(np.concatenate([right_bound, left_bound], axis = 0))
                reachable_sets_list.append(reachable_sets)
            
        return reachable_sets_list
    

