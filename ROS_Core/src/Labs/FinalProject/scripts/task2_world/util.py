import rospy
from visualization_msgs.msg import Marker
from typing import Optional, Tuple, Union
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
from pyspline.pyCurve import Curve
from threading import Lock
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import pickle

# ! IMPORTANT: You do not need to modify this file for the final project

def get_ros_param(param_name: str, default = None):
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
            if default is None:
                raise RuntimeError(f"Parameter '{param_name}' not found")
            else:
                rospy.logwarn("Parameter '%s' not found, using default: %s", param_name, default)
                return default

def get_text_msg(text, x=3.0, y=3.0, scale=0.5, r=1.0, g=1.0, b=1.0, ns="info", id=0):
    marker = Marker()
    
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    
    marker.scale.z = scale  
    marker.lifetime = rospy.Duration(1.0)
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0
    
    marker.text = text
    
    return marker

def extract_state_odom(odom_msg: Odometry):
    '''
    Extract the state from the full state [x, y, v, psi, timestamp]
    '''
    t_slam = odom_msg.header.stamp.to_sec()
    # get the state from the odometry message
    q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, 
            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
    # get the heading angle from the quaternion
    psi = euler_from_quaternion(q)[-1]
    
    state_global =np.array([
                        odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        odom_msg.twist.twist.linear.x,
                        psi,
                        t_slam
        ])
    return state_global

class GeneratePwm():
    '''
    This class apply an open-loop model to convert 
    acceleration and steering angle to PWM that can
    be read by the ESC and servo
    '''
    def __init__(self):
        '''
        Constructor for the GeneratePwm class
        '''        
        # Read the parameters from the parameter server
        self.read_parameters()

        # Define the open-loop model
        self.mlp_model = pickle.load(open(self.model_path, 'rb'))
        
    def read_parameters(self):
        '''
        Read the maximum and minimum throttle for safety
        '''
        self.max_throttle = get_ros_param('~max_throttle', 0.5)
        self.min_throttle = get_ros_param('~min_throttle', -0.3)
        self.model_path = get_ros_param('~PWM_model', 'model.pkl')
        
    def convert(self, accel: float, steer: float, v: float):
        '''
        convert the acceleration and steering angle to PWM given the current state
        Parameters:
            accel: float, linear acceleration of the robot [m/s^2]
            steer: float, steering angle of the robot [rad]
            state: State2D, current state of the robot
        '''

        # Do not allow the car to go over 3m/s
        if v > 3:
            accel = min(accel, 0)
            v_bounded = 3
        else: 
            v_bounded = v
        
        # negative pwm means turn left (positive steering angle)
        steer_pwm = -np.clip(steer/0.37, -1, 1)
        accel_bounded = np.sign(accel)*min(abs(accel), 2+v)

        # Generate Input vector
        input = np.array([[accel_bounded, v_bounded, np.abs(steer_pwm)]])
        
        # check nan
        if np.any(np.isnan(input)):
            rospy.logwarn("Contain NAN in control!")
            return self.min_throttle, steer_pwm

        # convert the acceleration and steering angle to PWM
        d = self.mlp_model.predict(input)[0]
        
        # clip the throttle to the maximum and minimum throttle
        throttle_pwm = np.clip(d, self.min_throttle, self.max_throttle)
        
        # Composite the throttle for low speed
        if v<0.2:
            throttle_pwm += np.abs(steer_pwm)*0.04
            
        return throttle_pwm, steer_pwm

class RefPath:
    def __init__(self, center_line: np.ndarray, 
                width_left: Union[np.ndarray, float] = 1,
                width_right: Union[np.ndarray, float] = 1,
                speed_limt: Union[np.ndarray, float] = 1,
                loop: Optional[bool] = True) -> None:
        '''
        Considers a track with fixed width.

        Args:
            center_line: 2D numpy array containing samples of track center line
                        [[x1,x2,...], [y1,y2,...]]
            width_left: float, width of the track on the left side
            width_right: float, width of the track on the right side
            loop: Boolean. If the track has loop
        '''
        self.center_line_data = center_line.copy()
        
        # First, we build the centerline spline in XY space
        self.center_line = Curve(x=center_line[0, :], y=center_line[1, :], k=3)
        
        # Project back to get the s for each point
        s_norm, _ = self.center_line.projectPoint(center_line.T)
        
        if not isinstance(width_left, np.ndarray):
            self.width_left = Curve(x=s_norm, y = np.ones_like(s_norm) * width_left, k=3)
        else:
            self.width_left = Curve(x=s_norm, y=width_left, k=3)
            
        if not isinstance(width_right, np.ndarray):
            self.width_right = Curve(x=s_norm, y = np.ones_like(s_norm) * width_right, k=3)
        else:
            self.width_right = Curve(x=s_norm, y=width_right, k=3)
            
        if not isinstance(speed_limt, np.ndarray):
            self.speed_limit =  Curve(x=s_norm, y = np.ones_like(s_norm) * speed_limt, k=3)
        else:
            self.speed_limit = Curve(x=s_norm, y=speed_limt, k=3)
        
        self.loop = loop
        self.length = self.center_line.getLength()

        # variables for plotting
        self.build_track()

    def _interp_s(self, s: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Gets the closest points on the centerline and the slope of trangent line on
        those points given the normalized progress.

        Args:
            s (np.ndarray): progress on the centerline. This is a vector of shape
                (N,) and each entry should be within [0, 1].

        Returns:
            np.ndarray: the position of the closest points on the centerline. This
                array is of the shape (2, N).
            np.ndarray: the slope of of trangent line on those points. This vector
                is of the shape (N, ).
        """
        n = len(s)
        interp_pt = self.center_line.getValue(s)
        if n == 1:
            interp_pt = interp_pt[np.newaxis, :]
        slope = np.zeros(n)

        for i in range(n):
            deri = self.center_line.getDerivative(s[i])
            slope[i] = np.arctan2(deri[1], deri[0])
        return interp_pt.T, slope
            
    def interp(self, theta_list):
        """
        Gets the closest points on the centerline and the slope of trangent line on
        those points given the unnormalized progress.

        Args:
            s (np.ndarray): unnormalized progress on the centerline. This is a
                vector of shape (N,).

        Returns:
            np.ndarray: the position of the closest points on the centerline. This
                array is of the shape (2, N).
            np.ndarray: the slope of of trangent line on those points. This vector
                is of the shape (N, ).
        """
        if self.loop:
            s = np.remainder(theta_list, self.length) / self.length
        else:
            s = np.array(theta_list) / self.length
            s[s > 1] = 1
        return self._interp_s(s)

    def build_track(self):
        N = 500
        theta_sample = np.linspace(0, 1, N, endpoint=False) * self.length
        interp_pt, slope = self.interp(theta_sample)
        self.track_center = interp_pt

        if self.loop:
            self.track_bound = np.zeros((4, N + 1))
        else:
            self.track_bound = np.zeros((4, N))

        # Inner curve.
        width_left = self.width_left.getValue(theta_sample)[:,1]
        self.track_bound[0, :N] = interp_pt[0, :] - np.sin(slope) * width_left
        self.track_bound[1, :N] = interp_pt[1, :] + np.cos(slope) * width_left

        # Outer curve.
        width_right = self.width_right.getValue(theta_sample)[:,1]
        self.track_bound[2, :N] = interp_pt[0, :] + np.sin(slope) * width_right
        self.track_bound[3, :N] = interp_pt[1, :] - np.cos(slope) * width_right

        if self.loop:
            self.track_bound[:, -1] = self.track_bound[:, 0]
            
    def get_reference(self, points: np.ndarray,
            normalize_progress: Optional[bool] = False, 
            eps: Optional[float] = 1e-3) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        closest_pt, slope, s = self.get_closest_pts(points, eps=eps)
        
        v_ref = self.speed_limit.getValue(s)[:,1]
        
        if not self.loop:
            temp = (1-s) * self.length
            # bring the speed limit to 0 at the end of the path
            v_ref = np.minimum(v_ref, temp)
        v_ref = v_ref[np.newaxis, :]

        width_left = self.width_left.getValue(s)[:,1][np.newaxis, :]
        width_right = self.width_right.getValue(s)[:,1][np.newaxis, :]
        
        if not normalize_progress:
            s = s * self.length
        s = s[np.newaxis, :]
        return np.concatenate([closest_pt, slope, v_ref, s, width_right, width_left], axis=0)

    def get_closest_pts(self, points: np.ndarray, eps: Optional[float] = 1e-3) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Gets the closest points on the centerline, the slope of their tangent
        lines, and the progress given the points in the global frame.

        Args:
            points (np.ndarray): the points in the global frame, of the shape
                (2, N).

        Returns:
            np.ndarray: the position of the closest points on the centerline. This
                array is of the shape (2, N).
            np.ndarray: the slope of of tangent line on those points. This vector
                is of the shape (1, N).
            np.ndarray: the normalized progress along the centerline. This vector is of the
                shape (1, N).
        """
        if len(points.shape) == 1:
            points = points[:, np.newaxis]
            s, _ = self.center_line.projectPoint(points.T, eps=eps)
            s = np.array([s])
        else:
            # s0, _ = self.center_line.projectPoint(points[:,0], eps=eps)
            # s1 = min(s0+5/self.length, 1)
            # s0 *= 0.9
            # local_center_line = self.center_line.windowCurve(s0, s1)
            # s_local, _ = local_center_line.projectPoint(points.T, eps=eps)
            # s = s_local*local_center_line.getLength()/self.length + s0
            s, _ = self.center_line.projectPoint(points.T, eps=eps)
            
        closest_pt, slope = self._interp_s(s)
        slope = slope[np.newaxis, :]

        return closest_pt, slope, s

    def local2global(self, local_states: np.ndarray, return_slope=False) -> np.ndarray:
        """
        Transforms trajectory in the local frame to the global frame (x, y) position.

        Args:
            local_states (np.ndarray): The first row is the progress of the trajectory
                and the second row is the lateral deviation.

        Returns:
            np.ndarray: trajectory in the global frame.
        """
        flatten = False
        if local_states.ndim == 1:
            flatten = True
            local_states = local_states[:, np.newaxis]
        num_pts = local_states.shape[1]
        progress = local_states[0, :]
        assert np.min(progress) >= 0. and np.max(progress) <= 1., (
            "The progress should be within [0, 1]!"
        )
        lateral_dev = local_states[1, :]
        global_states, slope = self._interp_s(progress)
        if num_pts == 1:
            global_states = global_states.reshape(2, 1)
        global_states[0, :] = global_states[0, :] + np.sin(slope) * lateral_dev
        global_states[1, :] = global_states[1, :] - np.cos(slope) * lateral_dev

        if flatten:
            global_states = global_states[:, 0]
        if return_slope:
            return global_states, slope
        return global_states

    def global2local(self, global_states: np.ndarray) -> np.ndarray:
        """
        Transforms trajectory in the global frame to the local frame (progress, lateral
        deviation).

        Args:
            global_states (np.ndarray): The first row is the x position and the
                second row is the y position.

        Returns:
            np.ndarray: trajectory in the local frame.
        """
        flatten = False
        if global_states.ndim == 1:
            flatten = True
            global_states = global_states[:, np.newaxis]
        local_states = np.zeros(shape=(2, global_states.shape[1]))
        closest_pt, slope, progress = self.get_closest_pts(
            global_states, normalize_progress=True
        )
        dx = global_states[0, :] - closest_pt[0, :]
        dy = global_states[1, :] - closest_pt[1, :]
        sr = np.sin(slope)
        cr = np.cos(slope)

        lateral_dev = sr*dx - cr*dy
        local_states[0, :] = progress.reshape(-1)
        local_states[1, :] = lateral_dev

        if flatten:
            local_states = local_states[:, 0]

        return local_states

    # region: plotting
    def plot_track(self, ax: Optional[matplotlib.axes.Axes] = None,
                        c: str = 'k', linewidth = 1, zorder=0, plot_center_line: bool = False):
        if ax is None:
            ax = plt.gca()
        # Inner curve.
        ax.plot(
            self.track_bound[0, :], self.track_bound[1, :], c=c, linestyle='-',
            linewidth = linewidth,
            zorder=zorder
        )
        # Outer curve.
        ax.plot(
            self.track_bound[2, :], self.track_bound[3, :], c=c, linestyle='-',
            zorder=zorder
        )
        if plot_center_line:
            self.plot_track_center(ax, c=c, zorder=zorder)

    def plot_track_center(self, ax: Optional[matplotlib.axes.Axes] = None, c: str = 'k', linewidth = 1, zorder=0):
        if ax is None:
            ax = plt.gca()
        ax.plot(
            self.track_center[0, :], self.track_center[1, :], c=c, linestyle='--',
            linewidth = linewidth,
            zorder=zorder
        )

    # endregion

class RealtimeBuffer:
    '''
    This class implements a real-time buffer for a single object.
    '''
    def __init__(self):
        self.rt_obj = None
        self.non_rt_obj = None
        self.new_data_available = False
        self.lock = Lock()
        
    def writeFromNonRT(self, obj):
        '''
        Write data to non-realtime object. If a real-time thread 
        is reading the non-realtime object, wait until it finish.
        
        Parameters:
            obj: object to be written
        '''
        self.lock.acquire(blocking=True)
        self.non_rt_obj = obj
        self.new_data_available = True
        self.lock.release()
        
    def readFromRT(self):
        '''
        if no thread is writing and new data is available, update rt-object 
        with non-rt object.
        
        Returns:
            rt_obj: real-time object
        '''
        # try to lock
        if self.lock.acquire(blocking=False):
            if self.new_data_available:
                temp = self.rt_obj
                self.rt_obj = self.non_rt_obj
                self.non_rt_obj = temp
                self.new_data_available = False
            self.lock.release()
        return self.rt_obj
    
    def reset(self):
        '''
        Reset the buffer to None
        '''
        self.writeFromNonRT(None)
