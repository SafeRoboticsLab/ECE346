from typing import Optional, Tuple, Union
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
from pyspline.pyCurve import Curve


class RefPath:
    def __init__(self, center_line: np.ndarray, width_left: Union[np.ndarray, float],
                    width_right: Union[np.ndarray, float], speed_limt: Union[np.ndarray, float], loop: Optional[bool] = True) -> None:
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
            eps: Optional[float] = 1e-5) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        
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

    def get_closest_pts(self, points: np.ndarray, eps: Optional[float] = 1e-5) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Gets the closest points on the centerline, the slope of their tangent
        lines, and the progress given the points in the global frame.

        Args:
            points (np.ndarray): the points in the global frame, of the shape
                (2, N).

        Returns:
            np.ndarray: the position of the closest points on the centerline. This
                array is of the shape (2, N).
            np.ndarray: the slope of of trangent line on those points. This vector
                is of the shape (1, N).
            np.ndarray: the normalized progress along the centerline. This vector is of the
                shape (1, N).
        """
        if len(points.shape) == 1:
            points = points[:, np.newaxis]
            
        s, _ = self.center_line.projectPoint(points.T, eps=eps)
        
        if points.shape[1] == 1:
            s = np.array([s])
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
    
    