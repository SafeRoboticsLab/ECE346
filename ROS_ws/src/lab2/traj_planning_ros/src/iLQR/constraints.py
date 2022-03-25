from typing import List
import numpy as np
from .ellipsoid_obj import EllipsoidObj
import time

class Constraints:

    def __init__(self, params):
        # load parameters
        self.T = params['T']  # Planning Time Horizon
        self.N = params['N']  # number of planning steps
        self.dt = self.T / (self.N - 1)  # time step for each planning step

        self.wheelbase = params['wheelbase']  # vehicle chassis length
        self.delta_min = params['delta_min']  # min steering angle rad
        self.delta_max = params['delta_max']  # max steering angle rad
        self.a_min = params['a_min']  # min longitudial accel
        self.a_max = params['a_max']  # max longitudial accel
        self.v_min = params['v_min']  # min velocity
        self.v_max = params['v_max']  # max velocity
        self.alat_max = params['alat_max']  # max lateral accel
        self.alat_min = -params['alat_max']  # min lateral accel
        self.track_width_L = params['track_width_L']
        self.track_width_R = params['track_width_R']

        # parameter for barrier functions
        self.q1_v = params['q1_v']
        self.q2_v = params['q2_v']

        self.q1_road = params['q1_road']
        self.q2_road = params['q2_road']

        self.q1_lat = params['q1_lat']
        self.q2_lat = params['q2_lat']

        self.q1_obs = params['q1_obs']
        self.q2_obs = params['q2_obs']

        # useful constants
        self.zeros = np.zeros((self.N))
        self.ones = np.ones((self.N))

        self.gamma = 0.9
        ''' ego system'''
        ego_a = params['length'] / 2.0
        self.r = ego_b = params['width'] / 2.0
        wheelbase = params['wheelbase']
        ego_q = np.array([wheelbase / 2, 0])[:, np.newaxis]
        ego_Q = np.diag([ego_a * ego_a, ego_b * ego_b])
        self.ego = EllipsoidObj(q=ego_q, Q=ego_Q)
        self.ego_ell = None
        ''' obstacles represented as FRS'''
        self.obs_list = None

    def update_obs(self, frs_list):
        self.obs_list = frs_list

    def state2ell(self, states: np.ndarray) -> List[EllipsoidObj]:
        ego_ell = []
        for i in range(self.N):
            theta = states[3, i]
            d = states[:2, i][:, np.newaxis]
            R = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
            temp = self.ego @ R
            temp.add(d)
            ego_ell.append(temp)
            
        return ego_ell

    def get_cost(self, states: np.ndarray, controls: np.ndarray,
                 closest_pt: np.ndarray, slope: np.ndarray) -> np.ndarray:
              
        # Road Boundary constarint
        c_boundary = self._road_boundary_cost(states, closest_pt, slope)

        # Obstacle constraints
        c_obs = np.zeros(self.N)
        if len(self.obs_list)>0:
            # Patch footprint around state trajectory
            self.ego_ell = self.state2ell(states)
            for i in range(self.N):
                ego_i = self.ego_ell[i]
                for obs_j in self.obs_list:  # obs_j is a list of obstacles.
                    obs_j_i = obs_j[i]  # Get the ith obstacle in list obs_j.
                    c_obs[i] += self.gamma**i * ego_i.obstacle_cost(
                        obs_j_i, self.q1_obs, self.q2_obs)

        # Minimum velocity constarint
        c_vel = self.q1_v * (np.exp(-states[2, :] * self.q2_v))

        # Lateral Acceleration constraint
        accel = states[2, :]**2 * np.tan(controls[1, :]) / self.wheelbase
        error_ub = accel - self.alat_max
        error_lb = self.alat_min - accel

        b_ub = self.q1_lat * (np.exp(self.q2_lat * error_ub) - 1)
        b_lb = self.q1_lat * (np.exp(self.q2_lat * error_lb) - 1)
        c_lat = b_lb + b_ub

        return c_vel + c_boundary + c_lat + c_obs

    def get_derivatives(self, states: np.ndarray, controls: np.ndarray,
                        closest_pt: np.ndarray,
                        slope: np.ndarray) -> np.ndarray:
        '''
    Calculates the Jacobian and Hessian of soft constraint cost.

    Args:
        states: 4xN array of planned trajectory
        controls: 2xN array of planned control
        closest_pt: 2xN array of each state's closest point [x,y] on the track
        slope: 1xN array of track's slopes (rad) at closest points
    '''

        # lateral acceleration constraints
        c_x_lat, c_xx_lat, c_u_lat, c_uu_lat, c_ux_lat = \
            self._lat_accec_bound_derivative(states, controls)

        # road bound constraints
        c_x_rd, c_xx_rd = self._road_boundary_derivatie(
            states, closest_pt, slope)

        # obstacle constraints
        c_x_obs = np.zeros_like(c_x_lat)
        c_xx_obs = np.zeros_like(c_xx_lat)
        if len(self.obs_list)>0:
            for i in range(self.N):
                ego_i = self.ego_ell[i]
                for obs_j in self.obs_list:
                    obs_j_i = obs_j[i]
                    c_x_obs_temp, c_xx_obs_temp = ego_i.obstacle_derivative(
                        states[:, i], self.wheelbase / 2, obs_j_i, self.q1_obs,
                        self.q2_obs)
                    c_x_obs[:, i] += self.gamma**i * c_x_obs_temp
                    c_xx_obs[:, :, i] += self.gamma**i * c_xx_obs_temp

        # min velocity
        c_x_vel, c_xx_vel = self._velocity_bound_derivatie(states)

        # sum up
        c_x_cons = c_x_rd + c_x_lat + c_x_obs + c_x_vel
        c_xx_cons = c_xx_rd + c_xx_lat + c_xx_obs + c_xx_vel

        c_u_cons = c_u_lat
        c_uu_cons = c_uu_lat
        c_ux_cons = c_ux_lat

        return c_x_cons, c_xx_cons, c_u_cons, c_uu_cons, c_ux_cons

    def _velocity_bound_derivatie(self, states: np.ndarray) -> np.ndarray:
        '''
    Calculates the Jacobian and Hessian of velocity soft constraint cost.

    Args:
        states: 4xN array of planned trajectory
    '''
        transform = np.array([self.zeros, self.zeros, self.ones, self.zeros])

        # larger than 0
        c = -states[2, :]
        c_x_l, c_xx_l = self.barrier_function(self.q1_v, self.q2_v, c,
                                              -transform)

        return c_x_l, c_xx_l

    def _lat_accec_bound_derivative(self, states: np.ndarray,
                                    controls: np.ndarray) -> np.ndarray:
        '''
    Calculates the Jacobian and Hessian of Lateral Acceleration soft constraint
        cost.

    Args:
        states: 4xN array of planned trajectory
        controls: 2xN array of planned control
    '''
        c_x = np.zeros((4, self.N))
        c_xx = np.zeros((4, 4, self.N))
        c_u = np.zeros((2, self.N))
        c_uu = np.zeros((2, 2, self.N))
        c_ux = np.zeros((2, 4, self.N))

        # calculate the acceleration
        accel = states[2, :]**2 * np.tan(controls[1, :]) / self.wheelbase

        error_ub = accel - self.alat_max
        error_lb = self.alat_min - accel

        b_ub = self.q1_lat * np.exp(np.clip(self.q2_lat * error_ub, None, 20))
        b_lb = self.q1_lat * np.exp(np.clip(self.q2_lat * error_lb, None, 20))

        da_dx = 2 * states[2, :] * np.tan(controls[1, :]) / self.wheelbase
        da_dxx = 2 * np.tan(controls[1, :]) / self.wheelbase

        da_du = states[2, :]**2 / (np.cos(controls[1, :])**2 * self.wheelbase)
        da_duu = (states[2, :]**2 * np.sin(controls[1, :]) /
                  (np.cos(controls[1, :])**3 * self.wheelbase))

        da_dux = 2 * states[2, :] / (np.cos(controls[1, :])**2 *
                                     self.wheelbase)

        c_x[2, :] = self.q2_lat * (b_ub - b_lb) * da_dx
        c_u[1, :] = self.q2_lat * (b_ub - b_lb) * da_du

        c_xx[2, 2, :] = self.q2_lat**2 * (
            b_ub + b_lb) * da_dx**2 + self.q2_lat * (b_ub - b_lb) * da_dxx
        c_uu[1, 1, :] = self.q2_lat**2 * (
            b_ub + b_lb) * da_du**2 + self.q2_lat * (b_ub - b_lb) * da_duu

        c_ux[1, 2, :] = (self.q2_lat**2 * (b_ub + b_lb) * da_dx * da_du +
                         self.q2_lat * (b_ub - b_lb) * da_dux)
        return c_x, c_xx, c_u, c_uu, c_ux

    def _road_boundary_cost(self, states: np.ndarray, closest_pt: np.ndarray,
                            slope: np.ndarray) -> np.ndarray:
        dx = states[0, :] - closest_pt[0, :]
        dy = states[1, :] - closest_pt[1, :]

        sr = np.sin(slope)
        cr = np.cos(slope)
        dis = sr * dx - cr * dy
        ''' right bound'''
        b_r = dis - (self.track_width_R - self.r)

        c_r = self.q1_road * np.exp(
            np.clip(self.q2_road * b_r, -0.025 * self.q2_road, 20))
        ''' Left Bound'''
        b_l = -dis - (self.track_width_L - self.r)

        c_l = self.q1_road * np.exp(
            np.clip(self.q2_road * b_l, -0.025 * self.q2_road, 20))

        return c_l + c_r

    def _road_boundary_derivatie(self, states: np.ndarray,
                                 closest_pt: np.ndarray,
                                 slope: np.ndarray) -> np.ndarray:
        '''
    Calculates the Jacobian and Hessian of road boundary soft constraint cost.

    Args:
        states: 4xN array of planned trajectory
        closest_pt: 2xN array of each state's closest point [x,y] on the track
        slope: 1xN array of track's slopes (rad) at closest points
    '''
        # constraint due to right road boundary. smaller than right_width

        n = states.shape[-1]
        dx = states[0, :] - closest_pt[0, :]
        dy = states[1, :] - closest_pt[1, :]

        sr = np.sin(slope)
        cr = np.cos(slope)
        dis = sr * dx - cr * dy
        ''' right bound'''
        b_r = dis - (self.track_width_R - self.r)
        idx_ignore = b_r < -0.025
        c_r = self.q1_road * np.exp(np.clip(self.q2_road * b_r, None, 20))

        # Jacobian
        c_x_r = np.zeros((4, n))
        c_x_r[0, :] = self.q2_road * c_r * sr
        c_x_r[1, :] = -self.q2_road * c_r * cr
        # c_x_r[3,:] = -center_L*q2*np.cos(theta - self.slope)*c_r

        c_xx_r = np.zeros((4, 4, n))
        c_xx_r[0, 0, :] = self.q2_road * self.q2_road * c_r * sr * sr
        c_xx_r[1, 1, :] = self.q2_road * self.q2_road * c_r * cr * cr

        c_xx_r[0, 1, :] = c_xx_r[
            1, 0, :] = -self.q2_road * self.q2_road * c_r * cr * sr  #x y

        # remove inactive
        c_x_r[:, idx_ignore] = 0
        c_xx_r[:, :, idx_ignore] = 0
        ''' Left Bound'''
        b_l = -dis - (self.track_width_L - self.r)
        idx_ignore = b_l < -0.025
        c_l = self.q1_road * np.exp(np.clip(self.q2_road * b_l, None, 20))

        # Jacobian
        c_x_l = np.zeros((4, n))
        c_x_l[0, :] = -self.q2_road * c_l * sr
        c_x_l[1, :] = self.q2_road * c_l * cr
        # c_x_l[3,:] = center_L*q2*np.cos(theta - self.slope)*c_l

        c_xx_l = np.zeros((4, 4, n))
        c_xx_l[0, 0, :] = self.q2_road * self.q2_road * c_l * sr * sr
        c_xx_l[1, 1, :] = self.q2_road * self.q2_road * c_l * cr * cr

        c_xx_l[0, 1, :] = c_xx_l[
            1, 0, :] = -self.q2_road * self.q2_road * c_l * cr * sr  #x y

        # # remove inactive
        c_x_l[:, idx_ignore] = 0
        c_xx_l[:, :, idx_ignore] = 0

        c_x = c_x_r + c_x_l
        c_xx = c_xx_r + c_xx_l

        return c_x, c_xx

    def barrier_function(self, q1: float, q2: float, c: np.ndarray,
                         c_dot: np.ndarray) -> np.ndarray:
        '''
    c = [n] array
    c_dot = [dxn] array
    '''
        b = q1 * (np.exp(np.clip(q2 * c, None, 20)))
        # b = q1*np.exp(q2*c)
        b_dot = np.einsum('n,an->an', q2 * b, c_dot)
        b_ddot = np.einsum('n,abn->abn', (q2**2) * b,
                           np.einsum('an,bn->abn', c_dot, c_dot))
        return b_dot, b_ddot
