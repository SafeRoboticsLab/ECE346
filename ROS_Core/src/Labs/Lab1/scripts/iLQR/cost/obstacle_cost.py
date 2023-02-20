from abc import ABC, abstractmethod
from functools import partial
from jaxlib.xla_extension import DeviceArray
import jax
from jax import numpy as jnp
import numpy as np

from .base_cost import BaseCost
from .base_cost import exp_linear_cost
import warnings

class SingleObstacleCost(BaseCost):
    def __init__(self, config):
        '''
        Obtain the cost and its gradient w.r.t to a single obstacle given a state, control,
        '''
        super().__init__()
        
                                                
        self.cost_func = exp_linear_cost
        
        self.obs_a = config.obs_a
        self.obs_b = config.obs_b

    @partial(jax.jit, static_argnums=(0,))
    def get_running_cost(
			self, state: DeviceArray, ctrl: DeviceArray, ref: np.ndarray
	) -> float:
        '''
        Given a state, control, and time index, return the cost.
        Input:
            state: (dim_x) state [x, y, v, psi, delta]
            ctrl: (dim_u) control
            ref: (5,) obstacle reference 
            time_idx: int (1)
        return:
            cost: float
        '''
        psi = state[3]
        rot_mat = jnp.array([[jnp.cos(psi), -jnp.sin(psi)],
                            [jnp.sin(psi), jnp.cos(psi)]])
        
        ego_pt = ref[:2]
        obs_pt = ref[2:4]
        dis_ref = ref[4]
        
        ego_pt_global = jnp.matmul(rot_mat, ego_pt) + state[:2]
        
        diff = obs_pt - ego_pt_global

        # handle the case when distance is 0 (when the obstacle touch each other)
        # THis leads to nan in the gradient
        dis = jnp.linalg.norm(diff)
        y = jnp.where(dis >= 1e-4, dis, 1e-4)

        y= -1*jnp.sign(dis_ref)*y   

        # Do not calrlate gradient w.r.t velocity
        v_no_grad = jax.lax.stop_gradient(state[2])
        obs_b = self.obs_b#*jnp.maximum(v_no_grad, 1)
        
        return exp_linear_cost(y, self.obs_a, obs_b)
    
class ObstacleCost():
    def __init__(self, config):
        self.single_obstacle_cost = SingleObstacleCost(config)
        
    def get_traj_cost(
			self, trajectory: DeviceArray, controls: DeviceArray, obs_refs: np.ndarray
	) -> float:
        '''
		Given a state, control, and time index, return the sum of the cost.
		Input:
			trajectory: (dim_x, N) List of trajectory
			controls: (dim_u, N) List of controls
			path_refs: (num_obstacle, dim_ref, N) List of references
		return:
			cost: float
		'''
        cost = 0
        if obs_refs is not None:
            if not isinstance(obs_refs, np.ndarray):
                warnings.warn(f"'obs_refs' is a class of {type(obs_refs)} instead of <an np.ndarray>. "+ \
                    "There maybe performance issue due to sliceing []")
            num_obstacle = obs_refs.shape[0]
            for i in range(num_obstacle):
                cost += self.single_obstacle_cost.get_traj_cost(trajectory, controls, obs_refs[i, :, :])
        return cost

    def get_derivatives_jax(
            self, trajectory: DeviceArray, controls: DeviceArray, obs_refs: DeviceArray
    ) -> DeviceArray:
        
        q = 0
        r = 0
        Q = 0
        R = 0
        H = 0
        
        if obs_refs is not None:
            num_obstacle = obs_refs.shape[0]
            
            for i in range(num_obstacle):
                cx_, r_, Q_, R_, H_ = self.single_obstacle_cost.get_derivatives_jax(trajectory, controls, obs_refs[i])
                q += cx_
                r += r_
                Q += Q_
                R += R_
                H += H_
        return (q, r, Q, R, H)