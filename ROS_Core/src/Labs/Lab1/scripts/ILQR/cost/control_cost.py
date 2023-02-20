from abc import ABC, abstractmethod
from functools import partial
from jaxlib.xla_extension import DeviceArray
import jax
from jax import numpy as jnp
import numpy as np

from .base_cost import BaseCost
from .base_cost import quadratic_cost, huber_cost


class ControlCost(BaseCost):
    def __init__(self, config):
        super().__init__()
        self.weight = jnp.array([config.ctrl_cost_accel_weight,
                                config.ctrl_cost_steer_weight]) # shape of (dim_u)
        
        self.delta = jnp.array([config.ctrl_cost_accel_huber_delta,
                                config.ctrl_cost_steer_huber_delta]) # shape of (dim_u)
        
        if config.ctrl_cost_type == 'quadratic':
            self.cost_func = quadratic_cost
        elif config.ctrl_cost_type == 'huber':
            self.cost_func = huber_cost
        else:
            raise NotImplementedError(
                f'Cost type {config.ctrl_cost_type} not implemented for CTRL COST. '+
                'Please choose from [quadratic, huber]'
                )
    
    
    @partial(jax.jit, static_argnums=(0,))
    def get_running_cost(
			self, state: DeviceArray, ctrl: DeviceArray, ref: DeviceArray
	) -> float:
        '''
        Given a state, control, and time index, return the cost.
        Input:
            state: (dim_x) state
            ctrl: (dim_u) control
            ref: (dim_ref) reference 
        return:
            cost: float
        '''
        return jnp.sum(self.cost_func(ctrl, self.weight, self.delta))