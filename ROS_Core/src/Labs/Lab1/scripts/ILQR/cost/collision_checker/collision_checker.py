from typing import Union
import jax
from functools import partial
import jax.numpy as jnp
from jaxlib.xla_extension import DeviceArray
import hppfcl
import numpy as np
import time
import warnings
class CollisionChecker:
    def __init__(self, config) -> None:
        
        self.width = config.width
        self.length = config.length
        self.wheelbase = config.wheelbase
        self.step = config.T
        
        # create a box for the vehicle
        # The vehicle is centered at its rear axle
        verts = hppfcl.StdVec_Vec3f()
        verts.extend(
            [
                np.array([-(self.length - self.wheelbase)/2.0, self.width/2.0, 100]),
                np.array([-(self.length - self.wheelbase)/2.0, -self.width/2.0, 100]),
                np.array([(self.length + self.wheelbase)/2.0, self.width/2.0, 100]),
                np.array([(self.length + self.wheelbase)/2.0+0.15, self.width/2.0, 100]),
                np.array([(self.length + self.wheelbase)/2.0, -self.width/2.0, 100]),
                np.array([-(self.length - self.wheelbase)/2.0, self.width/2.0, -100]),
                np.array([-(self.length - self.wheelbase)/2.0, -self.width/2.0, -100]),
                np.array([(self.length + self.wheelbase)/2.0, self.width/2.0, -100]),
                np.array([(self.length + self.wheelbase)/2.0+0.15, self.width/2.0, -100]),
                np.array([(self.length + self.wheelbase)/2.0, -self.width/2.0, -100]),
            ]
        )
        self.ego_object = hppfcl.Convex.convexHull(verts, False, None)
        self.ego_collision_object = hppfcl.CollisionObject(self.ego_object, np.eye(3), np.zeros(3))
    
    @partial(jax.jit, static_argnums=(0,))
    def _gen_pose(self, state):
        '''
        state: a 1D numpy array of shape (5,), (x, y, z, psi, v) 
        '''
        psi = state[3]
        cp = jnp.cos(psi)
        sp = jnp.sin(psi)
        ego_rotm = jnp.array([[cp, -sp],
                            [sp, cp]])
        ego_trans = state[:2]
        
        ego_rotm_inv = jnp.array([[cp, sp, 0],
                                [-sp, cp, 0],
                                [0,0,1]])
        ego_trans_inv = jnp.array([-(cp*state[0] + sp*state[1]),
                                (sp*state[0] - cp*state[1]), 0])
        return ego_rotm, ego_trans, ego_rotm_inv, ego_trans_inv
            
    def _check_collision(self, state: np.ndarray, polytope: hppfcl.ConvexBase) -> np.ndarray:
        """
        Check collision between the ego vehicle and a polytope
        args:
            
        state: a 1D numpy array of shape (5,), (x, y, z, psi, v) the ego vehicle state in the global coordinate frame
        polytope: a hppfcl collision object whose vertices are in the global coordinate frame
        """
        # get the ego vehicle pose 
        psi = state[3]
        cp = np.cos(psi)
        sp = np.sin(psi)
        ego_rotm = np.array([[cp, -sp],
                            [sp, cp]])
        ego_trans = state[:2]
        ego_rotm_inv = np.array([[cp, sp, 0],
                                [-sp, cp, 0],
                                [0,0,1]])
        ego_trans_inv = np.array([-(cp*state[0] + sp*state[1]),
                                (sp*state[0] - cp*state[1]), 0])
        
        # create a collision object for the polytope in the ego vehicle frame
        collision_object = hppfcl.CollisionObject(polytope, ego_rotm_inv, ego_trans_inv)
        # check collision between the ego vehicle and the obstacles
        request = hppfcl.DistanceRequest()
        result = hppfcl.DistanceResult()
        hppfcl.distance(self.ego_collision_object, collision_object, request, result)
        ego_point = result.getNearestPoint1()[:2]
        obj_point = result.getNearestPoint2()[:2]
        
        obj_point_global = ego_rotm@obj_point + ego_trans
        
        distance  = result.min_distance
        if distance < -1e10:
            distance = -0.01
        
        output = np.array([ego_point[0], ego_point[1], obj_point_global[0], obj_point_global[1], distance])
        if np.any(np.isnan(output)):
            print("NAN in collision")
        return output
    
    def check_collisions(self, state: np.ndarray, obstacles: list) -> np.ndarray:
        """
        Check collision between the ego vehicle and a list of obstacles
        args:
            
        state: a 2D numpy array of shape (5,n), 
            each column is (x, y, z, psi, v) the ego vehicle state in the global coordinate frame
        obstacles: a list of Obstacle objects
        
        returns:
        
        collision_ref: a 3D numpy array of shape (num_obstacles, 5, n)
        """
        if not isinstance(state, np.ndarray):
            warnings.warn(f"'obs_refs' is a class of {type(state)} instead of <an np.ndarray>. "+ \
                "There maybe performance issue due to sliceing []")
        
        num_obstacles = len(obstacles)
        # No obstacles return None
        if num_obstacles == 0:
            return None
        
        # initialize the collision_ref for the ego vehicle
        collision_ref = np.zeros((num_obstacles, 5, self.step))

        for i, obstacle in enumerate(obstacles):
            for t in range(self.step):
                collision_ref[i,:, t] = self._check_collision(state[:,t], obstacle.at(t))
        return collision_ref