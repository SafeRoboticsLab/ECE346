import hppfcl
import numpy as np
from typing import Union

class Obstacle:
    def __init__(self, vertices: Union[np.ndarray, list]) -> None:
        '''
        Constuct a time-varying obstacle from a list of vertices
        vertices: LIST: a list of 2D/3D numpy arrays of shape (n, 2 or 3)
        '''
        self.time_varying = isinstance(vertices, list)
        if self.time_varying:
            self.steps = len(vertices)
        else:
            self.steps = np.inf
            vertices = [vertices]
        self.time_varying_obstacle = []
        for v in vertices:
            self.time_varying_obstacle.append(self._construct_collision_object(v))        
        
    def _construct_collision_object(self, vertices):
        """
        Construct a collision object from a list of vertices
        vertices: a 2D numpy array of shape (n, 2)
        """
        num_points = vertices.shape[0]
        dim = vertices.shape[1]
        
        assert num_points >= 3, f"The number of vertices ({num_points}) must be at least 3"
        # assert dim == 2, f"The dimension of the vertices ({dim}) must be 2"
        verts = hppfcl.StdVec_Vec3f()
        if dim == 2:
            # add a column of ones to the polygon
            vertices = np.append(vertices, np.ones((num_points, 1)), axis=1)
            verts.extend(list(vertices))
            vertices[:,-1] = -1
            verts.extend(list(vertices))
        else:
            verts.extend(list(vertices))
        # create a polytope object
        polygon = hppfcl.Convex.convexHull(verts, True, None)
        
        return polygon 
    
    def at(self, step):
        assert step < self.steps, f"The step {step} is out of range [0, {self.steps-1}]"
        if not self.time_varying:
            step = 0
        return self.time_varying_obstacle[step]