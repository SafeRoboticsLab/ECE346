'''
Utilities for testing

Stanley Bak, 2018
'''

import numpy as np
from scipy.sparse import csc_matrix
from scipy.sparse.linalg import expm, expm_multiply

class Freezable():
    'a class where you can freeze the fields (prevent new fields from being created)'

    _frozen = False

    def freeze_attrs(self):
        'prevents any new attributes from being created in the object'
        self._frozen = True

    def __setattr__(self, key, value):
        if self._frozen and not hasattr(self, key):
            raise AttributeError("{} does not contain attribute '{}' (object was frozen)".format(self, key))

        object.__setattr__(self, key, value)

def compress_init_box(input_box, tol=1e-9):
    '''compress all constant values in the init set into a single input

    returns init_bm, init_bias, new_input_box
    '''

    inputs = len(input_box)

    cur_bias = np.array([0] * inputs, dtype=float)

    cur_bm_transpose = []
    new_input_box = []

    var_index = 0
    
    for dim, (lb, ub) in enumerate(input_box):
        mid = (lb + ub) / 2.0
            
        if abs(ub-lb) < tol:
            # equal, update cur_bias
            cur_bias[dim] = mid
        else:
            new_input_box.append((lb, ub))
            
            # add column from identity matrix to cur_bm
            cur_bm_transpose.append([1 if d == dim else 0 for d in range(inputs)])

            var_index += 1
    
    cur_bm = np.array(cur_bm_transpose, dtype=float).transpose()

    return cur_bm, cur_bias, new_input_box

def to_discrete_time_mat(a_mat, b_mat, dt, quick=False):
    'convert an a and b matrix to a discrete time version'

    rv_a = None
    rv_b = None

    if quick:
        if not isinstance(a_mat, np.ndarray):
            a_mat = np.array(a_mat, dtype=float)
        
        rv_a = np.identity(a_mat.shape[0], dtype=float) + a_mat * dt

        if b_mat is not None:
            if not isinstance(b_mat, np.ndarray):
                b_mat = np.array(b_mat, dtype=float)
            
            rv_b = b_mat * dt
    else:
        # first convert both to csc matrices
        a_mat = csc_matrix(a_mat, dtype=float)
        dims = a_mat.shape[0]

        rv_a = expm(a_mat * dt)

        rv_a = rv_a.toarray()

        if b_mat is not None:
            b_mat = csc_matrix(b_mat, dtype=float)

            rv_b = np.zeros(b_mat.shape, dtype=float)

            inputs = b_mat.shape[1]

            for c in range(inputs):
                # create the a_matrix augmented with a column of the b_matrix as an affine term
                indptr = b_mat.indptr

                data = np.concatenate((a_mat.data, b_mat.data[indptr[c]:indptr[c+1]]))
                indices = np.concatenate((a_mat.indices, b_mat.indices[indptr[c]:indptr[c+1]]))
                indptr = np.concatenate((a_mat.indptr, [len(data)]))

                aug_a_csc = csc_matrix((data, indices, indptr), shape=(dims + 1, dims + 1))

                mat = aug_a_csc * dt

                # the last column of matrix_exp is the same as multiplying it by the initial state [0, 0, ..., 1]
                init_state = np.zeros(dims + 1, dtype=float)
                init_state[dims] = 1.0

                col = expm_multiply(mat, init_state)

                rv_b[:, c] = col[:dims]

    return rv_a, rv_b
