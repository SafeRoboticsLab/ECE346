'''simulation utilities for quickzonoreach'''

import numpy as np
from scipy.integrate import solve_ivp

def get_center_sim(init_box, a_mat_list, b_mat_list, input_box_list, dt_list):
    '''get the discrete-time center simulation (good for correctness validation

    this is not optimized for performance

    returns a list of points (np.arrays)
    '''

    assert len(a_mat_list) == len(b_mat_list) == len(input_box_list) == len(dt_list), "all lists should be same length"

    rv = []
    state = np.array([(lb + ub) / 2 for lb, ub in init_box])

    rv.append(state.copy())

    for a_mat, b_mat, input_box, dt in zip(a_mat_list, b_mat_list, input_box_list, dt_list):

        if b_mat is not None:
            center_input = np.array([(lb + ub) / 2 for lb, ub in input_box])
            input_effect = np.dot(b_mat, center_input)
        else:
            input_effect = None

        state = sim_step(state, a_mat, input_effect, dt)
        rv.append(state)

    return np.array(rv, dtype=float)

def sim_step(state, a_mat, input_effect, dt):
    'simulate for a single time step and return the new state'

    def der(time, state):
        'derivative function'

        assert isinstance(state, np.ndarray)
        assert isinstance(time, float)

        rv = np.dot(a_mat, state)

        if input_effect is not None:
            rv += input_effect

        return rv

    # simulate from state for dt time
    times = [0, dt]
    res = solve_ivp(der, times, state, t_eval=[dt])
    assert res.success, f"solver failed, status was {res.status}: {res.message}"

    return np.ravel(res.y)

    
