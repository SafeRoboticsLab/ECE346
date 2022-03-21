from typing import Optional, Tuple
import numpy as np


class Dynamics:

  def __init__(self, params):
    self.dim_x = 4
    self.dim_u = 2

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

    # useful constants
    self.zeros = np.zeros((self.N))
    self.ones = np.ones((self.N))

  def forward_step(
      self, state: np.ndarray, control: np.ndarray, step: Optional[int] = 1,
      noise: Optional[np.ndarray] = None, noise_type: Optional[str] = 'unif'
  ) -> Tuple[np.ndarray, np.ndarray]:
    """
    Finds the next state of the vehicle given the current state and
    control input state.

    Args:
        state (np.ndarray): (4, ) array [X, Y, V, psi].
        control (np.ndarray): (2, ) array [a, delta].
        step (int, optional): The number of segements to forward the
            dynamics. Defaults to 1.
        noise (np.ndarray, optional): The ball radius or standard
            deviation of the Gaussian noise. The magnitude should be in the
            sense of self.dt. Defaults to None.
        noise_type(str, optional): Uniform or Gaussian. Defaults to 'unif'.

    Returns:
        np.ndarray: next state.
        np.ndarray: clipped control.
    """

    # Clips the controller values between min and max accel and steer values
    accel = np.clip(control[0], self.a_min, self.a_max)
    delta = np.clip(control[1], self.delta_min, self.delta_max)
    control_clip = np.array([accel, delta])
    next_state = state
    dt_step = self.dt / step

    for _ in range(step):
      # State: [x, y, v, psi]
      d_x = ((next_state[2] * dt_step + 0.5 * accel * dt_step**2)
             * np.cos(next_state[3]))
      d_y = ((next_state[2] * dt_step + 0.5 * accel * dt_step**2)
             * np.sin(next_state[3]))
      d_v = accel * dt_step
      d_psi = ((next_state[2] * dt_step + 0.5 * accel * dt_step**2)
               * np.tan(delta) / self.wheelbase)
      next_state = next_state + np.array([d_x, d_y, d_v, d_psi])
      if noise is not None:
        T = np.array([[np.cos(next_state[-1]),
                       np.sin(next_state[-1]), 0, 0],
                      [-np.sin(next_state[-1]),
                       np.cos(next_state[-1]), 0, 0], [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        if noise_type == 'unif':
          rv = np.random.rand(4) - 0.5
        else:
          rv = np.random.normal(size=(4))
        next_state = next_state + (T@noise) * rv / step

      # Clip the velocity
      next_state[2] = np.clip(next_state[2], 0, None)

    return next_state, control_clip

  def get_AB_matrix(self, nominal_states, nominal_controls):
    """
    Returns the linearized 'A' and 'B' matrix of the ego vehicle around
    nominal states and controls

      nominal_states: 4xN array
      nominal_controls: 2xN array
    """
    v = nominal_states[2, :]
    psi = nominal_states[3, :]
    accel = nominal_controls[0, :]
    delta = nominal_controls[1, :]

    A = np.empty((4, 4, self.N), dtype=float)
    A[0, :, :] = [
        self.ones, self.zeros,
        np.cos(psi) * self.dt,
        -(v * self.dt + 0.5 * accel * self.dt**2) * np.sin(psi)
    ]
    A[1, :, :] = [
        self.zeros, self.ones,
        np.sin(psi) * self.dt,
        (v * self.dt + 0.5 * accel * self.dt**2) * np.cos(psi)
    ]
    A[2, :, :] = [self.zeros, self.zeros, self.ones, self.zeros]
    A[3, :, :] = [
        self.zeros, self.zeros,
        np.tan(delta) * self.dt / self.wheelbase, self.ones
    ]

    B = np.empty((4, 2, self.N), dtype=float)
    B[0, :, :] = [self.dt**2 * np.cos(psi) / 2, self.zeros]
    B[1, :, :] = [self.dt**2 * np.sin(psi) / 2, self.zeros]
    B[2, :, :] = [self.dt * self.ones, self.zeros]
    B[3, :, :] = [
        np.tan(delta) * self.dt**2 / (2 * self.wheelbase),
        (v * self.dt + 0.5 * accel * self.dt**2) /
        (self.wheelbase * np.cos(delta)**2)
    ]

    return A, B
