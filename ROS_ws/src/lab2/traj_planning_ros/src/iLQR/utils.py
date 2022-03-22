from typing import Optional, List
import numpy as np
import matplotlib

from .ellipsoid import Ellipsoid, DynSys, Reach, plot_ellipsoids
from .ellipsoid_obj import EllipsoidObj


def plot_footprint(
    ax: matplotlib.axes.Axes, states: np.ndarray,
    agent: Optional[Ellipsoid] = None, color: str = "r"
) -> None:
  if agent is None:
    a = 0.335
    b = 0.13
    agent = Ellipsoid(
        q=np.zeros((2, 1)), Q=np.array([[a * a, 0.0], [0.0, b * b]])
    )
  horizon = states.shape[1]

  ellipsoids = []
  for k in range(horizon):
    theta = states[3, k]
    rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
    ellipsoids.append(agent@rot_mat + states[:2, k:k + 1])

  arg_list = [dict(c=color)]
  plot_ellipsoids(ax, ellipsoids, arg_list=arg_list, N=50)
  return


def state2ell(state: np.array, agent: EllipsoidObj) -> EllipsoidObj:
  theta = state[3]
  position = state[:2][:, np.newaxis]
  rotation = np.array([[np.cos(theta), -np.sin(theta)],
                       [np.sin(theta), np.cos(theta)]])
  return agent@rotation + position


def get_agent_FRS(
    As: np.ndarray, Bs: np.ndarray, Ks: np.ndarray, states: np.ndarray,
    init_set: Optional[Ellipsoid] = None, dstb: Optional[Ellipsoid] = None,
    projection: Optional[List[int]] = [0, 1]
) -> List[EllipsoidObj]:
  """
  Computes the forward reachable set of an agent given its nominal trajectory,
  linearized dynamics, linear policy, initial set, footprint and the projected
  subspace.

  Args:
      As (np.ndarray): A matrices of dynamics, of the shape (x_dim, x_dim, N).
      Bs (np.ndarray): B matrices of dynamics, of the shape (x_dim, u_dim, N).
      Ks (np.ndarray): linear policy, of the shape (u_dim, x_dim, N).
      states (np.ndarray): nominal state trajectory, of the shape (x_dim, N).
      init_set (Optional[Ellipsoid], optional): initial set. Defaults to None.
      dstb (Optional[Ellipsoid], optional): 1D disturbance. Defaults to None.
      agent (Optional[Ellipsoid], optional): agent's footprint.
          Defaults to None.
      projection (Optional[List[int]], optional): dimension to be preserved in
          the projection. Defaults to [0, 1].

  Returns:
      List[EllipsoidObj]: a list of the over approximation of forward reachable
          sets with agent's footprint.
  """
  x_dim, _, horizon = Bs.shape
  if init_set is None:
    init_set = Ellipsoid(q=np.zeros((x_dim, 1)), Q=1e-3 * np.eye(x_dim))
  if dstb is None:
    dstb = Ellipsoid(q=np.zeros((4, 1)), Q=np.diag([1e-4, 1e-3, 1e-3, 1e-2]))
  else:
    assert dstb.q.shape[1] == 1, "We currently supports 1D disturbance!"

  obj_a = 0.25
  obj_b = 0.12
  L = 0.257
  # Define ego car footprint
  obj_q = np.array([L / 2, 0.])[:, np.newaxis]
  obj_Q = np.diag([obj_a**2, obj_b**2])

  E_ft = Ellipsoid(obj_q, obj_Q)

  # region: Constructs dynamic system.
  A_list = []
  G_list = []
  for k in range(horizon - 1):
    Ak = As[:, :, k]
    Bk = Bs[:, :, k]
    Kk = Ks[:, :, k]

    A_list.append(Ak + Bk@Kk)
    theta = states[3, k]
    c = np.cos(theta)
    s = np.sin(theta)

    G_k = 0.1 * np.array([[c, s, 0, 0], [-s, c, 0, 0], [0, 0, 1, 0],
                          [0, 0, 0, 1]])
    G_list.append(G_k)
  sys = DynSys(
      sys_type='DTLTV', A=A_list, B=np.array([]), c=np.array([]), G=G_list,
      T=horizon
  )
  reach_sys = Reach(sys, init_set, dstb)
  # endregion

  # region: Computes forward reachable set with footprint.
  FRS_sys = reach_sys.FRS(T=horizon)

  ellipsoids = []

  for k in range(horizon):

    E = FRS_sys[k] + states[:, k:k + 1]

    # Rotate ego car
    theta = states[3, k]
    rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
    E_ft_rotated = E_ft @ rot_mat

    # Ego car footprint minksum FRS
    E_aug = E.projection(projection).minksum_minVol(E_ft_rotated)
    ellipsoids.append(EllipsoidObj(E_aug))

  return ellipsoids
