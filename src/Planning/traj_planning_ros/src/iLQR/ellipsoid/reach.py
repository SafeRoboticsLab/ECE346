import numpy as np
from .ellipsoid import Ellipsoid


class Reach():
  """
  The Reach class. Computes reachable sets of the dynamical system.
  Author: Haimin Hu (haiminh@princeton.edu)
  Reference: Ellipsoidal Toolbox (MATLAB) by Dr. Alex Kurzhanskiy.
  """

  def __init__(
      self, sys, X0, D=Ellipsoid(), U=Ellipsoid(), utype='openloop',
      dtype='minmax'
  ):
    """
    Constructor for Reach object.
    Args:
        sys (DynSys): dynamical system.
        X0 (Ellipsoid): initial set.
        D (Ellipsoid or list): disturbance set(s).
        U (Ellipsoid or list): control set(s).
        utype (str): control option ('openloop' or 'closedloop')
        dtype (str): disturbance option ('minmax' or 'maxmin').
    Todo list:
      - Backward reachable sets computation.
    """

    self.sys = sys
    if not isinstance(X0, Ellipsoid):
      raise ValueError("[ellReach-Reach] initial set X0 must be an ellipsoid.")
    self.X0 = X0
    if not isinstance(U, Ellipsoid):
      raise ValueError("[ellReach-Reach] control set U must be an ellipsoid.")
    self.U = U
    self.utype = utype
    if not isinstance(D, Ellipsoid):
      raise ValueError(
          "[ellReach-Reach] disturbance set D must be an ellipsoid."
      )
    self.D = D
    self.dtype = dtype

  def FRS(self, T, L=[]):
    """
    Computes forward reachable sets.
    Args:
        T (int): horizon.
        L (list of np.ndarray): list of direction vectors.
    Returns:
        list of ellipsoids: ellipsoidal forward reachable sets.
    """
    E_list = [self.X0]
    # Discrete-time linear systems
    if self.sys.sys_type == 'DTLTI' or self.sys.sys_type == 'DTLTV':
      if self.sys.sys_type == 'DTLTV' and T > self.sys.T:
        raise ValueError(
            "[ellReach-Reach] FRS horizon is larger than the system horizon."
        )
      if self.sys.autonomous():
        for k in range(0, T-1):
          if self.sys.time_varying():
            A = self.sys.A[k]
            c = self.sys.c[k]
            if not self.sys.no_dstb():
              G = self.sys.G[k]
            else:
              G = None
          else:
            A = self.sys.A
            c = self.sys.c
            G = self.sys.G
          E_now = E_list[-1].copy()
          # Autonomous part
          E_tmp = E_now@A + c
          # Disturbance part (try different minksum and pick the one with
          #  minimal volume)
          if not self.sys.no_dstb():
            D = self.D @ G

            if len(L) == 0:
              _, eigv = np.linalg.eig(E_tmp.Q)
              for i in range(self.X0.dim()):
                L.append(eigv[i, :].reshape(self.X0.dim(), 1))

            E_list_tmp = []
            V_list_tmp = []
            for l in L:
              E_next_tmp = E_tmp.minksum(D, l)
              E_list_tmp.append(E_next_tmp)
              V_list_tmp.append(E_next_tmp.volume())
            idx = V_list_tmp.index(min(V_list_tmp))
            E_tmp = E_list_tmp[idx]
          E_list.append(E_tmp)
      else:
        raise NotImplementedError
    else:
      raise NotImplementedError
    return E_list
