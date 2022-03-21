import numpy as np


class DynSys():
  """
  The dynamical system class.
  Author: Haimin Hu (haiminh@princeton.edu)
  Reference: Ellipsoidal Toolbox (MATLAB) by Dr. Alex Kurzhanskiy.
  Supports:
    DTLTI:  Discrete-time linear time-invariant system.
            x[k+1]  =  A x[k]  +  B u[k]  +  c  +  G d[k]
    DTLTV:  Discrete-time linear time-varying system.
            x[k+1]  =  A[k] x[k]  +  B[k] u[k]  +  c[k]  +  G[k] d[k]
    CTLTI:  Continuous-time linear time-invariant system (not yet implemented).
            dx/dt  =  A x(t)  +  B u(t)  +  c  +  G d(t)
    CTLTV:  Continuous-time linear time-varying system (not yet implemented).
            dx/dt  =  A(t) x(t)  +  B(t) u(t)  +  c(t)  +  G(t) d(t)
    NNCS:   Neural-network control system (not yet implemented).
    x - state, vector in R^n.
    u - control, vector in R^m.
    c - constant offset, vector in R^n.
    d - disturbance, vector in R^l.
    A - system matrix, in R^(nxn).
    B - control matrix, in R^(nxm).
    G - disturbance matrix, in R^(nxl).
  Todo list:
    - Accout for output map and noise: y(t)  =  C(t) x(t)  +  w(t).
  """

  def __init__(self, sys_type, A, B, c=np.array([]), G=np.array([]), T=0):
    """
    Constructor for dynamical system object.
    Args:
        sys_type (str): system type.
        A (np.ndarray or a list of np.ndarray): system matrix.
        B (np.ndarray or a list of np.ndarray): control matrix.
        c (np.ndarray or a list of np.ndarray, optional): offset vector.
        G (np.ndarray or a list of np.ndarray, optional): disturbance matrix.
        T (int): time horizon (for time-varying systems).
    """

    # Discrete-time linear time-invariant system (DTLTI).
    if sys_type == 'DTLTI':
      self.sys_type = 'DTLTI'
      # A matrix
      if not isinstance(A, np.ndarray):
        raise ValueError(
            "[ellReach-DynSys] A must be an np.ndarray for DTLTI systems."
        )
      n = A.shape[0]
      if n != A.shape[1]:
        raise ValueError("[ellReach-DynSys] A must be a square matrix.")
      self.A = A
      # B matrix
      if np.size(B) > 0:
        if not isinstance(B, np.ndarray):
          raise ValueError(
              "[ellReach-DynSys] B must be an np.ndarray for DTLTI systems."
          )
        if n != B.shape[0]:
          raise ValueError(
              "[ellReach-DynSys] Dimensions of A and B do not match."
          )
      self.B = B
      # c vector
      if np.size(c) == 0:
        self.c = np.zeros((n, 1))
      else:
        if not isinstance(c, np.ndarray):
          raise ValueError(
              "[ellReach-DynSys] c must be an np.ndarray for DTLTI systems."
          )
        if n != c.shape[0]:
          raise ValueError(
              "[ellReach-DynSys] Dimensions of A and c do not match."
          )
        self.c = c
      # G matrix
      if np.size(G) > 0:
        if not isinstance(G, np.ndarray):
          raise ValueError(
              "[ellReach-DynSys] G must be an np.ndarray for DTLTI systems."
          )
        if n != G.shape[0]:
          raise ValueError(
              "[ellReach-DynSys] Dimensions of A and G do not match."
          )
      self.G = G
    # Discrete-time linear time-varying system (DTLTV).
    elif sys_type == 'DTLTV':
      self.sys_type = 'DTLTV'
      if not isinstance(T, int) or not T > 0:
        raise ValueError("[ellReach-DynSys] T must be a positive integer.")
      self.T = T
      # A matrices
      if not isinstance(A, list):
        raise ValueError(
            "[ellReach-DynSys] A must be a list for DTLTV systems."
        )
      if len(A) != T-1:
        raise ValueError("[ellReach-DynSys] T and length of A do not match.")
      n = A[0].shape[0]
      self.A = A
      # B matrices
      if np.size(B) > 0:
        if not isinstance(B, list):
          raise ValueError(
              "[ellReach-DynSys] B must be a list for DTLTV systems."
          )
        if len(B) != T-1:
          raise ValueError("[ellReach-DynSys] T and length of B do not match.")
      self.B = B
      # c vectors
      if np.size(c) == 0:
        self.c = [np.zeros((n, 1))] * T
      else:
        if not isinstance(c, list):
          raise ValueError(
              "[ellReach-DynSys] c must be a list for DTLTV systems."
          )
        if len(c) != T-1:
          raise ValueError("[ellReach-DynSys] T and length of c do not match.")
        self.c = c
      # G matrices
      if np.size(G) > 0:
        if not isinstance(G, list):
          raise ValueError(
              "[ellReach-DynSys] G must be a list for DTLTV systems."
          )
        if len(G) != T-1:
          raise ValueError("[ellReach-DynSys] T and length of G do not match.")
      self.G = G
    else:
      raise ValueError("[ellReach-DynSys] Unsupported system type.")

  def display(self):
    """
    Displays information of the DynSys object.
    """
    print("\n")
    print("System type: ", self.sys_type)
    if self.sys_type == 'DTLTI':
      print("A matrix: \n", self.A)
      if not self.autonomous():
        print("B matrix: \n", self.B)
      else:
        print("This is an autonomous system.")
      print("c vector: \n", self.c)
      if not self.no_dstb():
        print("G matrix: \n", self.G)
      else:
        print("This system has no disturbance.")
    elif self.sys_type == 'DTLTV':
      print("Horizon T =", self.T)
      if self.autonomous():
        print("This is an autonomous system.")
      if self.no_dstb():
        print("This system has no disturbance.")
    print("\n")

  def time_varying(self):
    """
    Check if the system is time-varying.
    """
    if self.sys_type == 'DTLTV' or self.sys_type == 'CTLTV':
      return True
    else:
      return False

  def autonomous(self):
    """
    Check if the system is autonomous (empty B matrix).
    """
    if np.size(self.B) == 0:
      return True
    else:
      return False

  def no_dstb(self):
    """
    Check if the system has no distrubances (empty G matrix).
    """
    if np.size(self.G) == 0:
      return True
    else:
      return False
