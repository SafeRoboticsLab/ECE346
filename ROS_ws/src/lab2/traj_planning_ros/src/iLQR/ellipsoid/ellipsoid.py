# from __future__ import annotations
import numpy as np
# import matplotlib.pyplot as plt


class Ellipsoid():
    """
  The ellipsoid class.
  Author: Haimin Hu (haiminh@princeton.edu)
  Reference: Ellipsoidal Toolbox (MATLAB) by Dr. Alex Kurzhanskiy.
  E = { x : <(x - q), Q^(-1)(x - q)> <= 1 } - ellipsoid.
  q - center.
  Q - shape matrix.
  """

    def __init__(self,
                 q=np.array([]),
                 Q=np.array([[], []]),
                 auto_sym=True,
                 psd_tol=1e-15,
                 check = False):
        """
    Constructor of the ellipsoid object.
    Args:
        q (np.ndarray (n, 1)): Center.
        Q (np.ndarray (n, n)): Shape matrix (a positive semi-definite matrix).
        psd_tol (float): PSD matrix tolerance (a small positive number).
    """

        def check_symmetric(a, rtol=1e-05, atol=1e-05):
            return np.allclose(a, a.T, rtol=rtol, atol=atol)

        def symmetricize(Q):
            if Q.shape[1] == 0:
                return Q
            else:
                return (Q + Q.T) / 2
        if check:
            if auto_sym:
                Q = symmetricize(Q)
            if q.size > 0 and len(q.shape) != 2 and q.shape[1] != 1:
                raise ValueError("[ellReach] center (q) must be a column vector.")
            if Q.size > 0 and len(Q.shape) != 2:
                raise ValueError("[ellReach] Q must be a matrix.")
            n = q.size
            if Q.shape[1] != n:
                raise ValueError(
                    "[ellReach] dimensions of the q and Q do not match.")
            if n > 0:
                if not check_symmetric(Q):
                    raise ValueError("[ellReach] Q must be a symmetric matrix.")
                mev = min(np.linalg.eigvals(Q))
                if mev < 0:
                    if abs(mev) > psd_tol:
                        raise ValueError(
                            "[ellReach] shape matrix Q must be positive semi-definite."
                        )
        self.q = q  # center
        self.Q = Q  # shape matrix

    def __matmul__(self, A):
        """
    Multiplication of the ellipsoid by a matrix or a scalar.
    If E(q,Q) is an ellipsoid, and A - matrix of suitable dimensions, then
        E(q, Q) @ A = E(Aq, AQA').
    """
        if self.is_empty():
            return self
        else:
            if A.shape[1] != self.dim():
                raise ValueError(
                    "[ellReach] dimensions of A and the shape matrix do not match."
                )
            q = A @ self.q
            Q = A @ self.Q @ A.T
            return Ellipsoid(q, Q)

    def __add__(self, b):
        """
    Computes vector addition with an ellipsoid:
        E + b = E(q + b, Q).
    Args:
        b (np.ndarray): a vector.
    """
        if b.shape != self.q.shape:
            raise ValueError(
                "[ellReach] dimensions of b and q of the ellipsoid do not match."
            )
        return Ellipsoid(self.q + b, self.Q)

    def __sub__(self, b):
        """
    Computes vector subtraction from an ellipsoid:
        E - b = E(q - b, Q).
    Args:
        b (np.ndarray): a vector.
    """
        if b.shape != self.q.shape:
            raise ValueError(
                "[ellReach] dimensions of b and q of the ellipsoid do not match."
            )
        return Ellipsoid(self.q - b, self.Q)

    def inv(self, mode='reg'):
        """
    Inverts the shape matrix of the ellipsoid.
    """
        if self.is_empty():
            return self
        elif self.is_degenerate():
            if mode == 'reg':
                self.regularize()
                self.Q = np.linalg.inv(self.Q)
            elif mode == 'pinv':
                self.Q = np.linalg.pinv(self.Q)
        else:
            self.Q = np.linalg.inv(self.Q)

    def copy(self):
        """
    Creats an identical ellipsoid object.
    """
        return Ellipsoid(self.q, self.Q)

    def dim(self):
        """
    Returns the dimension of the ellipsoid.
    """
        return self.q.size

    def rank(self):
        """
    Returns the rank of the shape matrix.
    """
        if self.dim() == 0:
            return 0
        else:
            return np.linalg.matrix_rank(self.Q)

    def parameters(self):
        """
    Returns parameters of the ellipsoid.
    """
        return self.q, self.Q

    def display(self):
        """
    Displays information of the ellipsoid object.
    """
        print("\n")
        if self.dim() == 0:
            print("Empty ellipsoid.")
        else:
            print("Center: \n", self.q)
            print("Shape Matrix: \n", self.Q)
            if self.rank() < self.dim():
                print("Degenerate (rank %d) ellipsoid in R^%d.\n" %
                      (self.rank(), self.dim()))
            else:
                print("Nondegenerate ellipsoid in R^%d.\n" % self.dim())
                print("Volumn:%f.\n" % self.volume())

    def is_degenerate(self):
        """
    Checks if the ellipsoid is degenerate.
    Returns True if ellipsoid E is degenerate, False - otherwise.
    """
        return self.rank() < self.dim()

    def is_empty(self):
        """
    Checks if the ellipsoid object is empty.
    Returns True if ellipsoid E is empty, False - otherwise.
    """
        return self.dim() == 0

    def eig(self):
        """
    Computes the eigenvalues of the ellipsoid.
    Returns:
        np.ndarray: a list of eigenvalues
    """
        if self.is_empty():
            return np.array([])
        else:
            return np.linalg.eigvals(self.Q)

    def min_eig(self):
        """
    Returns the minimal eigenvalue of the ellipsoid.
    """
        if self.is_empty():
            return np.array([])
        else:
            return min(np.linalg.eigvals(self.Q))

    def max_eig(self):
        """
    Returns the maximal eigenvalue of the ellipsoid.
    """
        if self.is_empty():
            return np.array([])
        else:
            return max(np.linalg.eigvals(self.Q))

    def trace(self):
        """
    Returns the trace of the ellipsoid.
    """
        if self.is_empty():
            return np.array([])
        else:
            return np.trace(self.Q)

    def move2origin(self):
        """
    Moves the ellipsoid object to the origin.
    """
        if not self.is_empty():
            self.q = np.zeros_like(self.q)

    def symmetricize(self):
        """
    Symmetricize the shape matrix.
    """
        if not self.is_empty():
            self.Q = (self.Q + self.Q.T) / 2

    def regularize(self, abs_tol=1e-7):
        """
    Regularization of a singular symmetric matrix.
    """
        n = self.dim()
        r = self.rank()
        if n > r:
            U, _, _ = np.linalg.svd(self.Q)
            R1 = np.concatenate((np.zeros((r, r)), np.zeros((r, n - r))),
                                axis=1)
            R2 = np.concatenate((np.zeros(
                (n - r, r)), abs_tol * np.eye(n - r)),
                                axis=1)
            reg = np.concatenate((R1, R2), axis=0)
            self.Q = self.Q + (U @ reg @ U.T)
            self.symmetricize()

    def projection(self, dims):
        """
    Computes projection of the ellipsoid onto the given dimensions.
    Args:
        dims (list): dimension to be preserved.
    Returns:
        ellipsoid: projected ellipsoid (of lower dimension).
    """
        if not isinstance(dims, list):
            raise ValueError("[ellReach-projection] dims must be a list.")
        n = self.dim()
        m = len(dims)
        B = np.zeros((n, m))
        for i in range(m):
            B[dims[i], i] = 1
        return self @ B.T

    def projection_subspace(self, B, abs_tol=1e-7):
        """
    Computes projection of the ellipsoid onto the given subspace.
    Args:
        B (np.ndarray): orthogonal basis vectors that specifies the subspace.
    Returns:
        ellipsoid: projected ellipsoid (of lower dimension).
    """
        # Checks the orthogonality of the columns of B.
        for i in range(0, B.shape[1] - 1):
            for j in range(i + 1, B.shape[1]):
                if B[:, i] @ B[:, j] > abs_tol:
                    raise ValueError(
                        "[ellReach-projection] basis vectors must be orthogonal."
                    )
        # Normalizes the basis vectors.
        for i in range(0, B.shape[1]):
            B[:, i] = B[:, i] / np.linalg.norm(B[:, i])
        # Computes projection.
        return self @ B.T

    def contains(self, E, reverse=False):
        """
    Checks if the ellipsoid object contains E (the other way around if
    reverse=True).
    Args:
        E (ellipsoid): the other ellipsoid.
        reverse (bool, optional): whether to reverse the order of inclusion.
        Defaults to False.
    """
        raise NotImplementedError

    def is_internal(self, x, abs_tol=1e-7):
        """
    Checks if given points belong to the ellipsoid, i.e. if inequality
        <(x - q), Q^(-1)(x - q)> <= 1
    holds.
    Args:
        x (np.ndarray): a column vector with the same dimension as the
          ellipsoid object.
    """
        if np.size(x) != self.dim():
            raise ValueError(
                "[ellReach-is_internal] dimensions of x and ellipsoid do not match."
            )
        if self.is_empty():
            return False
        E = self.copy()
        if E.is_degenerate():
            E.regularize()
        q = x - E.q
        Q = E.Q
        r = (q.T @ np.linalg.inv(Q) @ q)[0, 0]
        if r < 1 or np.abs(r - 1) < abs_tol:
            return True
        else:
            return False

    def volume(self):
        """
    Returns the volume of the ellipsoid.
    """
        if self.is_degenerate():
            return 0
        N = self.dim() - 1
        if np.mod(N, 2) > 0:
            k = (N + 1) / 2
            S = (np.pi**k) / np.math.factorial(k)
        else:
            k = N / 2
            S = ((2**(2 * k + 1)) * (np.pi**k) *
                 np.math.factorial(k)) / np.math.factorial(2 * k + 1)
        return S * np.sqrt(np.linalg.det(self.Q))

    def plot(self, ax, color='r', dims=None, N=200, plot_center=True):
        """
    Plots the ellipsoid object in 1D, 2D or 3D.
    Args:
        ax: plot object
        dims (list): dimension to be preserved.
        N (int): number of boundary points.
    """
        if dims:
            E = self.projection(dims)
        else:
            E = self.copy()
        if E.dim() > 3:
            raise ValueError(
                "[ellReach-plot] ellipsoid dimension can be 1, 2 or 3.")
        if E.dim() == 1:
            x = np.array([(E.q - np.sqrt(E.Q))[0, 0], (E.q + np.sqrt(E.Q))[0,
                                                                           0]])
            ax.plot(x, np.zeros_like(x), color)
            ax.plot(E.q, 0, color + '*')
        if E.dim() == 2:
            phi = np.array(np.linspace(0., 2. * np.pi, N))
            L = np.concatenate(
                (np.cos(phi).reshape(1, N), np.sin(phi).reshape(1, N)), axis=0)
            _, x = E.rho(L)
            # x = np.concatenate((x, x[:, 0].reshape(2, 1)), axis=1)
            ax.plot(x[0, :], x[1, :], color)
            if plot_center:
                ax.plot(E.q[0, 0], E.q[1, 0], color + '.')
            ax.set_aspect('equal')
        if E.dim() == 3:
            raise NotImplementedError

    def rho(self, L, eps=1e-10):
        """
    Computes the support function of the ellipsoid E in directions specified by
    the columns of matrix L, and boundary points X of this ellipsoid that
    correspond to directions in L.
    Args:
        E (ellipsoid): an ellipsoid object.
        L (np.ndarray): a direction matrix.
    Returns:
        res (np.ndarray): the values of the support function for the specified
            ellipsoid E and directions L.
        x (np.ndarray): boundary points of the ellipsoid E that correspond to
            directions in L.
    """
        d = L.shape[1]
        res = np.array([]).reshape((1, 0))
        x = np.array([]).reshape((self.dim(), 0))
        for i in range(d):
            l = L[:, i].reshape(self.dim(), 1)
            sr = np.maximum(np.sqrt(l.T @ self.Q @ l)[0, 0], eps)
            res = np.hstack((res, self.q.T @ l + sr))
            x = np.hstack((x, self.Q @ l / sr + self.q))
        return res, x

    def minksum(self, E2, l, mode='ea'):
        """
    Computes an external ellipsoid approximating the minksum of self and
    ellipsoid E2 in given directions.
    Args:
        E2 (ellipsoid): the other ellipsoid.
        l (np.ndarray): a vector in R^n specifying the direction.
        mode (str): 'ea' = external approximation
                    'ia' = internal approximation
    """
        if mode == 'ea':
            E1 = self.copy()
            for E in [E1, E2]:
                if E.is_degenerate():
                    E.regularize()
            Q1 = E1.Q
            Q2 = E2.Q
            p1 = np.sqrt(l.T @ Q1 @ l)
            p2 = np.sqrt(l.T @ Q2 @ l)
            S = (1 / p1) * Q1 + (1 / p2) * Q2
            Q = (p1 + p2) * 0.5 * (S + S.T)
            return Ellipsoid(E1.q + E2.q, Q)
        else:
            raise NotImplementedError

    def minksum_minVol(self, E2):
        """
    Computes an external ellipsoid approximating the minksum of self and
    ellipsoid E2. Try different minksum (with candidate directions given by the
    eigenvectors of E1.Q and E2.Q) and pick the one with the minimal volume.
    Args:
        E2 (ellipsoid): the other ellipsoid.
    """
        E1 = self.copy()
        for E in [E1, E2]:
            if E.is_degenerate():
                E.regularize()
        if E1.dim() != E2.dim():
            raise ValueError(
                "[ellReach-minksum_minVol] Dimensions of E1 and E2 do not match."
            )
        L = []
        _, eigv1 = np.linalg.eig(E1.Q)
        _, eigv2 = np.linalg.eig(E2.Q)
        for i in range(E1.dim()):
            L.append(eigv1[i, :].reshape(E1.dim(), 1))
            L.append(eigv2[i, :].reshape(E2.dim(), 1))
        E_list = []
        V_list = []
        for l in L:
            E_sum = E1.minksum(E2, l)
            E_list.append(E_sum)
            V_list.append(E_sum.volume())
        idx = V_list.index(min(V_list))
        return E_list[idx]
