from ellipsoid import Ellipsoid
from dyn_sys import DynSys
from reach import Reach
import numpy as np
from plot_ellipsoids import plot_ellipsoids

q1d = np.array([[2]]).T
Q1d = np.array([[5]])
E1d = Ellipsoid(q1d, Q1d)

q = np.array([[0, 0]]).T
Q = np.array([[2, 0], [0, 3]])
E = Ellipsoid(q, Q)

q = np.array([[3, 4]]).T
Q = np.array([[1, 0], [0, 5]])
E1 = Ellipsoid(q, Q)

q = np.array([])
Q = np.array([[], []])
# E_empty = ellipsoid(q, Q)
E_empty = Ellipsoid()
# E_empty.display()

q = np.array([[1, 2, 3, 4]]).T
Q = np.array([[1, 0, 0, 0], [0, 4, 0, 0], [0, 0, 2, 0], [0, 0, 0, 0]])
E_dg = Ellipsoid(q, Q)
# E_dg.display()

# E_dg.display()
# E_dg.plot([0, 1])

# A = np.array([[1, 2, 3], [2, 3, 4], [3, 4, 5]])
# b = np.array([[7, 8, 9]]).T
# print(np.linalg.matrix_rank(A))
# E.display()
# (E @ A).display()
# (E@A + b).display()

sys1 = DynSys(
    'DTLTI',
    np.array([[0.8, 0, 0.1, 0], [0, 0.6, 0, 0.1], [0.1, 0, 0.6, 0],
              [0, 0.1, 0, 0.7]]), B=np.array([]), c=np.array([]),
    G=np.eye(4)[:, 0:2]
)
sys1.display()

X0 = Ellipsoid(q=np.array([[-10, 2, 2, 2]]).T, Q=1 * np.eye(4))

D = Ellipsoid(q=np.array([[0, 0]]).T, Q=.01 * np.eye(2))

reach_sys1 = Reach(sys1, X0, D)

FRS_sys1 = reach_sys1.FRS(T=10)
plot_ellipsoids(FRS_sys1, [], dims=[0, 1])

# T = 10
# sys2 = DynSys(
#     'DTLTV', [np.eye(3)] * T, [np.array([[0, 0, 1]]).T] * T, np.array([]),
#     np.array([]), T
# )
# sys2.display()

# N = 10
# V = np.array(np.linspace(0, 2 * np.pi, N))
# L = np.concatenate((np.cos(V).reshape(1, N), np.sin(V).reshape(1, N)), axis=0)
# r, x = E.rho(L)
