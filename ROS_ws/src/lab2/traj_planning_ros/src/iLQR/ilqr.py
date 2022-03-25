from weakref import ref
import numpy as np
from .cost import Cost
from .dynamics import Dynamics
import time


class iLQR():

    def __init__(self, ref_path, params):

        self.T = params['T']
        self.N = params['N']

        self.ref_path = ref_path

        self.steps = params['max_itr']

        self.tol = 1e-2
        self.lambad = 10
        self.lambad_max = 100
        self.lambad_min = 1e-3

        self.dynamics = Dynamics(params)
        self.alphas = 1.1**(-np.arange(10)**2)

        self.dim_x = self.dynamics.dim_x
        self.dim_u = self.dynamics.dim_u

        self.cost = Cost(params)

    def forward_pass(self, nominal_states, nominal_controls, K_closed_loop,
                     k_open_loop, alpha):
        # t0 = time.time()
        X = np.zeros_like(nominal_states)
        U = np.zeros_like(nominal_controls)

        X[:, 0] = nominal_states[:, 0]
        for i in range(self.N - 1):
            K = K_closed_loop[:, :, i]
            k = k_open_loop[:, i]
            u = (nominal_controls[:, i] + alpha * k +
                 K @ (X[:, i] - nominal_states[:, i]))
            X[:, i + 1], U[:, i] = self.dynamics.forward_step(X[:, i], u)
        # print("forward pass integration use: ", time.time()-t0)
        
        closest_pt, slope, theta = self.ref_path.get_closest_pts(X[:2, :])
        t1 = time.time()
        J = self.cost.get_cost(X, U, closest_pt, slope, theta)
        # print("get cost use: ", time.time()-t1)
        # print("forward pass use: ", time.time()-t0)
        return X, U, J, closest_pt, slope, theta

    def backward_pass(self, nominal_states, nominal_controls, closest_pt,
                      slope):
        # t0 = time.time()
        L_x, L_xx, L_u, L_uu, L_ux = self.cost.get_derivatives(
            nominal_states, nominal_controls, closest_pt, slope)
        # print("backward pass derivative use: ", time.time()-t0)
        fx, fu = self.dynamics.get_AB_matrix(nominal_states, nominal_controls)

        k_open_loop = np.zeros((self.dim_u, self.N - 1))
        K_closed_loop = np.zeros((self.dim_u, self.dim_x, self.N - 1))
        # derivative of value function at final step
        V_x = L_x[:, -1]
        V_xx = L_xx[:, :, -1]

        reg_mat = self.lambad * np.eye(self.dim_u)

        Q_u_hist = np.zeros([self.dim_u, self.N - 1])
        Q_uu_hist = np.zeros([self.dim_u, self.dim_u, self.N - 1])
        for i in range(self.N - 2, -1, -1):
            Q_x = L_x[:, i] + fx[:, :, i].T @ V_x
            Q_u = L_u[:, i] + fu[:, :, i].T @ V_x
            Q_xx = L_xx[:, :, i] + fx[:, :, i].T @ V_xx @ fx[:, :, i]
            Q_ux = fu[:, :, i].T @ V_xx @ fx[:, :, i] + L_ux[:, :, i]
            Q_uu = L_uu[:, :, i] + fu[:, :, i].T @ V_xx @ fu[:, :, i]

            Q_uu_inv = np.linalg.inv(Q_uu + reg_mat)
            k_open_loop[:, i] = -Q_uu_inv @ Q_u
            K_closed_loop[:, :, i] = -Q_uu_inv @ Q_ux

            # Update value function derivative for the previous time step
            V_x = Q_x - K_closed_loop[:, :, i].T @ Q_uu @ k_open_loop[:, i]
            V_xx = Q_xx - K_closed_loop[:, :, i].T @ Q_uu @ K_closed_loop[:, :,
                                                                          i]

            Q_u_hist[:, i] = Q_u
            Q_uu_hist[:, :, i] = Q_uu
        # print("backward pass use: ", time.time()-t0)

        return K_closed_loop, k_open_loop

    def solve(self, cur_state, controls=None, obs_list=[], record=False):
        status = 0
        self.lambad = 10

        time0 = time.time()

        if controls is None:
            controls = np.zeros((self.dim_u, self.N))
        states = np.zeros((self.dim_x, self.N))
        states[:, 0] = cur_state

        for i in range(1, self.N):
            states[:,
                   i], _ = self.dynamics.forward_step(states[:, i - 1],
                                                      controls[:, i - 1])
        closest_pt, slope, theta = self.ref_path.get_closest_pts(states[:2, :])

        self.cost.update_obs(obs_list)

        J = self.cost.get_cost(states, controls, closest_pt, slope, theta)

        converged = False

        # have_not_updated = 0
        for i in range(self.steps):
            K_closed_loop, k_open_loop = self.backward_pass(
                states, controls, closest_pt, slope)
            updated = False
            for alpha in self.alphas:
                X_new, U_new, J_new, closest_pt_new, slope_new, theta_new = (
                    self.forward_pass(states, controls, K_closed_loop,
                                      k_open_loop, alpha))
                if J_new <= J:
                    if np.abs((J - J_new) / J) < self.tol:
                        converged = True
                    J = J_new
                    states = X_new
                    controls = U_new
                    closest_pt = closest_pt_new
                    slope = slope_new
                    theta = theta_new
                    updated = True
                    break
            if updated:
                self.lambad *= 0.7
            else:
                status = 2
                break
            # self.lambad = min(max(self.lambad_min, self.lambad), self.lambad_max)
            self.lambad = max(self.lambad_min, self.lambad)

            if converged:
                status = 1
                break
        t_process = time.time() - time0
        # print("step, ", i, "alpha:", alpha)

        if record:
            # get parameters for FRS
            K_closed_loop, _ = self.backward_pass(states, controls, closest_pt,
                                                  slope)
            fx, fu = self.dynamics.get_AB_matrix(states, controls)
        else:
            K_closed_loop = None
            fx = None
            fu = None
        return states, controls, t_process, status, theta, K_closed_loop, fx, fu
