from typing import Tuple, Optional, Dict, Union
from jaxlib.xla_extension import ArrayImpl
import time
import os
import numpy as np
import jax
from .dynamics import Bicycle5D
from .cost import Cost, CollisionChecker, Obstacle
from .ref_path import RefPath
from .config import Config
import time

status_lookup = ["Iteration Limit Exceed",
                "Converged",
                "Failed Line Search"]

class ILQR_np():
	def __init__(self, config_file = None) -> None:

		self.config = Config()  # Load default config.
		if config_file is not None:
			self.config.load_config(config_file)  # Load config from file.
		
		self.load_parameters()
		print('ILQR setting:', self.config)

		# Set up Jax parameters
		jax.config.update('jax_platform_name', self.config.platform)
		print('Jax using Platform: ', jax.lib.xla_bridge.get_backend().platform)

		self.dyn = Bicycle5D(self.config)
		self.cost = Cost(self.config)
		self.ref_path = None

		# collision checker
		# Note: This will not be used until lab2.
		self.collision_checker = CollisionChecker(self.config)
		self.obstacle_list = []

		self.warm_up()

	def load_parameters(self):
		'''
		This function defines ILQR parameters from <self.config>.
		'''
		# ILQR parameters
		self.dim_x = self.config.num_dim_x
		self.dim_u = self.config.num_dim_u
		self.T = int(self.config.T)
		self.dt = float(self.config.dt)
		self.max_iter = int(self.config.max_iter)
		self.tol = float(self.config.tol)  # ILQR update tolerance.

		# line search parameters.
		self.alphas = self.config.line_search_base**(np.arange(self.config.line_search_a,
                                                self.config.line_search_b,
                                                self.config.line_search_c)
                                            )

		print('Line Search Alphas: ', self.alphas)

		# regularization parameters
		self.reg_min = float(self.config.reg_min)
		self.reg_max = float(self.config.reg_max)
		self.reg_init = float(self.config.reg_init)
		self.reg_scale_up = float(self.config.reg_scale_up)
		self.reg_scale_down = float(self.config.reg_scale_down)
		self.max_attempt = self.config.max_attempt
		
	def warm_up(self):
		'''
		Warm up the jitted functions.
		'''
		# Build a fake path as a 1 meter radius circle.
		theta = np.linspace(0, 2 * np.pi, 100)
		centerline = np.zeros([2, 100])
		centerline[0,:] = 1 * np.cos(theta)
		centerline[1,:] = 1 * np.sin(theta)

		self.ref_path = RefPath(centerline, 0.5, 0.5, 1, True)

		# add obstacle
		obs = np.array([[0, 0, 0.5, 0.5], [1, 1.5, 1, 1.5]]).T
		obs_list = [[obs for _ in range(self.T)]]
		self.update_obstacles(obs_list)

		x_init = np.array([0.0, -1.0, 1, 0, 0])
		print('Start warm up ILQR...')
		# import matplotlib.pyplot as plt
		self.plan(x_init, verbose=False)
		print('ILQR warm up finished.')
		# plt.plot(plan['trajectory'][0,:], plan['trajectory'][1,:])
		# print(f'Warm up takes {plan['t_process']} seconds.')
		self.ref_path = None
		self.obstacle_list = []

	def update_ref_path(self, ref_path: RefPath):
		'''
		Update the reference path.
		Args:
			ref_path: RefPath: reference path.
		'''
		self.ref_path = ref_path

	def update_obstacles(self, vertices_list: list):
		'''
		Update the obstacle list for a list of vertices.
		Args:
			vertices_list: list of np.ndarray: list of vertices for each obstacle.
		'''
		# Note: This will not be used until lab2.
		self.obstacle_list = []
		for vertices in vertices_list:
			self.obstacle_list.append(Obstacle(vertices))

	def get_references(self, trajectory: Union[np.ndarray, ArrayImpl]):
		'''
		Given the trajectory, get the path reference and obstacle information.
		Args:
			trajectory: [num_dim_x, T] trajectory.
		Returns:
			path_refs: [num_dim_x, T] np.ndarray: references.
			obs_refs: [num_dim_x, T] np.ndarray: obstacle references.
		'''
		trajectory = np.asarray(trajectory)
		path_refs = self.ref_path.get_reference(trajectory[:2, :])
		obs_refs = self.collision_checker.check_collisions(trajectory, self.obstacle_list)
		return path_refs, obs_refs

	def plan(self, init_state: np.ndarray, 
				controls: Optional[np.ndarray] = None, verbose=False) -> Dict:
		
		# We first check if the planner is ready
		if self.ref_path is None:
			return dict(status=-1)
		
		t_start = time.time()
		'''
		Main ILQR loop.
		Args:
			init_state: [num_dim_x] np.ndarray: initial state.
			control: [num_dim_u, N] np.ndarray: initial control.
		'''
		# np.set_printoptions(suppress=True, precision=5)
		if controls is None:
			print('No initial control is provided. Using zero control.')
			controls =np.zeros((self.dim_u, self.T))
		else:
			assert controls.shape[1] == self.T
		
		# Rolls out the nominal trajectory and gets the initial cost.
		#* This is differnet from the naive ILQR as it relies on the information
		#* from the pyspline.
		trajectory, controls = self.dyn.rollout_nominal_np(init_state, controls)

		path_refs, obs_refs = self.get_references(trajectory)

		J = self.cost.get_traj_cost(trajectory, controls, path_refs, obs_refs)

		converged = False
		reg = self.reg_init
		
		# parameters for tracking the status
		fail_attempts = 0

		t_setup = time.time() - t_start
		status = 0
		num_cost_der = 0
		num_dyn_der = 0
		num_back = 0
		num_forward = 0
		t_cost_der = 0
		t_dyn_der = 0
		t_back = 0
		t_forward = 0

		for i in range(self.max_iter):
			updated = False
			# Get the derivatives of the cost 
			t0 = time.time()
			q, r, Q, R, H = self.cost.get_derivatives_np(trajectory, controls, path_refs, obs_refs)
			t_cost_der += (time.time()-t0)
			num_cost_der += 1
			
			# We only need dynamics derivatives (A and B matrics) from 0 to N-2.
			# But doing slice with Jax will leads to performance issue.
			# So we calculate the derivatives from 0 to N-1 and Backward pass will ignore the last one.
			t0 = time.time()
			A, B = self.dyn.get_jacobian_np(trajectory, controls)
			t_dyn_der += (time.time()-t0)
			num_dyn_der += 1
			
			if np.any(np.isnan(Q)):
				print("Detect NAN in Q")
				print("traj:", trajectory)
				print("control", controls)
				print("path", path_refs)
				print("obs", obs_refs)
				break
			#Backward pass with new regularization.
			t0 = time.time()
			K_closed_loop, k_open_loop, reg = self.backward_pass(
				q=q, r=r, Q=Q, R=R, H=H, A=A, B=B, reg = reg
			)
			t_back += (time.time()-t0)
			num_back += 1
			
			if np.any(np.isnan(K_closed_loop)):
				print("Detect NAN in K_closed_loop")
				break

			if np.any(np.isnan(k_open_loop)):
				print("Detect NAN in k_open_loop")
				break

			# Line search through alphas.
			for alpha in self.alphas:
				t0 = time.time()
				X_new, U_new, J_new, refs_new, obs_refs_new = (
						self.forward_pass(
								trajectory, controls, K_closed_loop, k_open_loop, alpha
						)
				)
				t_forward += (time.time()-t0)
				num_forward += 1
				
				# check NAN
				if np.any(np.isnan(X_new)):
					print("Detect NAN")
					continue

				if J_new <= J:  # Improved!
					# Small improvement.
					# if np.abs(J-J_new)/max(1, np.abs(J)) < self.tol:
					if np.abs(J-J_new) < (self.tol*min(1,alpha)):
						converged = True
					if verbose:
						print("Update from ", J, " to ", J_new, "reg: ", reg,
							"alpha: {0:0.3f}".format(alpha), "{0:0.3f}".format(time.time()-t_start))
					# Updates nominal trajectory and best cost.
					J = J_new
					trajectory = X_new
					controls = U_new
					path_refs = refs_new
					obs_refs = obs_refs_new
					reg = max(self.reg_min, reg/self.reg_scale_down)
					updated = True
					fail_attempts = 0
					break # break the for loop
				elif np.abs(J-J_new) < 1e-3:
					converged = True
					if verbose:
						print(f"cost increase from {J} to {J_new}, but the difference is {np.abs(J-J_new)} is small.")
					break

			# Terminates early if the objective improvement is negligible.
			if converged:
				status = 1
				break

			# if no line search succeeds, terminate.
			if not updated:
				fail_attempts += 1
				reg = reg*self.reg_scale_up
				if verbose: 
					print(f"Fail attempts {fail_attempts}, cost increase from {J} to {J_new} new reg {reg}.")
				if fail_attempts > self.max_attempt or reg > self.reg_max:
					status = 2
					break

		t_process = time.time() - t_start
		analysis_string = f"Exit after {i} iterations with runtime {t_process} with status {status_lookup[status]}. "+ \
					f"Set uo takes {t_setup} s. " + \
					f"Total {num_cost_der} cost derivative with average time of {t_cost_der/num_cost_der} s. " + \
					f"Total {num_dyn_der} dyn derivative with average time of {t_dyn_der/num_dyn_der} s. " + \
					f"Total {num_forward} forward with average time of {t_forward/num_forward} s. " +\
					f"Total {num_back} cost derivative with average time of {t_back/num_back} s."

		if verbose:
			print(analysis_string)

		solver_info = dict(
				trajectory=np.asarray(trajectory), controls=np.asarray(controls),
				K_closed_loop=np.asarray(K_closed_loop),
				k_open_loop=np.asarray(k_open_loop), t_process=t_process,
				status=status, J=J, q=q, r=r,
				Q=Q, R=R, H=H,
				A=A, B=B, T = self.T, dt = self.dt, 
				analysis = analysis_string
		)
		return solver_info

	def forward_pass(
			self, nominal_states: np.ndarray, nominal_controls: np.ndarray,
			K_closed_loop: np.ndarray, k_open_loop: np.ndarray, alpha: float
	) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray,
			np.ndarray]:
		X, U = self.rollout(
				nominal_states, nominal_controls, K_closed_loop, k_open_loop, alpha
		)
		path_refs, obs_refs = self.get_references(X)
		
		J = self.cost.get_traj_cost(X, U, path_refs, obs_refs)
		
		return X, U, J, path_refs, obs_refs

	def backward_pass(
			self, q: np.ndarray, r: np.ndarray, Q: np.ndarray,
			R: np.ndarray, H: np.ndarray, A: np.ndarray, B: np.ndarray,
			reg: float
	) -> Tuple[np.ndarray, np.ndarray]:
		"""
		backward pass looped computation.

		Args:
				q (np.ndarray): (dim_x, N)
				r (np.ndarray): (dim_u, N)
				Q (np.ndarray): (dim_x, dim_x, N)
				R (np.ndarray): (dim_u, dim_u, N)
				H (np.ndarray): (dim_u, dim_x, N)
				A (np.ndarray): (dim_x, dim_x, N-1)
				B (np.ndarray): (dim_x, dim_u, N-1)

		Returns:
				Ks (np.ndarray): gain matrices (dim_u, dim_x, N - 1)
				ks (np.ndarray): gain vectors (dim_u, N - 1)
		"""
		Ks = np.zeros((self.dim_u, self.dim_x, self.T - 1))
		ks = np.zeros((self.dim_u, self.T - 1))
		V_x = q[:, -1]
		V_xx = Q[:, :, -1]
		t = self.T-2

		while t>=0:
			Q_x = q[:, t] + A[:, :, t].T @ V_x
			Q_u = r[:, t] + B[:, :, t].T @ V_x
			Q_xx = Q[:, :, t] + A[:, :, t].T @ V_xx @ A[:, :, t]
			Q_ux = H[:, :, t] + B[:, :, t].T @ V_xx @ A[:, :, t]
			Q_uu = R[:, :, t] + B[:, :, t].T @ V_xx @ B[:, :, t]

			# According to the paper, the regularization is added to Vxx for robustness.
			# http://roboticexplorationlab.org/papers/iLQR_Tutorial.pdf
			reg_mat = reg* np.eye(self.dim_x)
			V_xx_reg = V_xx + reg_mat
			Q_ux_reg = H[:, :, t] + B[:, :, t].T @ V_xx_reg @ A[:, :, t]
			Q_uu_reg = R[:, :, t] + B[:, :, t].T @ V_xx_reg @ B[:, :, t]

			if (not (np.all(np.linalg.eigvalsh(Q_uu_reg) > 0)) and (reg < self.reg_max)):
				t = self.T-2
				V_x = q[:, -1]
				V_xx = Q[:, :, -1]
				reg *= self.reg_scale_up
				continue

			Q_uu_reg_inv = np.linalg.inv(Q_uu_reg)

			Ks[:, :, t] = (-Q_uu_reg_inv @ Q_ux_reg)
			ks[:, t] = (-Q_uu_reg_inv @ Q_u)

			V_x = Q_x + Ks[:,:,t].T @ Q_u + Q_ux.T @ ks[:, t] + Ks[:,:,t].T @ Q_uu @ ks[:, t] 
			V_xx = Q_xx +  Ks[:, :, t].T @ Q_ux+ Q_ux.T @ Ks[:, :, t] + Ks[:, :, t].T @ Q_uu @ Ks[:, :, t]
			t -= 1

		return Ks, ks, reg

	def rollout(
			self, nominal_states: np.ndarray, nominal_controls: np.ndarray,
			K_closed_loop: np.ndarray, k_open_loop: np.ndarray, alpha: float
	) -> Tuple[np.ndarray, np.ndarray]:
		X = np.zeros((self.dim_x, self.T))
		U = np.zeros((self.dim_u, self.T))  #  Assumes the last ctrl are zeros.

		X[:,0] = nominal_states[:,0]

		for t in range(self.T - 1):
			dx = X[:, t] - nominal_states[:, t]
			# VERY IMPORTANT: THIS IS A HACK TO MAKE THE ANGLE DIFFERENCE BETWEEN -pi and pi
			dx[3] = np.mod(dx[3] + np.pi, 2 * np.pi) - np.pi
			u_fb = np.einsum("ik,k->i", K_closed_loop[:, :, t], dx)
			u = nominal_controls[:, t] + alpha * k_open_loop[:, t] + u_fb
			x_nxt, u_clip = self.dyn.integrate_forward_np(X[:, t], u)
			X[:, t+1] = x_nxt
			U[:, t] =u_clip

		return X, U

	def warm_up(self):
		"""Warm up the jitted functions."""
		
		# Build a fake path as a 1 meter radius circle.
		theta = np.linspace(0, 2 * np.pi, 100)
		centerline = np.zeros([2, 100])
		centerline[0,:] = 1 * np.cos(theta)
		centerline[1,:] = 1 * np.sin(theta)

		self.ref_path = RefPath(centerline, 0.5, 0.5, 1, True)

		# add obstacle
		obs = np.array([[0, 0, 0.5, 0.5], [1, 1.5, 1, 1.5]]).T
		obs_list = [[obs for _ in range(self.T)]]
		self.update_obstacles(obs_list)

		x_init = np.array([0.0, -1.0, 1, 0, 0])
		print("Start warm up ILQR...")
		# import matplotlib.pyplot as plt
		plan = self.plan(x_init, verbose=False)
		print("ILQR warm up finished.")
		# plt.plot(plan['trajectory'][0,:], plan['trajectory'][1,:])
		# print(f"Warm up takes {plan['t_process']} seconds.")
		self.ref_path = None
		self.obstacle_list = []
	

