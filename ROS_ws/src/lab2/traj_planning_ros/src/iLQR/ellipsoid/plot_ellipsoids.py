import numpy as np


def plot_ellipsoids(
    ax, E_list, arg_list=[], dims=None, N=200, plot_center=True,
    use_alpha=False
):
  """
  Plot a list of ellipsoids
  Args:
      E_list (list): list of ellipsoids
      arg_list (list): list of args dictionary.
      dims (list): dimension to be preserved.
      N (int): number of boundary points.
      plot_center (bool): plot the center of the ellipsoid if True. Defaults to
          True.
      use_alpha (bool): make later ellipsoids more transparent if True.
          Defaults to False.
  """
  if len(arg_list) == 0:
    arg_list = [dict(c='r')] * len(E_list)
  elif len(arg_list) == 1:
    arg_list = arg_list * len(E_list)
  else:
    assert len(arg_list) == len(E_list), "The length does not match."
  if use_alpha:
    alpha_list = np.linspace(1., 0.1, len(E_list))
  else:
    alpha_list = np.ones(len(E_list))

  for E0, arg, alpha in zip(E_list, arg_list, alpha_list):
    if dims:
      E = E0.projection(dims)
    else:
      E = E0.copy()
    if E.dim() > 3:
      raise ValueError("[ellReach-plot] ellipsoid dimension can be 1, 2 or 3.")
    if E.dim() == 1:
      x = np.array([(E.q - np.sqrt(E.Q))[0, 0], (E.q + np.sqrt(E.Q))[0, 0]])
      ax.plot(x, np.zeros_like(x), color=arg)
      ax.plot(E.q, 0, color=arg, marker='*')
    if E.dim() == 2:
      if E.is_degenerate():
        print("degenerate!")
      phi = np.array(np.linspace(0., 2. * np.pi, N))
      L = np.concatenate(
          (np.cos(phi).reshape(1, N), np.sin(phi).reshape(1, N)), axis=0
      )
      _, x = E.rho(L)
      # x = np.concatenate((x, x[:, 0].reshape(2, 1)), axis=1)
      ax.plot(x[0, :], x[1, :], alpha=alpha, **arg)
      if plot_center:
        ax.plot(E.q[0, 0], E.q[1, 0], marker='.', alpha=alpha, **arg)
      ax.set_aspect('equal')
    if E.dim() == 3:
      raise NotImplementedError
