from abc import ABC, abstractmethod
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt

class TIntersectionVisualizer(ABC):
    def __init__(self):
        self.N = 50
        self.T = 1.0
        self.dt = self.T / self.N
        self.car_traj_x = CubicSpline(np.arange(0, self.T, self.T/6), np.array([3.85, 4, 5, 7.5, 12.5, 18.0]))
        self.car_traj_y = CubicSpline(np.arange(0, self.T, self.T/6), np.array([28, 25, 23.5, 22.5, 21.85, 21.7]))

        self.ego_traj_x = CubicSpline(np.arange(0, self.T, self.T/6), np.array([7.45, 7.5, 7.75, 9.0, 12.5, 18.0]))
        self.ego_traj_y = CubicSpline(np.arange(0, self.T, self.T/6), np.array([16, 18, 20., 21.5, 21.85, 21.85]))

    def plot_road(self, ax):
        road_rules = {
            "x_min": 2,
            "x_max": 9.4,
            "y_max": 27.4,
            "y_min": 20,
            "width": 3.7
        }

        x_max = 25
        y_max = 40

        x_center = road_rules["x_min"] + 0.5 * (road_rules["x_max"] - road_rules["x_min"])
        y_center = road_rules["y_min"] + 0.5 * (road_rules["y_max"] - road_rules["y_min"])

        ax.plot([road_rules["x_min"], road_rules["x_min"]], [0, y_max], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_max"], road_rules["x_max"]], [0, road_rules["y_min"]], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_max"], road_rules["x_max"]], [road_rules["y_max"], y_max], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_min"], road_rules["x_min"]], [road_rules["y_min"], road_rules["y_min"]], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_max"], x_max], [road_rules["y_min"], road_rules["y_min"]], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_min"], road_rules["x_min"]], [road_rules["y_max"], road_rules["y_max"]], c="k", linewidth = 2, zorder = -1)
        ax.plot([road_rules["x_max"], x_max], [road_rules["y_max"], road_rules["y_max"]], c="k", linewidth = 2, zorder = -1)
        ax.plot([x_center, x_center], [0, y_max], "--", c = 'k', linewidth = 2, dashes=(5, 5), zorder = -1)
        ax.plot([road_rules["x_max"], x_max], [y_center, y_center], "--", c = 'k', linewidth = 2, dashes=(5, 5), zorder = -1)

    def plot(self, state):
        assert len(state) == 2, "State shape must be 2"
        ego_index = int(state[0].replace("ego_", "")) - 1
        car_index = int(state[1].replace("car_", "")) - 1
        # plot sample T-intersection and three sample trajectories
        # (one for our car and 2 for the other car)
        plt.figure(0, figsize=(20/3, 15/3))
        plt.axis("off")

        plt.plot(self.car_traj_x(np.linspace(0, self.T, self.N)), self.car_traj_y(np.linspace(0, self.T, self.N)), "--r", alpha=0.5)
        for i in np.arange(0, self.T, 10.0*self.dt):
            plt.scatter(self.car_traj_x(i), self.car_traj_y(i), c="r", s=50, alpha=0.5)
        
        plt.scatter(self.car_traj_x(car_index*10*self.dt), self.car_traj_y(car_index*10*self.dt), c="r", s=200)

        plt.plot(self.ego_traj_x(np.linspace(0, self.T, self.N)), self.ego_traj_y(np.linspace(0, self.T, self.N)), "--b", alpha=0.5)
        for i in np.arange(0, self.T, 10.0*self.dt):
            plt.scatter(self.ego_traj_x(i), self.ego_traj_y(i), c="b", s=50, alpha=0.5)

        plt.scatter(self.ego_traj_x(ego_index*10*self.dt), self.ego_traj_y(ego_index*10*self.dt), c="b", s=200)

        plt.xlim(0, 20)
        plt.ylim(15, 30)

        ax = plt.gca()
        self.plot_road(ax)