# Lab 2 - Trajectory Planning with ILQR

**[Due 11:59PM Thursday, February 13]**

This lab will focus on the fundamental robot trajectory planning problem using optimization-based methods. We will first express the trajectory planning problem as an optimal control problem and look into vehicle models that govern our robots' equations of motion. Then, we will utilize the iterative linear quadratic regulator (ILQR) to generate locally optimal trajectories and policy. In addition, we will design a receding horizon trajectory planner using your ILQR and test them on the simulator and the real robot.
	
There are **4 Tasks** and **2 Checkpoints** in this lab, and you will need to submit the results and demonstrate them to the AI before **11:59PM February 13, 2024**.

\input{lab2/sections/intro.tex}


\input{lab2/sections/ILQR.tex}




# ILQR as a Policy Planner
\input{lab2/sections/policy.tex}

# Receding Horizon Trajectory Planner with ILQR
\input{lab2/sections/mpc.tex}

# Testing Your Planner on Mini-Truck

\input{lab2/sections/truck}

## Getting Started ##
**Note**: Make sure you have **pulled the code from upstream** into your repository and **updated all submodules**, i.e.,
[TODO: DOUBLE CHECK COMMAND, UPDATE-SUBMODULE?]
```bash
git pull upstream 2025 --recurse submodules
```


If you encounter the `ModuleNotFoundError`, please install missing packages using `mamba install <package_name>` under the `ros_base` environment. For example, to fix `ModuleNotFoundError: No module named sklearn`, you can use `mamba install scikit-learn`. 

## Software Structure
In this lab, you will build a trajectory planner for our robot. Specifically, we will develop the `racecar_planner` ROS package under the directory [ROS_Core/src/Labs/Lab2](https://github.com/SafeRoboticsLab/ECE346/tree/SP2025/ROS_Core/src/Labs/Lab2). The basic software structure can be found in **Figure 1**.

![Software Structure for Lab 2](assets/file.png) [TODO: FILL IN]

***Figure 1**: Software Structure for Lab 2*
[TODO: CHECK THIS FORMAT, LINKS, ASK TAS]
We will implement the ILQR algorithm in class ([`ILQR`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/ilqr.py#L17)) and test it in the Jupyter Notebook ([`task1.ipynb`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/task1.ipynb)). Then, we will develop open-loop and receding horizon trajectory planning algorithms with ROS inside ([`TrajectoryPlanner`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23)) class. We will compare their performances in simulation and the real robot.



## Trajectory Planning by Optimization
 We can formulate a trajectory planning problem as a discrete-time optimal control problem with a finite horizon $horizon$:

[TODO: FIND BETTER WAY TO TYPE EQUATION (GO TO OVERLEAF)]\
[TODO START]

    $ 
    \min_{csig} J(traj, csig) := \sum_{tdisc=0}^{horizon} costfunc(state_tdisc, ctrl_tdisc)  
    $
    \st \ \ & \state_{\tdisc+1} = \dyn (\state_\tdisc, \ctrl_\tdisc), ~ \tdisc = 0, 1, \cdots, \horizon-1 \nonumber 
    $

    $
    \delta = \arctan \left(\frac{2 L \sin(\alpha)}{l_d}\right)
    $

    where we want to find a desired control sequences $\csig := (\ctrl_0, \cdots, \ctrl_\horizon)$ that leads to a trajectory $\traj := (\state_0, \cdots, \state_\horizon)$, minimizes the cost $J$ over next $H$ steps.\\

    ## Robot Dynamics $\dyn(\state_\tdisc, \ctrl_\tdisc)$

Throughout this semester, we will use the kinematic bicycle model to describe the 2D motion of ground vehicles. As shown the **Figure 2**, instead of modeling all four wheels, we combine the front wheels as a single wheel at $F$, and represent both rear wheels as a single wheel at $R$. The steering angle $\delta$ is the angle between the front wheel and the longitudinal axis of the robot.

![Kinematic Bicycle Model.](assets/bicycle_model_flat.jpg) [TODO: FILL IN]

***Figure 2**: Kinematic Bicycle Model.*

 We assume the entire robot is a point mass at $R=\begin{bmatrix} X&Y \end{bmatrix}$ position, and the heading angle of the robot is $\psi$. The longitudinal velocity of the robot is $v$, and the steering angle is $\delta$. In addition, we also assume tires are under no-slip conditions so that both wheels' velocities align with their directions.\\
 
 Let us consider the state of robot $\state = \begin{bmatrix}
X & Y & \velocity & \psi & \delta
\end{bmatrix}^\top$. Under the kinematic bicycle model, the system dynamics can be expressed as 
\begin{equation}
	\begin{bmatrix}
		\dot{X} \\ 
		\dot{Y} \\ 
		\dot{\velocity}\\
		\dot{\psi} \\
            \dot{\delta}
% 		\dot{\delta}
	\end{bmatrix} = \begin{bmatrix}
		\velocity\cos(\psi) \\
		\velocity\sin(\psi) \\
		a\\
		\frac{\velocity}{L}\tan(\delta)\\
            \omega
	\end{bmatrix}
	\label{eq: kinematic_bicycle_model_flat}
\end{equation}
where the system has control $\ctrl = \begin{bmatrix}
    a & \omega
\end{bmatrix}$ as $a$ is the longitudinal acceleration ($[m/s^2]$) and ${\omega}$ is the rate of steering ($[rad/s]$).\\

This kinematic bicycle dynamic has been implemented in the [`Bicycle5D`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/dynamics/bicycle5d.py#L8) class. 
In addition, we provide you with very efficient implementations of trajectory rollout and derivative using [Jax](https://jax.readthedocs.io/en/latest/). Specifically, you will find [`integrate_forward_np`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/dynamics/bicycle5d.py#L55) and 
[`get_jacobian_np`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/dynamics/bicycle5d.py#L73) useful for your ILQR. Please refer to their docstrings for instructions.

## Cost Function $\costfunc_\tdisc(\state_\tdisc, \ctrl_\tdisc)$
% Our robot receives a path and other reference information from the route planner. 
One way to solve the optimal control problem posed in \autoref{eq: ocp} is using ILQR, which will find locally optimal control sequences by minimizing the cost function. Typical costs for our robot include deviation from the reference trajectory and velocity, penalties for large control values and collision, etc. By combining various cost functions with different weights, you can generate characteristic behaviors using your ILQR algorithm. As the example given in *Figure 3**, the ILQR finds a time-optimal trajectory in a racetrack, whose centerline and track boundary are provided.  

![Trajectory around [Motorsport Arena Oschersleben](https://www.racingcircuits.info/europe/germany/oschersleben.html) generated by ILQR.](assets/traj_example.png) [TODO: FILL IN]

***Figure 3**: Trajectory around [Motorsport Arena Oschersleben](https://www.racingcircuits.info/europe/germany/oschersleben.html) generated by ILQR.*


We have implemented a set of cost functions within the [`Cost`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/cost/cost.py#L13) class, whose parameters can be defined by your configuration file. The description of each cost function and its parameters can be found in the code. In addition, we provide you with very efficient implementation to obtain Jacobian and Hessian of the cost function using [Jax](https://jax.readthedocs.io/en/latest/). Specifically, you will find [`get_derivatives_np`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/cost/cost.py#L48) and [`get_traj_cost`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/cost/cost.py#L21) useful for your ILQR. Please refer to their docstrings for instructions.

**In Lab 2, cost parameters for all tasks are provided. You are certainly welcome but not required to fine-tune those parameters.**

### Task 1: Implementing ILQR Algorithm
Unlike the tasks you had in Lab 0, Task 1 is very open-ended. You will need to complete the main ILQR loop in the [`plan`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/ilqr.py#L133) function of [`ILQR`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/ILQR/ilqr.py#L17) class following pseudocodes provided in the ILQR handout.

The `plan` function takes in **the initial state $\state_0$} and **optional initial control sequences $\ncsig$}. After optimization using ILQR, it outputs a **dictionary} containing **planned trajectory $\traj$, control sequences $\csig$, feedback gain $\{\closedloop\}$}, and other information.

We have provided helper functions to compute cost and system rollout, as well as their derivatives. Detailed information can be found in **the comment block of `plan` function. Once finished, test your planner with provided Jupyter Notebook [`task1.ipynb`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/task1.ipynb) and show visualization results to your AIs.


In Task 1, your ILQR generated a reference trajectory $\traj=\{\hat{\state}_0,\cdots,\hat{\state}_T\}$ and reference control $\csig=\{\hat{\ctrl}_0,\cdots,\hat{\ctrl}_T\}$ to complete the time trial on a racetrack. In addition, ILQR provides a local state feedback control policy to track the reference trajectory at each time step. For example, if the current state of the robot is $\state_\tdisc$, the feedback control can be found as:
\begin{equation}
    \ctrl_\tdisc = \hat{\ctrl}_\tdisc + \closedloop(\state_\tdisc - \hat{\state}_\tdisc).
    \label{eq: feedback_control}
\end{equation}

### Task 2: Computing Feedback Control
We have implemented the function to attain polices to traverse along a reference path in the [`policy_planning_thread`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L344) function of the [`TrajectoryPlanner`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23) class. In Task 2, you are asked to compute the robot's control as described in \autoref{eq: feedback_control} by completing the [`compute_control`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L192) function of the [`TrajectoryPlanner`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23) class.

After finishing Task 2, you can test the ILQR within our simulated environment. Launch your ROS nodes using the following:
```bash
roslaunch racecar_planner ilqr_simulation.launch
```
After seeing `ILQR warm up finished` on your terminal, you can choose any point on the map using **2D Nav Goal** on your RViz. In **Figure 4**, we show an exemplary open-loop trajectory planned by the ILQR, where the red line is the reference path from the route planner and the green line is ILQR planned trajectory. Demonstrate your simulation results to AIs to check out Task 2.


![Example of Task 2 Results](assets/task2_result.png)

***Figure 4a**: Example of Task 2 (Policy Planner) Results*

![Example of Task 3 Results](assets/task3_result.png)

***Figure 4b**: Example of Task 3 (Receding Horizon Planner) Results*

Instead of computing the entire plan to track the reference path, we can utilize ILQR in a receding horizon fashion. Every time when the ROS node receives a new pose, we call ILQR to generate a new plan over a short horizon and use planned policy to generate controls.

### Task 3: Implementing the Receding Horizon Planner

In this task, you will need to finish the [`receding_horizon_planning_thread`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L409) function of the [`TrajectoryPlanner`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2025/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23) class. You may find comment blocks inside this function helpful for your implementation. Once finished, test your receding horizon planner by launching:
```bash
roslaunch racecar_planner ilqr_simulation.launch receding_horizon:=true
```

After seeing `ILQR warm up finished` on your terminal, you can choose any point on the map using **2D Nav Goal** on your RViz and verify your receding horizon planner. Think about the advantages and disadvantages of the policy planner in Task 2 and the receding horizon planner in this task. Share your thoughts with AIs and demonstrate your simulation to check out this task.

The modularity of ROS allows us to quickly deploy our algorithms from the simulated environment into the real robot with minimal changes to your code. Follow the instructions in the [TODO: UPDATE NAME OF DOC] `Intro to Mini Truck` tutorial and test your trajectory planner on the Mini Truck with the provided `ilqr_truck.launch`. You can use `receding_horizon` option to choose between policy planner and receding horizon planner.

![Update dynamic reconfigure parameters using RQT](assets/dyn_reconfig.png)

***Figure 5**: Update dynamic reconfigure parameters using RQT*

## Updating Parameters Using Dynamic Reconfigure
Due to hardware limitations, you might find it necessary to tune the direction and center point of the steering control, as well as the latency composition value to improve the performance of your planner on the robot. Instead of passing those values as ROS parameters, and setting them by re-launching, we can use **ROS Dynamic Reconfigure** to adjust them on the fly. Detailed tutorials on Dynamic Reconfigure can be found [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials). You can adjust those parameters using RQT as shown in **Figure 5**.

### Task 4: Demonstrating ILQR Planner on Robot
Finally, test your planner on the mini-truck robot and demonstrate its performance with your AIs.

