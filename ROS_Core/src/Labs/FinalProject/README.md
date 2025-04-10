# Final Project: Safety-Critical Navigation
**[Due 4:00PM Thursday, May 5]**

The final project consists of **two** tasks:
* **Obstacle Track Racing (40 points)**
* **Safety Filter (40 points)**

There will also be a graded written report due on May 9th.

## Preparation

### Simulation

To ensure your files are up to date, make sure to run:

```bash
git pull upstream SP2025 --recurse-submodules
```
To test that this works, run

```bash
cd ROS_Core
rm -rf build devel # delete old compiled files
conda install pynput #needed for Keyboard Control
catkin_make # recompile files (now including final_project pkg)
source devel/setup.bash # source environment variables (now final_project is found!)
roslaunch final_project task1_simulation.launch
```

You should be able to see a purple line denoting the reference path, and three lines denoting the left lane boundary (indigo), right lane boundary (red), and ILQR path (light blue). These will help you with debugging situations where the robot appears to leave the lane.

Notice that the simulation is horrendous at avoiding obstacles. You will improve upon the base algorithm to more effectively avoid obstacles in Task 1.

### Truck

For your truck to successfully detect static obstacles with SLAM, you will have to make a change to the internal files on your truck. This will involve using a command-line text editor (like `vim` or `nano`).

```bash
cd ~/StartUp/src/AprilTagSLAM_ROS/config
nano config.yaml # or vim config.yaml
```

You can find the lines with `publish_image_with_tags` and `publish_tags`. Replace the `false` in each of these lines with `true`. Then, if using `nano`, you can press `Ctrl X Y` to save. (or `Esc :wq` for `vim`) Afterwards, run the following commands:

```bash
cd ~/StartUp
catkin_make_isolated
```

To test that this works, you can put a static obstacle cube in front of the robot

```bash
roslaunch final_project task1_detection.launch
```
Note: you have to run the 3 terminals as before; Thus the line above is the 3rd terminal where you run your algorithm.

Once you start SLAM, a static obstacle should appear in real-time (and change position if you move it around). This should work with multiple static obstacles, assuming no occlusion.

## Task 1: Obstacle Track Racing

You will design an improved algorithm to drive around a cluttered obstacle course. Below are some tips to help you get started.

### Goal and Obstacle Files

See [task1.yaml](task1.yaml) for an adjustable list of goals and obstacles. Feel free to adjust the obstacle positions, but do not change the locations of the intermediate goals, as your truck must complete the course in the specified order to earn credit.

### Trajectory Planner

View [traj_planner.py](scripts/traj_planner.py) for a baseline trajectory planner. **You will likely need to adjust the ILQR reference path to ensure that the algorithm converges to a safer path.**

For a suggestion of where you can change code, reference the `TODO (Task 1)` marker (through a `Ctrl F` search).

### ILQR Hyperparameters

Like all optimization algorithms, ILQR has many hyperparameters that can make the difference between properly avoiding obstacles and slamming directly into them. See a configurable set of hyperparameters in [task1_ilqr.yaml](cfg/task1_ilqr.yaml).

**You will absolutely need to adjust these hyperparameters to get your algorithm working smoothly and safely on the truck. Our parameters were tuned for simulation, and variations in truck speeds, friction, etc may cause unexpected behavior.**

Here is a quick list of a few hyperparameters you should consider changing, as well as places you can find documentation on them in the ILQR planner code:
* [Velocity limits](./scripts/ILQR/dynamics/bicycle5d.py) and [velocity reference](./scripts/ILQR/ref_path.py):
  * If you find that your robot is consistently moving too slow or fast, consider manipulating `v_max`, `v_min`, and `v_ref`.
* [Path offset cost](./scripts/ILQR/cost/state_cost.py):
  * If the robot is too closely adhering to the reference path and not leaving it to avoid obstacles, consider manipulating `path_weight`.
* [Velocity Cost](./scripts/ILQR/cost/state_cost.py):
  * If you find that your robot is not sufficiently slowing down when an obstacle appears, consider decreasing `vel_weight`. If the robot is too quickly changing speed, consider increasing it.
* [Heading Cost](./scripts/ILQR/cost/state_cost.py):
  * If you find that your robot is not sufficiently veering when an obstacle appears, consider decreasing `heading_weight`. If the robot is oscillating widely in heading, consider increasing it.
* [Lane Boundary Cost](./scripts/ILQR/cost/state_cost.py):
  * If you find that your robot is staying too far away from (or straying too close to) lane boundaries, consider changing `lane_boundary_a` and `lane_boundary_b`. Together, they form an [`exp_linear_cost`](./scripts/ILQR/cost/base_cost.py), with `lane_boundary_a` serving as an exponent and `lane_boundary_b` serving as a linear scaling factor.
* [Obstacle Cost](./scripts/ILQR/cost/obstacle_cost.py):
  * If you find that your robot ignores obstacles, consider changing `obs_a` and `obs_b`, which again form an [`exp_linear_cost`](./scripts/ILQR/cost/base_cost.py).

Note that changing just a few of these hyperparameters should be good enough to satisfactorily avoid obstacles. **Make sure you leave enough time to experiment with different values on your truck. Tuning ILQR hyperparameters can take many hours to get right.**

### Lanelets

In the [traj_planner.py](scripts/traj_planner.py) file, there is some helper code in `path_callback` that references `pylanelet`, a library that helps break the course into smaller lanes.

We recommend thinking about a lane-switching protocol, to force the reference path to switch to another lane if an obstacle intersects with the reference path. You can use [pylanelet](../../Utility/Routing/script/routing/pylanelet) and [test_lanelet.ipynb](../../Utility/Routing/script/routing/test_pylanlet.ipynb) as references.

Note that abruptly switching lanes on the reference path will lead to a jagged reference path, which is bad for ILQR. You can use methods like **moving-average filtering** or **gaussian filtering** (both of which are low-pass filters) on the coordinates of the reference path.

## Task 2: Safety Filter

You will design an algorithm to override unsafe inputs (like steering off the track, running into obstacles, and veering into a lane with oncoming traffic).

**Make sure to leave enough time to complete Task 2. Unlike Task 1, we have intentionally left the intended approach open-ended.**

**For example, you could build upon ILQR (use obstacle distances or some cost threshold to choose when to override a user) or train an aversarial machine learning approach.**

### Keyboard Subscriber and Publisher

Inspect the [keyboard_control.py](./scripts/keyboard_control.py) file, and add a subscriber. Look for keywords `TODO (Task 2)`. 

in the `ros_base` terminal, run `conda install pynput`

Inspect the [traj_planner.py](./scripts/traj_planner.py) file, and add a publisher. Look for keywords `TODO (Task 2)`.

Inspect the [task2.yaml](./task2.yaml) file to add your own obstacles and goals

Inspect the [task2_ilqr.yaml](./cfg/task2_ilqr.yaml) file to manipulate costs


### Getting Started

We suggest viewing the `control_thread` and `receding_horizon_planning_thread` functions, and looking for places to add new code. Feel free to add additional helper functions.
