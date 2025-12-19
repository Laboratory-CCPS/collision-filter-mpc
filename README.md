# Safety Filter MPC
This package implements an MPC safety filter using [acados](https://docs.acados.org/). Acados can also be simply installed with [acados installation script](acados_install.sh), by calling:
```bash
./acados_install.sh
```

Check the acados with this command, while there are some extra arguments that can be parsed.
```bash
python sripts/safety_filter_mpc.py -p
```

#### Parsing Arguments
| Argument | Default | Values | Description |
|---|---|---|---|
| `-p`, `--plot` | `false` | `N/A` (flag) | If set, generates and displays plots. |
| `--gen_code_path` | `.` | `</path/to/dir>` | Specifies the directory to save the generated C code. |
| `num_obs` | `10` | `Integer > 0` | Sets the number of obstacles to consider from the occupancy grid. |
| `horizon` | `15` | `Integer > 0` | Defines the prediction horizon length (N) for the MPC. |
| `gen_acados_ros` | `false` | `N/A` (flag) | If set, generate acados inbuild ROS2 interfaces. (Not yet in the official acados repo) |


## Optimal Control Problem (OCP) Formulation

The goal of the safety filter is to solve an Optimal Control Problem (OCP) over a prediction horizon $N$. The objective is to find a sequence of control inputs $u_0, ..., u_{N-1}$ that minimizes a given objective function while adhering to all system and safety constraints.

### 1. Prediction Model (System Dynamics)

The underlying [model](scripts/safety_filter_scripts/safety_filter_ocp/skid_steer_model.py) is a skid-steer model.

* **State Vector** $x \in \mathbb{R}^5$:  
$$
x = 
\begin{bmatrix}
p_x \\\\
p_y \\\\
psi \\\\
v \\\\
omega
\end{bmatrix}
$$

    Where $(p_x, p_y)$ are the position, $\psi$ is the orientation, $v$ is the linear velocity, and $\omega$ is the angular velocity.

* **Control Input Vector** $u \in \mathbb{R}^2$:  
$$
u = 
\begin{bmatrix} 
v_{c} \\\ 
\omega_{c} 
\end{bmatrix} $$
    These are the target velocities sent to the system.

* **Continuous System Dynamics** $\dot{x} = f(x, u)$:  
$$
\dot{x} = 
\begin{bmatrix}
v \cos(\psi) \\\
v \sin(\psi) \\\
\omega \\\
(v_{c} - v) / \tau_v \\\
(\omega_{c} - \omega) / \tau_\omega
\end{bmatrix}
$$  
    The parameters $\tau_v$ and $\tau_\omega$ are time constants that model the actuator dynamics.

---

### 2. Objective Function (Cost Function)

The [cost](scripts/safety_filter_scripts/safety_filter_ocp/solver.py#L83-L110) function $J$ is designed to find an optimal, safe control sequence that adheres to all constraints while staying as close as possible to the original, potentially unsafe reference command.

$$\min_{u, s} J = \underbrace{ \|u_0 - u_{\text{ref}}\|_{R_{\text{ref}}}^2 + \sum_{k=0}^{N-1} \|u_k - x_{\text{vel},k}\|_{R_{\delta}}^2 }_{\text{Initial and Stage Cost}} + \underbrace{ \sum_{k=1}^{N-1} \left( z_{l,k}^T s_{l,k} + \frac{1}{2}s_{l,k}^T Z_{l,k} s_{l,k} \right) }_{\text{Slack Cost}}$$

Where:
* **$u_k$**: The optimized, *safe* control input at time step $k$.
* **$u_{\text{ref}}$**: The external, *unsafe* reference command desired by the user or a higher-level planner.
* **$x_{\text{vel},k} = [v_k, \omega_k]^T$**: The velocity part of the state vector $x_k$.
* **$s_k \ge 0$**: Slack variables for the implementation of soft constraints for collision avoidance.
* **$R_{\text{ref}}, R_{\delta}, Z$**: Positive (semi-)definite weighting matrices.

The cost function balances three objectives:
1.  **Reference Tracking** (Term $\|u_0 - u_{\text{ref}}\|_{R_{\text{ref}}}^2$): The first control input $u_0$ should be as close as possible to the desired command $u_{\text{ref}}$.
2.  **Control Effort Minimization** (Term $|u_k - x_{\text{vel},k}|_{R_{\delta}}^2$): Penalizes the deviation of the target velocity $u_k$ from the actual velocity $x_{\text{vel},k}$, which reduces aggressive accelerations and promotes smooth motion.
3.  **Penalty for Soft Constraint Violation** (Term $\|s_k\|^2_{Z}$): Penalizes the violation of the collision avoidance constraints.

---

### 3. Constraints

The minimization is performed subject to the following [constraints](scripts/safety_filter_scripts/safety_filter_ocp/solver.py#L112-125) for all time steps $k \in \{0, \dots, N-1\}$:

1.  **Discrete System Dynamics**:
    $$x_{k+1} = F(x_k, u_k)$$
    Here, $F(x_k, u_k)$ is the RK4 discretized form of the continuous dynamics $f(x, u)$.

2.  **State and Input Constraints**:
    $$x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}$$
    These are typically box constraints. The velocity constraint is direction-dependent on the reference $v_{c,\text{ref},k}$:
    $$v_{c,k} \in 
    \begin{cases}
    [0, \min(v_{\max}, v_{c,\text{ref},k})] & \text{ for } v_{c,\text{ref},k} \ge 0 \\\
    [\max(v_{\min}, v_{c,\text{ref},k}), 0] & \text{ for } v_{c,\text{ref},k} < 0
    \end{cases}$$
    The angular velocity is constrained by its limits:
    $$\omega_k \in  [\omega_{\min}, \omega_{\max}]$$

3.  **Collision Avoidance (Soft Constraints)**:
    For each obstacle $i$, the distance from the robot to the obstacle must exceed a safety radius $r_{\text{unsafe}}$. This is formulated as a soft constraint to ensure feasibility:
    $$\begin{align*}
    (p_{x, \text{front}, k} - x_i)^2 + (p_{y, \text{front}, k} - y_i)^2 - r^2_{\text{unsafe}} &\ge -s_{k, i, \text{front}} \\\
    (p_{x, \text{rear}, k} - x_i)^2 + (p_{y, \text{rear}, k} - y_i)^2 - r^2_{\text{unsafe}} &\ge -s_{k, i, \text{rear}}
    \end{align*}$$
    where the slack variables must be non-negative: $s_k \ge 0$.

---

## ROS2 Humble build

For the standard case when acados is installed, build with: 
```bash
colcon build --packages-select safety_filter_mpc
``` 

The advanced build with e.g. a non [default acados venv](acados_install.sh) installation 
```bash
colcon build --packages-select safety_filter_mpc --event-handlers console_direct+ --cmake-args -DVENV_PYTHON_EXECUTABLE=/home/josua/.acados_env/bin/python -DNUM_OBS=12 -DHORIZON=15 -DCMAKE_EXPORT_COMPILE_COMMANDS=1 
``` 

#### Cmake Arguments
| Argument | Default | Values | Description |
|---|---|---|---|
| `VENV_PYTHON_EXECUTABLE` | System `python` | `</path/to/venv/bin/python>` | Specifies the Python executable from your `acados` virtual environment. |
| `NUM_OBS` | `10` | `Integer > 0` | Bakes the number of obstacles into the compiled code. |
| `HORIZON` | `15` | `Integer > 0` | Bakes the horizon length (N) into the compiled code. |
| `CMAKE_EXPORT_COMPILE_COMMANDS` | `OFF` | `ON`/`OFF` | Generates a `compile_commands.json` file for IDE code analysis. |


## Usage

### Launch File: `safe_teleop.launch.py`

To run an example with teleoperation with a controller, type in the command:
```bash
ros2 launch safety_filter_mpc safe_teleop.launch.py
```

Available launch arguments:

| Argument | Default | Values | Description |
|----------|---------|--------|-------------|
| `enable_tb3_sim` | `false` | `true`/`false` | Starts the TurtleBot3 Gazebo world and enables simulated time. |
| `enable_rviz` | `false` | `true`/`false` | Launches RViz with a predefined config. |
| `teleop` | `joy` | `none` / `joy` / `key` | Select teleop source: none, joy (`teleop_twist_joy`), or key (`teleop_twist_keyboard`) |
| `joy_node` | `none` | `none` / `wsl` / `joy` | Select joystick source: none, WSL evdev bridge (`wsl_joystick_bridge`), or standard `joy_node`. |
| `log_level` | `warn` | e.g. `debug`, `info`, `warn`, `error`, `fatal` | Unified ROS 2 log level applied to all launched nodes. |

Nodes launched depending on the arguments:
* `gazebo` (via `turtlebot3_world.launch.py`) if `enable_tb3_sim=true`
* `slam_toolbox` (always; uses `use_sim_time` when simulation enabled)
* `evdev_joy_node` when `joy_node=wsl` (need to download and install [wsl_joy_bridge](https://github.com/ArgoJ/wsl-joystick-bridge))
* `joy_node` when `joy_node=joy`
* Neither joystick node when `joy_node=none`
* `teleop_twist_joy` when `teleop=joy` – remaps `/cmd_vel` -> `/cmd_vel_unsafe`
* `teleop_twist_keyboard` when `teleop=key` – remaps `/cmd_vel` -> `/cmd_vel_unsafe`
* `safety_filter_mpc` – filters to `/cmd_vel`
* `rviz2` when `enable_rviz=true`

#### Examples

WSL joystick bridge with simulation + RViz:
```bash
ros2 launch safety_filter_mpc safe_teleop.launch.py enable_tb3_sim:=true enable_rviz:=true teleop:=joy joy_node:=wsl log_level:=info
```

Note: Switch between `wsl` and `joy` depending on your environment (WSL vs. native Linux input devices). Use `none` for automated tests or playback of recorded trajectories without teleop.