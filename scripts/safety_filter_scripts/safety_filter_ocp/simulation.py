import logging
import numpy as np
from acados_template import AcadosOcpSolver, AcadosSimSolver

from .helper import MapInfo, get_relevant_obstacles


sim_logger = logging.getLogger("SF.SIMULATE")

def simulate(
        init_state: np.ndarray,
        unsafe_inputs: np.ndarray, 
        map_info: MapInfo,
        safety_radius: float,
        ocp_solver: AcadosOcpSolver, 
        sim_solver: AcadosSimSolver = None,
        N_sim: int = 1,
        print_statistics: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    N_horizon = ocp_solver.acados_ocp.solver_options.N_horizon
    nx = ocp_solver.acados_ocp.model.x.shape[0]  # Number of states
    nu = ocp_solver.acados_ocp.model.u.shape[0]  # Number of controls

    x_traj = np.full((N_sim, N_horizon + 1, nx), np.nan)
    u_traj = np.full((N_sim, N_horizon, nu), np.nan)
    x_sim = np.full((N_sim + 1, nx), np.nan)
    x_current = init_state.copy()
    x_traj[0, 0] = init_state.copy()

    sim_logger.info("Starting simulation...")
    for i in range(N_sim):
        # set initial state
        ocp_solver.set(0, "lbx", x_current)
        ocp_solver.set(0, "ubx", x_current)

        # set yref
        ocp_solver.set(0, "yref", 
            np.concatenate((
                unsafe_inputs[i] if i < len(unsafe_inputs) else unsafe_inputs[-1], 
                np.zeros(ocp_solver.acados_ocp.dims.nu))))
        
        # set obstacles
        num_obs = (ocp_solver.acados_ocp.dims.np - 2) // 2 
        relevant_obstacles = get_relevant_obstacles(
            map_info=map_info,
            robot_position=x_current[:2],
            local_patch_size_m=50*map_info.resolution,
            max_obstacles_to_return=num_obs
        )
        num_active_obstacles = len(relevant_obstacles)
        p_values = np.zeros(ocp_solver.acados_ocp.dims.np)
        p_values[:2] = [0.2, 0.2]
        if num_active_obstacles > 0:
            obs_flat = relevant_obstacles.flatten()
            p_values[2:len(obs_flat)+2] = obs_flat

        for k in range(N_horizon + 1):
            ocp_solver.set(k, "p", p_values)

        # Setze den Safety Radius
        lh_active = np.full((num_active_obstacles * 2,), safety_radius**2)
        lh_inactive = np.full((ocp_solver.acados_ocp.dims.nh - num_active_obstacles * 2,), -1e1)
        lh_values = np.concatenate((lh_active, lh_inactive))

        for k in range(1, N_horizon):
            ocp_solver.constraints_set(k, "lh", lh_values)

        # Solve OCP
        status = ocp_solver.solve()
        if status != 0:
            sim_logger.warning(f"Solver failed at stage {i} with status {status}")
            if print_statistics:
                ocp_solver.print_statistics()
            break
        
        # Get states and inputs
        for k in range(N_horizon + 1):
            xk = ocp_solver.get(k, "x")
            x_traj[i, k] = xk
        for k in range(N_horizon):
            uk = ocp_solver.get(k, "u")
            u_traj[i, k] = uk

        # Simulate
        if sim_solver is not None:
            sim_solver.set('x', x_current.copy())
            sim_solver.set('u', u_traj[i, 0])
            status_sim = sim_solver.solve()
            if status_sim != 0:
                sim_logger.warning(f'Simulator failed with status {status_sim}')
                break
            x_current = sim_solver.get('x')
            x_sim[i + 1] = x_current.copy()
        else:
            x_current = x_traj[i, 1]

        if i < (N_sim - 1):
            x_traj[i + 1, 0] = x_current.copy()

        sim_logger.debug("States:")
        for k, xk in enumerate(x_traj[i]):
            value_strings = [f"{xkj:.3g}" for xkj in xk]
            sim_logger.debug(f"Stage {k}: {value_strings}")

        sim_logger.debug("Inputs:")
        for k, uk in enumerate(u_traj[i]):
            value_strings = [f"{uk_j:.3g}" for uk_j in uk]
            sim_logger.debug(f"Stage {k}: {value_strings}")

    sim_logger.info("Simulation finished")
    return x_traj, u_traj, x_sim
