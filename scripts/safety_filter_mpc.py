import numpy as np


def setup_weights() -> tuple[np.ndarray, np.ndarray]:
    # Cost weights
    v_scale = 1e1
    omega_scale = 1e-1
    R_ref = np.diag([1.5 * v_scale, 1. * omega_scale])
    R_delta = np.diag([1. * v_scale, 3. * omega_scale])
    return R_ref, R_delta


# %%
if __name__ == "__main__":
    import argparse
    import os
    import numpy as np

    from safety_filter_scripts.logging_setup import setup_logger
    from safety_filter_scripts.safety_filter_ocp import *

    from logging import DEBUG, INFO, WARN, ERROR
    logger = setup_logger()
    logger.setLevel(INFO)
    
    cwd = os.path.dirname(__file__)
    default_cgencode = os.path.join(cwd, "c_generated_code")

    parser = argparse.ArgumentParser(description='Generate ACADOS solver code.')
    parser.add_argument('--gen_code_path', type=str, default=default_cgencode, help='The path for the generated c code.')
    parser.add_argument('-p', '--plot', action='store_true', help='Plot the occupancy map and the input references and resulting inputs.')
    parser.add_argument('--num_obs', type=int, default=10, help='The number of occupied fields used in the MPC.')
    parser.add_argument('--horizon', type=int, default=15, help='The prediction horizon for the MPC.')
    parser.add_argument('--gen_acados_ros', type=bool, default=False, help='If true, generate acados inbuild ROS2 interfaces.')
    args = parser.parse_args()

    N_horizon = args.horizon
    dt = 0.1

    # --- Occupancy Map ---
    map_origin = np.array([0.0, 0.0])
    map_resolution = 0.1
    occupancy_map = np.zeros((100, 100))
    occupancy_map[20:50, 30:50] = 1 # Hindernis 1
    map_info = MapInfo(map_origin, occupancy_map, map_resolution)


    # --- Solver setup ---
    safety_radius = map_info.resolution * np.sqrt(2) + 0.3
    R_ref, R_delta = setup_weights()
    ocp_solver = create_solver(N_horizon, dt, args.num_obs, safety_radius**2, R_ref, R_delta, args.gen_code_path, generate_acados_ros=args.gen_acados_ros)
    sim_solver = create_sim(dt*1.3, args.gen_code_path)
    
    x0 = np.array([0.0, 0.0, np.pi/4, 0.5, 0.0])
    unsafe_inputs, unsafe_path = generate_unsafe_trajectory(dt, x0, map_info)

    x_traj, u_traj, x_sim = simulate(
        x0, 
        unsafe_inputs, 
        map_info,
        safety_radius,
        ocp_solver=ocp_solver, 
        sim_solver=sim_solver,
        N_sim=100,
        print_statistics=False
    )

    if args.plot:
        import matplotlib.pyplot as plt
        import safety_filter_scripts.plots as sfsp
        sfsp.plot_map(map_info, x0, unsafe_path, x_sim, x_traj)
        sfsp.plot_inputs(dt, np.vstack(unsafe_inputs), u_traj[:, 0, :])
        plt.show()

    print(f"Filled states in trajectory (not NaN): {x_traj.shape[0] - np.count_nonzero(np.isnan(x_traj[:,2,0]))} / {x_traj.shape[0]}")