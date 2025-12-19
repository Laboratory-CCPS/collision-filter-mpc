import matplotlib.pyplot as plt
import numpy as np

from .safety_filter_ocp.helper import MapInfo

def plot_inputs(
        dt: float,
        target_input: np.ndarray, 
        safe_input: np.ndarray
):
    time_vector_target = np.arange(len(target_input)) * dt
    time_vector_safe = np.arange(len(safe_input)) * dt
    
    # Erstelle die Abbildung und die Subplots
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Vergleich der Steuerbefehle', fontsize=16)
    
    # --- Subplot für lineare Geschwindigkeit (v) ---
    axs[0].plot(time_vector_target, target_input[:, 0], 'r--', label='Target (Unsafe) $v$')
    axs[0].plot(time_vector_safe, safe_input[:, 0], 'b-', label='Safe $v$')
    axs[0].set_ylabel('Geschwindigkeit [m/s]')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].set_title('Lineare Geschwindigkeit')
    
    # --- Subplot für Winkelgeschwindigkeit (omega) ---
    axs[1].plot(time_vector_target, target_input[:, 1], 'r--', label='Target (Unsafe) $\omega$')
    axs[1].plot(time_vector_safe, safe_input[:, 1], 'b-', label='Safe $\omega$')
    axs[1].set_ylabel('Winkelgeschw. [rad/s]')
    axs[1].set_xlabel('Zeit [s]')
    axs[1].legend()
    axs[1].grid(True)
    axs[1].set_title('Winkelgeschwindigkeit')


def plot_map(
        map_info: MapInfo,
        initial_state: np.ndarray | None = None, 
        unsafe_path: np.ndarray | None = None, 
        sim_state_history: np.ndarray | None = None, 
        optim_state_history: np.ndarray | None = None, 
):
    fig, ax = plt.subplots()
    extent = [map_info.origin[0], map_info.origin[0] + map_info.width * map_info.resolution,
                  map_info.origin[1], map_info.origin[1] + map_info.height * map_info.resolution]
    ax.imshow(map_info.grid, cmap='Greys', origin='lower', extent=extent)
    
    if optim_state_history is not None:
        for optim_traj in optim_state_history:
            ax.plot(optim_traj[:, 0], optim_traj[:, 1], 'c-')

    if map_info.obstacle_positions.size > 0:
        map_info.obstacle_positions = np.vstack(map_info.obstacle_positions)
        ax.plot(map_info.obstacle_positions[:, 0], map_info.obstacle_positions[:, 1], 'yo', markersize=4, label='Obstacle points')

    if unsafe_path is not None:
        ax.plot(unsafe_path[:, 0], unsafe_path[:, 1], 'r-o', markersize=3, label='Unsichere Trajektorie')

    if sim_state_history is not None:
        ax.plot(sim_state_history[:, 0], sim_state_history[:, 1], 'b-o', markersize=3, label='Trajektorie')

    if initial_state is not None:
        start_x, start_y, start_psi, *_ = initial_state
        ax.plot(start_x, start_y, 'ro', markersize=8, label='Start')
        arrow_length = 0.5
        ax.arrow(start_x, start_y, 
                arrow_length * np.cos(start_psi), 
                arrow_length * np.sin(start_psi),
                head_width=0.3, head_length=0.2, fc='r', ec='r')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('MPC-Navigation mit `acados` und Occupancy Map')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')