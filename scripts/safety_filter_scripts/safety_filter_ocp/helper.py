import numpy as np

from dataclasses import dataclass, field


@dataclass
class MapInfo:
    origin: np.ndarray
    grid: np.ndarray
    resolution: float
    obstacle_positions: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        rows, cols = np.where(self.grid == 1)
        obs_x = self.origin[0] + (cols + 0.5) * self.resolution
        obs_y = self.origin[1] + (rows + 0.5) * self.resolution
        self.obstacle_positions = np.vstack((obs_x, obs_y)).T

    @property
    def height(self) -> int:
        return self.grid.shape[0]

    @property
    def width(self) -> int:
        return self.grid.shape[1]


def get_relevant_obstacles(
        map_info: MapInfo, 
        robot_position: np.ndarray,
        local_patch_size_m: float,
        max_obstacles_to_return: int
) -> np.ndarray:
    """
    Findet die k-nächsten Hindernis-Zellen zum Roboter.
    """
    # --- Schritt 1 & 2: Kandidaten finden (optimiert und wie bisher) ---
    robot_col = int((robot_position[0] - map_info.origin[0]) / map_info.resolution)
    robot_row = int((robot_position[1] - map_info.origin[1]) / map_info.resolution)
    
    patch_radius_cells = int(local_patch_size_m / 2 / map_info.resolution)
    row_start = max(0, robot_row - patch_radius_cells)
    row_end = min(map_info.height, robot_row + patch_radius_cells)
    col_start = max(0, robot_col - patch_radius_cells)
    col_end = min(map_info.width, robot_col + patch_radius_cells)
    
    local_grid = map_info.grid[row_start:row_end, col_start:col_end]
    
    occupied_rows_local, occupied_cols_local = np.where(local_grid == 1)
    if occupied_rows_local.size == 0:
        return np.empty((0, 3))

    occupied_cols_global = occupied_cols_local + col_start
    occupied_rows_global = occupied_rows_local + row_start
    
    obstacle_candidates_x = map_info.origin[0] + (occupied_cols_global + 0.5) * map_info.resolution
    obstacle_candidates_y = map_info.origin[1] + (occupied_rows_global + 0.5) * map_info.resolution
    obstacle_candidates = np.vstack((obstacle_candidates_x, obstacle_candidates_y)).T

    # --- Schritt 3: KNN-Auswahl (schnell und vektorisiert) ---
    dist_sq = np.sum((obstacle_candidates - robot_position)**2, axis=1)
    sorted_indices = np.argsort(dist_sq)
    
    # Wähle die 'max_obstacles_to_return' nächsten Indizes aus.
    num_obstacles_to_select = min(max_obstacles_to_return, len(sorted_indices))
    closest_indices = sorted_indices[:num_obstacles_to_select]
    
    final_obstacles = obstacle_candidates[closest_indices]
    
    return final_obstacles


def get_stage_ref(initial_state: np.ndarray, target_state: np.ndarray, target_input: np.ndarray): 
    ref_phi = np.arctan2(target_state[1] - initial_state[1], target_state[0] - initial_state[0])
    return np.array([target_state[0], target_state[1], ref_phi, target_state[3], target_input[0], target_input[1]])


def normalize_angle(angle: float):
    """Normalisiert einen Winkel auf den Bereich [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def generate_unsafe_trajectory(
        dt: float,
        start_state: np.ndarray, 
        map_info: MapInfo, 
):
    # Finde das Zentrum des Hindernisses aus der Occupancy Map
    rows, cols = np.where(map_info.grid == 1)
    center_row = np.mean(rows)
    center_col = np.mean(cols)
    
    # Konvertiere das Hinderniszentrum in Weltkoordinaten
    obstacle_center_y = center_col * map_info.resolution + map_info.origin[0]
    obstacle_center_x = center_row * map_info.resolution + map_info.origin[1]
    
    # Definiere Wegpunkte, die durch das Hindernis führen
    waypoints = [
        np.array([obstacle_center_x, obstacle_center_y + 1.5]), # Wegpunkt im Hindernis
        np.array([obstacle_center_x + 4.0, obstacle_center_y])    # Ziel nach dem Hindernis
    ]
    
    # --- 2. Simpler P-Regler zur Generierung der Steuerbefehle ---
    unsafe_commands = []
    unsafe_trajectory = [start_state.copy()]
    current_state = start_state.copy()
    
    # Regler-Parameter
    KP_angle = 2.5
    TARGET_SPEED = 1.0 # m/s
    WAYPOINT_THRESHOLD = 0.2 # m

    for target_wp in waypoints:
        distance_to_wp = np.linalg.norm(current_state[:2] - target_wp)
        
        while distance_to_wp > WAYPOINT_THRESHOLD:
            # Berechne Winkel zum Ziel
            error_vec = target_wp - current_state[:2]
            target_angle = np.arctan2(error_vec[1], error_vec[0])
            angle_error = normalize_angle(target_angle - current_state[2])

            # P-Regler
            v_cmd = TARGET_SPEED if abs(angle_error) < np.pi / 4 else 0.0
            omega_cmd = KP_angle * angle_error
            
            # Begrenze die Kommandos (optional, aber gute Praxis)
            v_cmd = np.clip(v_cmd, -1.5, 1.5)
            omega_cmd = np.clip(omega_cmd, -1.0, 1.0)
            
            command = np.array([v_cmd, omega_cmd])
            unsafe_commands.append(command)
            
            # Simuliere einen Schritt vorwärts mit dem Robotermodell
            px, py, psi, v, omega = current_state
            
            px_new = px + v * np.cos(psi) * dt
            py_new = py + v * np.sin(psi) * dt
            psi_new = psi + omega * dt
            
            # Annahme: Geschwindigkeiten folgen dem Befehl (vereinfacht)
            v_new = v_cmd
            omega_new = omega_cmd
            
            current_state = np.array([px_new, py_new, psi_new, v_new, omega_new])
            unsafe_trajectory.append(current_state.copy())
            
            distance_to_wp = np.linalg.norm(current_state[:2] - target_wp)

    return unsafe_commands, np.array(unsafe_trajectory)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    def plot_map(map_info: MapInfo, robot_pos: np.ndarray, relevant_obstacles: np.ndarray):
        fig, ax = plt.subplots(figsize=(10, 10))

        # BUGFIX: shape[1] für die Breite (x-Achse) und shape[0] für die Höhe (y-Achse) verwenden
        extent = [map_info.origin[0], map_info.origin[0] + map_info.width * map_info.resolution,
                  map_info.origin[1], map_info.origin[1] + map_info.height * map_info.resolution]
        
        ax.imshow(map_info.grid, cmap='Greys', origin='lower', extent=extent)
        
        # Optional: Alle Hindernispunkte in einer anderen Farbe plotten (z.B. gelb)
        if map_info.obstacle_positions.size > 0:
            ax.plot(map_info.obstacle_positions[:, 0], map_info.obstacle_positions[:, 1], 
                    's', markersize=2, color='darkkhaki', label='Alle Hindernis-Zellen')

        # NEU: Die vom Algorithmus als relevant befundenen Hindernisse plotten
        if relevant_obstacles.size > 0:
            ax.plot(relevant_obstacles[:, 0], relevant_obstacles[:, 1], 
                    'ro', markersize=8, label='Relevante Hindernisse (für MPC)', mfc='none')
            
        # NEU: Die Roboterposition plotten
        ax.plot(robot_pos[0], robot_pos[1], 'r*', markersize=15, label='Roboter')
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('MPC-Navigation: Relevante Hindernisse')
        ax.legend()
        ax.grid(True)
        ax.set_aspect('equal')

    occupancy_grid_map = np.zeros((40, 50))
    occupancy_grid_map[18:22, 5:35] = 1 # Breites Hindernis
    occupancy_grid_map[10:12, 18:22] = 1 # Kleines Hindernis weiter weg

    # Erstelle das MapInfo-Objekt
    map_info = MapInfo(
        origin=np.array([-5.0, -5.0]), # Ursprung der Karte in der Welt [x, y]
        grid=occupancy_grid_map,
        resolution=0.5 # Jede Zelle ist 50x50 cm
    )

    # Roboter befindet sich nahe am Ursprung
    robot_pos = np.array([0.0, 0.0])

    print("Alle Hindernispositionen (aus MapInfo):")
    # print(map_info.obstacle_positions) # Das sind sehr viele!
    print(f"Anzahl aller Hindernisse: {len(map_info.obstacle_positions)}")
    print("-" * 30)


    # Finde die relevanten Hindernisse für den MPC-Solver
    relevant_obstacles = get_relevant_obstacles(
        map_info=map_info,
        robot_position=robot_pos,
        local_patch_size_m=25.0, 
        num_angular_bins=25, 
        max_obstacles_to_return=20 
    )

    print(f"Roboterposition: {robot_pos}")
    print(f"Anzahl relevanter Hindernisse gefunden: {len(relevant_obstacles)}")
    print("Koordinaten der relevanten Hindernisse:")
    print(relevant_obstacles)

    plot_map(map_info, robot_pos, relevant_obstacles)
    plt.show()