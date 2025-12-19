import os
import numpy as np
import scipy.linalg as spl
import casadi as ca

from acados_template import AcadosOcp, AcadosOcpSolver, builders, AcadosSim, AcadosSimSolver
from .skid_steer_model import get_skid_steer_model


def set_nonlinear_param_constraints(ocp: AcadosOcp, max_num_obs: int, r_unsafe_square):
    ocp.dims.np = max_num_obs * 2 + 2
    ocp.dims.nh = max_num_obs * 2
    
    ocp.dims.nsh = ocp.dims.nh
    ocp.constraints.idxsh = np.arange(ocp.dims.nh)

    p = ca.MX.sym('p', ocp.dims.np) 
    ocp.model.p = p

    px, py, psi = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
    d_front, d_rear = p[0], p[1]

    px_front = px + d_front * ca.cos(psi)
    py_front = py + d_front * ca.sin(psi)
    px_rear  = px - d_rear * ca.cos(psi)
    py_rear  = py - d_rear * ca.sin(psi)

    h_list = []
    for i in range(max_num_obs):
        xi, yi = p[2*i + 2], p[2*i + 3]
        d2_front = (px_front - xi)**2 + (py_front - yi)**2
        d2_rear  = (px_rear - xi)**2 + (py_rear - yi)**2
        h_list.append(d2_front)
        h_list.append(d2_rear)

    ocp.model.con_h_expr = ca.vertcat(*h_list)

    ocp.constraints.lh = np.full(ocp.dims.nh, r_unsafe_square)
    ocp.constraints.uh = np.full(ocp.dims.nh, 1e4)

    ocp.constraints.lsh = np.zeros(ocp.dims.nsh)
    ocp.constraints.ush = 1e4 * np.ones(ocp.dims.nsh)

    ocp.cost.zl = 1e3 * np.ones(ocp.dims.nsh)
    ocp.cost.Zl = 1e3 * np.ones(ocp.dims.nsh)
    ocp.cost.zu = 1e3 * np.ones(ocp.dims.nsh)
    ocp.cost.Zu = 1e3 * np.ones(ocp.dims.nsh)



def create_solver(
        N_horizon: int,
        dt: float, 
        max_num_obs: int, 
        r_unsafe_square: float, 
        R_ref: np.ndarray = None,
        R_delta: np.ndarray = None,
        gen_code_path: str = "",
        generate_acados_ros: bool = False
):
    """
    Erstellt und konfiguriert den AcadosOcpSolver.
    """
    ocp = AcadosOcp()
    ocp.model = get_skid_steer_model(dt)

    # --- Dimensions  ---
    ocp.dims.nx = ocp.model.x.size1()
    ocp.dims.nu = ocp.model.u.size1()

    # --- Cost ---
    v_scale = 1e1
    omega_scale = 1e-1

    if R_ref is None:
        R_ref = np.diag([1.5 * v_scale, 1. * omega_scale])
    if R_delta is None:
        R_delta = np.diag([1. * v_scale, 3. * omega_scale])

    nx = ocp.dims.nx
    nu = ocp.dims.nu

    # Initial Cost
    ocp.cost.cost_type_0 = "LINEAR_LS"
    ocp.dims.ny_0 = 2 * nu
    Vx_0 = np.zeros((ocp.dims.ny_0, nx))
    Vx_0[nu:2*nu, 3:5] = -np.eye(nu)
    ocp.cost.Vx_0 = Vx_0

    Vu_0 = np.zeros((ocp.dims.ny_0, nu))
    Vu_0[0:nu, :] = np.eye(nu)
    Vu_0[nu:2*nu, :] = np.eye(nu)
    ocp.cost.Vu_0 = Vu_0
    
    ocp.cost.W_0 = spl.block_diag(R_ref, R_delta)
    ocp.cost.yref_0 = np.zeros(ocp.dims.ny_0)

    # Stage Cost
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.dims.ny = nu
    Vx = np.zeros((ocp.dims.ny, nx))
    Vx[:, 3:5] = -np.eye(nu)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ocp.dims.ny, nu))
    Vu[:, :] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.W = R_delta
    ocp.cost.yref = np.zeros(ocp.dims.ny)

    # --- Constraints ---
    # Box-Constraints für Steuerungen
    v_max = 1.0
    omega_max = 1.5
    ocp.constraints.lbu = np.array([-v_max, -omega_max])
    ocp.constraints.ubu = np.array([v_max, omega_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # Box-Constraints für Zustände (Geschwindigkeit)
    ocp.constraints.lbx = np.array([-v_max, -omega_max])
    ocp.constraints.ubx = np.array([v_max, omega_max])
    ocp.constraints.idxbx = np.array([3, 4])

    set_nonlinear_param_constraints(ocp, max_num_obs, r_unsafe_square)

    # --- Parameter ---
    # Anfangszustand
    ocp.constraints.x0 = np.zeros(ocp.dims.nx)
    ocp.parameter_values = np.zeros(ocp.dims.np) 

    # --- Solver-Options ---
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    # ocp.solver_options.nlp_solver_tol_ineq = 1e-4
    # ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = dt * N_horizon

    # --- Solver creation ---
    if gen_code_path:
        ocp.code_export_directory = gen_code_path

    cm_builder = builders.ocp_get_default_cmake_builder()
    cm_builder.options_on = ['BUILD_ACADOS_OCP_SOLVER_LIB']

    if generate_acados_ros:
        from acados_template import AcadosOcpRosOptions
        ocp.ros_opts = AcadosOcpRosOptions()
        ocp.ros_opts.package_name = "safety_filter"

    json_file = os.path.join(os.path.dirname(__file__) if not gen_code_path else gen_code_path, 'safety_filter_ocp.json')
    if os.path.exists(json_file):
        os.remove(json_file)

    solver = AcadosOcpSolver(ocp, json_file=json_file, cmake_builder=cm_builder)

    return solver


def create_sim(
        dt: float,
        gen_code_path: str = ""
):
    sim = AcadosSim()
    sim.model = get_skid_steer_model(dt)
    sim.solver_options.T = dt
    sim.solver_options.integrator_type = 'ERK'
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 1

    json_file = os.path.join(os.path.dirname(__file__) if not gen_code_path else gen_code_path, 'skid_steer_sim.json')
    if os.path.exists(json_file):
        os.remove(json_file)

    acados_simulator = AcadosSimSolver(sim, json_file=json_file)
    return acados_simulator