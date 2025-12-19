import casadi as ca
from acados_template import AcadosModel

def get_skid_steer_model(dt):
    time_constant_v = max(0.010, dt)
    time_constant_yawrate = max(0.050, dt)

    model = AcadosModel()

    # States
    px = ca.MX.sym('px')
    py = ca.MX.sym('py')
    psi = ca.MX.sym('psi')
    v = ca.MX.sym('v')
    omega = ca.MX.sym('omega')
    states = ca.vertcat(px, py, psi, v, omega)

    # Inputs
    v_target = ca.MX.sym('v_target')
    omega_target = ca.MX.sym('omega_target')
    controls = ca.vertcat(v_target, omega_target)

    # Model
    model.f_expl_expr = ca.vertcat(
        v * ca.MX.cos(psi),
        v * ca.MX.sin(psi),
        omega,
        (v_target - v) / time_constant_v,
        (omega_target - omega) / time_constant_yawrate,
    )
    model.x = states
    model.u = controls
    model.name = 'agilex'
    return model