from acados_template import *
from casadi import SX, vertcat, sin, cos


def export_ode_model():
    model_name = 'robot'

    # parameters
    g0 = 9.8066  # [m.s^2]
    mq = 1.04    # [kg] for Gazebo simulation
    Ixx = 0.010  # [kg.m^2]
    Iyy = 0.010  # [kg.m^2]
    Izz = 0.070  # [kg.m^2]
    Cd = 1.0e+1  # [Nm/kHz^2]
    Ct = 5.95e+2  # [N/kHz^2] -- real experiments
    l = 0.23  # [m] distance between motor center and the axis of rotation
    Im = 0.08  # [kg.m^2] rotor
    b = 5e-4  # considering 0.5 ms for the mechanical constant

    # states (f_exp)
    # states for the robot's cost function
    xq = SX.sym('xq')
    yq = SX.sym('yq')
    zq = SX.sym('zq')
    phi = SX.sym('phi')
    theta = SX.sym('theta')
    psi = SX.sym('psi')
    vbx = SX.sym('vbx')
    vby = SX.sym('vby')
    vbz = SX.sym('vbz')
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    w1 = SX.sym('w1')
    w2 = SX.sym('w2')
    w3 = SX.sym('w3')
    w4 = SX.sym('w4')
    x = vertcat(xq, yq, zq, phi, theta, psi, vbx, vby, vbz, wx, wy, wz, w1, w2, w3, w4)

    # controls
    t1 = SX.sym('t1')
    t2 = SX.sym('t2')
    t3 = SX.sym('t3')
    t4 = SX.sym('t4')
    u = vertcat(t1, t2, t3, t4)  # motor torques

    # for f_impl
    # for the robot
    xq_dot = SX.sym('xq_dot')
    yq_dot = SX.sym('yq_dot')
    zq_dot = SX.sym('zq_dot')
    phi_dot = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    psi_dot = SX.sym('psi_dot')
    vbx_dot = SX.sym('vbx_dot')
    vby_dot = SX.sym('vby_dot')
    vbz_dot = SX.sym('vbz_dot')
    wx_dot = SX.sym('wx_dot')
    wy_dot = SX.sym('wy_dot')
    wz_dot = SX.sym('wz_dot')
    w1_dot = SX.sym('w1_dot')
    w2_dot = SX.sym('w2_dot')
    w3_dot = SX.sym('w3_dot')
    w4_dot = SX.sym('w4_dot')

    xdot = vertcat(xq_dot, yq_dot, zq_dot, phi_dot, theta_dot, psi_dot, vbx_dot, vby_dot, vbz_dot, wx_dot, wy_dot,
                   wz_dot, w1_dot, w2_dot, w3_dot, w4_dot)

    # Model equations
    dxq = vbx * cos(psi) * cos(theta) - vbz * sin(theta) + vby * cos(theta) * sin(psi)
    dyq = vby * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) - vbx * (
                cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) + vbz * cos(theta) * sin(phi)
    dzq = vbx * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) - vby * (
                cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) + vbz * cos(phi) * cos(theta)
    dphi = wz / cos(theta)
    dtheta = wy / cos(psi) - (wz * sin(psi) * sin(theta)) / (cos(psi) * cos(theta))
    dpsi = wx + (wy * sin(psi)) / cos(psi) - (wz * sin(theta)) / (cos(psi) * cos(theta))
    dvbx = vby * wz - vbz * wy - g0 * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta))
    dvby = vbz * wx - vbx * wz + g0 * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta))
    dvbz = vbx * wy - vby * wx + (Ct * (w1 ** 2 + w2 ** 2 + w3 ** 2 + w4 ** 2)) / mq - g0 * cos(phi) * cos(theta)
    dwx = (Iyy*wy*wz - Izz*wy*wz + Ct*l*(w2 ** 2 - w4 ** 2)) / Ixx
    dwy = -(Ixx*wx*wz - Izz*wx*wz + Ct*l*(w1 ** 2 - w3 ** 2)) / Iyy
    dwz = (Cd*(w1 ** 2 - w2 ** 2 + w3 ** 2 - w4 ** 2) + Ixx*wx*wy - Iyy*wx*wy) / Izz
    dw1 = (t1 - Cd * w1**2 - b * w1) / Im
    dw2 = (t2 - Cd * w2**2 - b * w2) / Im
    dw3 = (t3 - Cd * w3**2 - b * w3) / Im
    dw4 = (t4 - Cd * w4**2 - b * w4) / Im

    # Explicit and Implicit functions
    f_expl = vertcat(dxq, dyq, dzq, dphi, dtheta, dpsi, dvbx, dvby, dvbz, dwx, dwy, dwz, dw1, dw2, dw3, dw4)
    f_impl = xdot - f_expl

    # algebraic variables
    z = []

    # # parameters
    p = []

    # dynamics
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.name = model_name
    model.p = p

    # terminal constraint
    # xr = SX.sym('xr', 16)
    # model.con_h_expr_e = model.x - xr
    # model.p = xr

    return model
