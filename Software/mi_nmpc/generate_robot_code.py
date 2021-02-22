from mi_nmpc.export_robot_model import *
import numpy as np
import scipy.linalg
from typing import Sequence


def robot_nmpc(yref: Sequence[float]):
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # export model
    model = export_ode_model()
    ocp.model = model

    Tf = 0.75
    N = 50
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    # set dimensions
    ocp.dims.nx = nx
    ocp.dims.ny = ny
    ocp.dims.ny_e = ny_e
    ocp.dims.nbx = 4
    ocp.dims.nbu = nu
    ocp.dims.nbx_e = 0
    ocp.dims.nu = model.u.size()[0]
    ocp.dims.N = N

    # parameters
    g0 = 9.8066  # [m.s^2]
    mq = 1.04  # [kg] for Gazebo simulation
    Ct = 5.95e+2  # [N/kHz^2] -- For real experiments
    Cd = 1.0e+1  # [Nm/kHz^2]

    # bounds
    hov_w = np.sqrt((mq * g0) / (4 * Ct))  # [kHz]
    max_mspeed = 0.09  # [kHz]
    max_torque = 3 * Cd * hov_w ** 2  # [Nm]

    # set weighting matrices
    Q = np.eye(nx)
    # robot
    Q[0, 0] = 250.0  # x
    Q[1, 1] = 250.0  # y
    Q[2, 2] = 300.0  # z
    Q[3, 3] = 80 * 50.0  # phi human
    Q[4, 4] = 80 * 50.0  # theta human
    Q[5, 5] = 10.0  # psi
    Q[6, 6] = 3.0  # vbx
    Q[7, 7] = 3.0  # vby
    Q[8, 8] = 3.0  # vbz
    Q[9, 9] = 3.0  # wx
    Q[10, 10] = 3.0  # wy
    Q[11, 11] = 10.0  # wz
    Q[12, 12] = 80 * 50.0  # w1 human
    Q[13, 13] = 80 * 50.0  # w2 human
    Q[14, 14] = 80 * 50.0  # w3 human
    Q[15, 15] = 80 * 50.0  # w4 human

    # torques
    R = np.eye(nu)
    R[0, 0] = 70  # tau1
    R[1, 1] = 70  # tau2
    R[2, 2] = 70  # tau3
    R[3, 3] = 70  # tau4

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[16, 0] = 1.0
    Vu[17, 1] = 1.0
    Vu[18, 2] = 1.0
    Vu[19, 3] = 1.0
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    # reference
    ocp.cost.yref = yref
    ocp.cost.yref_e = yref[:ny_e]

    # constraints
    ocp.constraints.lbu = np.array([-max_torque, -max_torque, -max_torque, -max_torque])
    ocp.constraints.ubu = np.array([+max_torque, +max_torque, +max_torque, +max_torque])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbx = np.array([0, 0, 0, 0])
    ocp.constraints.ubx = np.array([+max_mspeed, +max_mspeed, +max_mspeed, +max_mspeed])
    ocp.constraints.idxbx = np.array([12, 13, 14, 15])

    # initial state
    ocp.constraints.x0 = yref[:nx]

    # set QP solver
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 0

    # set prediction horizon
    ocp.solver_options.tf = Tf
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_' + model.name + '.json')

    print('>> robot NMPC exported')

    return ocp_solver


def robot_integrator():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # export model
    model = export_ode_model()
    ocp.model = model

    # set dimensions
    ocp.dims.nx = model.x.size()[0]
    ocp.dims.nu = model.u.size()[0]

    integrator = AcadosSimSolver(ocp, json_file='acados_ocp_' + model.name + '.json')

    print('>> robot integrator exported')

    return integrator
