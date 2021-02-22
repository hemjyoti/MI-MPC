from artificial_human.export_robot_model import *
import numpy as np
import scipy.linalg
from typing import Sequence
import utils


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
    # nprm = model.p.size()[0]

    # set dimensions
    ocp.dims.nx = nx
    ocp.dims.ny = ny
    ocp.dims.ny_e = ny_e
    ocp.dims.nbx = 0
    ocp.dims.nbu = nu
    ocp.dims.nbx_e = 0
    ocp.dims.nu = model.u.size()[0]
    ocp.dims.N = N
    # ocp.dims.np = nprm
    # ocp.dims.ns_e = nprm
    # ocp.dims.nh_e = nprm
    # ocp.dims.nsh_e = nprm

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
    # novice
    Q[0, 0] = 8.0  # x
    Q[1, 1] = 8.0  # y
    Q[2, 2] = 1.0  # z
    Q[3, 3] = 1e-3  # phi
    Q[4, 4] = 1e-3  # theta
    Q[5, 5] = 5  # psi
    Q[6, 6] = 3.e-1  # vbx
    Q[7, 7] = 3.e-1  # vby
    Q[8, 8] = 4.e-1  # vbz
    Q[9, 9] = 5  # wx
    Q[10, 10] = 1.e-3  # wy
    Q[11, 11] = 1.e-3  # wz
    Q[12, 12] = 1.e-3  # w1
    Q[13, 13] = 1.e-3  # w2
    Q[14, 14] = 1.e-3  # w3
    Q[15, 15] = 1.e-3  # w4

    # experienced
    # Q[0, 0] = 10.0  # x
    # Q[1, 1] = 10.0  # y
    # Q[2, 2] = 10.0  # z
    # Q[3, 3] = 1.0e-1  # phi human
    # Q[4, 4] = 1.0e-1  # theta human
    # Q[5, 5] = 5  # psi
    # Q[6, 6] = 1  # vbx
    # Q[7, 7] = 1  # vby
    # Q[8, 8] = 1.0  # vbz
    # Q[9, 9] = 1.0  # wx
    # Q[10, 10] = 1.0  # wy
    # Q[11, 11] = 5.0  # wz
    # Q[12, 12] = 10.0  # w1 human
    # Q[13, 13] = 10.0  # w2 human
    # Q[14, 14] = 10.0  # w3 human
    # Q[15, 15] = 10.0  # w4 human

    R = np.eye(nu)
    R[0, 0] = 50  # tau1
    R[1, 1] = 50  # tau2
    R[2, 2] = 50  # tau3
    R[3, 3] = 50  # tau4

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[16, 0] = 1.0
    Vu[17, 1] = 1.0
    Vu[18, 2] = 1.0
    Vu[19, 3] = 1.0
    ocp.cost.Vu = Vu

    # slack variables cost term
    # ocp.cost.zl_e = 5e+3 * np.ones((nprm,))  # gradient wrt lower slack
    # ocp.cost.Zl_e = np.zeros((nprm,))  # Hessian wrt lower slack
    # ocp.cost.zu_e = 5e+3 * np.ones((nprm,))  # gradient wrt upper slack
    # ocp.cost.Zu_e = np.zeros((nprm,))  # Hessian wrt upper slack

    # reference
    ocp.cost.yref = yref
    ocp.cost.yref_e = yref[:ny_e]

    # constraints
    ocp.constraints.lbu = np.array([-max_torque, -max_torque, -max_torque, -max_torque])
    ocp.constraints.ubu = np.array([+max_torque, +max_torque, +max_torque, +max_torque])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    # ocp.constraints.lbx = np.array([0, 0, 0, 0])
    # ocp.constraints.ubx = np.array([+max_mspeed, +max_mspeed, +max_mspeed, +max_mspeed])
    # ocp.constraints.idxbx = np.array([12, 13, 14, 15])
    # # for the nonlinear constraints
    # ocp.constraints.lh_e = np.zeros((nprm,))
    # ocp.constraints.uh_e = 10e+3 * np.ones((nprm,))
    # # for the soft nonlinear constraints
    # ocp.constraints.lsh_e = np.zeros((nprm,))  # lower bound
    # ocp.constraints.ush_e = 10e+3 * np.ones((nprm,))  # upper bound
    # ocp.constraints.idxsh_e = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])  # indexes of bounds

    # initial state
    ocp.constraints.x0 = yref[0:nx]

    # set parameters
    # ocp.parameter_values = yref[0:nx]

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


# --------------------------------------
# references
# --------------------------------------
robot_yref = utils.parse("../trajectory/helix_traj.txt")
robot_yref0 = robot_yref[0, :]

robot_nmpc = robot_nmpc(robot_yref0)
robot_integrator = robot_integrator()

nx = robot_nmpc.acados_ocp.dims.nx
nu = robot_nmpc.acados_ocp.dims.nu

# --------------------------------------
# parameters for simulation
# --------------------------------------
Tf = 9
Ts = 0.015
Nsim = int(Tf / Ts)
t = np.linspace(0.0, Tf, Nsim)

simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))

xcurrent = robot_yref0[:robot_nmpc.acados_ocp.dims.nx]
simX[0, :] = xcurrent
simU[0, :] = np.array([0, 0, 0, 0])

# --------------------------------------
# robot's parameters
# --------------------------------------
g0 = 9.8066  # [m.s^2]
mq = 1.04  # [kg] -- for Gazebo simulation
Ct = 5.95e+2  # [N/kHz^2] -- real experiments
hov_w = np.sqrt((mq * g0) / (4 * Ct))
f_hov = Ct*(4*hov_w**2)

fstates = open("../output/new_hi/xn.txt", "w")

# --------------------------------------
# artificial human (FAC)
# --------------------------------------

for i in range(Nsim):

    robot_nmpc.set(0, "lbx", xcurrent)
    robot_nmpc.set(0, "ubx", xcurrent)

    status = robot_nmpc.solve()

    # get solution
    simU[i, :] = robot_nmpc.get(0, "u")

    # update robot's reference
    for j in range(robot_nmpc.acados_ocp.dims.N):
        yref = robot_yref[j + i, :]
        robot_nmpc.set(j, "yref", yref)
    yref_e = robot_yref[robot_nmpc.acados_ocp.dims.N + i, :nx]
    robot_nmpc.set(robot_nmpc.acados_ocp.dims.N, "yref", yref_e)

    # simulate system
    robot_integrator.set("x", xcurrent)
    robot_integrator.set("u", simU[i, :])

    # simulate human inputs
    status = robot_integrator.solve()

    # update state
    xcurrent = robot_integrator.get("x")
    simX[i + 1, :] = xcurrent

    if i == 0:
        states = robot_yref0[:nx].reshape(1, nx)
        np.savetxt(fstates, states, fmt='%4f', delimiter=" ", newline="\n")
    else:
        states = simX[i, :nx].reshape(1, nx)
        np.savetxt(fstates, states, fmt='%4f', delimiter=" ", newline="\n")

## plot results
import matplotlib.pyplot as plt
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

fig = plt.figure()
line_labels = [r'reference', r'fully autonomous NMPC']
ax2 = fig.add_subplot(111, projection='3d')
ax2.plot3D(robot_yref[0:Nsim, 0], robot_yref[0:Nsim, 1], robot_yref[0:Nsim, 2], color='k', linewidth=1.5,
           linestyle='--')
ax2.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color='r', linewidth=2.0)
ax2.plot3D([robot_yref[Nsim, 0]], [robot_yref[Nsim, 1]], [robot_yref[Nsim, 2]], '*', color='k', markersize=10.0)
ax2.view_init(63, 33)
# ax2.set_zticklabels([])
# ax2.set_yticklabels([])
# ax2.set_xticklabels([])
ax2.grid(True, linewidth=0.5)
plt.savefig('../mi_tube/novice' + '.pdf', dpi=400, bbox_inches="tight")

Ct = 5.95e+2  # [N/kHz^2] -- real experiments
thr_robot = Ct * (simX[:Nsim, 12] ** 2 + simX[:Nsim, 13] ** 2 + simX[:Nsim, 14] ** 2 + simX[:Nsim, 15] ** 2)

fig, (phi, the, Th) = plt.subplots(3)
fig.suptitle('Human inputs')
phi.plot(t, simX[:Nsim, 3] * 180/np.pi, 'k')
the.plot(t, simX[:Nsim, 4] * 180/np.pi, 'k')
Th.plot(t, thr_robot, 'k')

plt.show()
