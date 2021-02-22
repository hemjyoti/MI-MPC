from mi_nmpc.generate_robot_code import *
import pygame
import numpy as np
import utils
from scipy.spatial import distance
from scipy.signal import butter, lfilter


def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=2):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Initialize the library for Joystick control
pygame.init()
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
# Initialize the joysticks
pygame.joystick.init()
# Get the number of Joystick connected
joystick_count = pygame.joystick.get_count()

# --------------------------------------
# tube's prediction horizon
# --------------------------------------
Nt = 10
# --------------------------------------
# pilot skill level
# (noob = 0, pro = 2)
# -------------------------------------
LEVEL = 0
# --------------------------------------
# filter requirements
# --------------------------------------
fs = 66.67      # sample rate, Hz
order = 2       # filter order
# desired cutoff frequency
if LEVEL == 0:
    cutoff = 28
if LEVEL == 2:
    cutoff = 31
# --------------------------------------
# robot's parameters
# --------------------------------------
g0 = 9.8066  # [m.s^2]
mq = 1.04  # [kg] -- for Gazebo simulation
Ct = 5.95e+2  # [N/kHz^2] -- real experiments
hov_w = np.sqrt((mq * g0) / (4 * Ct))
max_mspeed = 0.09  # [kHz]
# --------------------------------------
# references
# --------------------------------------
robot_yref = utils.parse("trajectory/helix_traj.txt")
robot_yref0 = robot_yref[0, :]

robot_nmpc = robot_nmpc(robot_yref0)
robot_integrator = robot_integrator()

nx = robot_nmpc.acados_ocp.dims.nx
nu = robot_nmpc.acados_ocp.dims.nu
N = robot_nmpc.acados_ocp.dims.N
# --------------------------------------
# weight matrices
# --------------------------------------
Wf = np.eye(len(robot_nmpc.acados_ocp.cost.W))
Wf_e = np.eye(len(robot_nmpc.acados_ocp.cost.W_e))
# --------------------------------------
# lambda
# --------------------------------------
lambda_factor = np.ndarray((1,))
# --------------------------------------
# parameters for simulation
# --------------------------------------
Tf = 9
Ts = 0.015
Nsim = int(Tf / Ts)
t = np.linspace(0.0, Tf, Nsim)

simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))
simH = np.ndarray((Nsim, 3))
simXpredicted = np.ndarray((Nsim, N, nx))

xcurrent = robot_yref0[:nx]
simX[0, :] = xcurrent
simU[0, :] = np.array([0, 0, 0, 0])

yref = np.ndarray((nx + nu))
maximum_error = distance.euclidean([0.1, 0.1, 0.1], [0, 0, 0])
horizon_euclidean_distance = np.ndarray((Nsim, Nt, 1))
lambdas = np.ndarray((Nsim,))
lf_buffer = np.zeros(order,)

mu = 0.99
t1 = mu*maximum_error
t2 = 50
t3 = 0.3*maximum_error
xi = 1e-13

# --------------------------------------
# for open-loop trajectories
# --------------------------------------
simX_robot = np.ndarray((Nsim, N, nx))
simU_robot = np.ndarray((Nsim, N, nu))
# --------------------------------------
# artificial human (nhfc)
# --------------------------------------
if LEVEL == 0:
    replay_hinputs = utils.parse("output/new_hi/xn.txt")
    lvl_state = "/xn_" + str(Nt) + ".txt"
    flambda = "results/xn_lambda.txt"
    os.makedirs(os.path.dirname(flambda), exist_ok=True)
    flog_lambda = open(flambda, "w")
if LEVEL == 2:
    replay_hinputs = utils.parse("output/new_hi/xe.txt")
    lvl_state = "/xe_" + str(Nt) + ".txt"

# --------------------------------------
# creating and storing the results
# --------------------------------------
filename = "results/horizons" + lvl_state
os.makedirs(os.path.dirname(filename), exist_ok=True)
flog = open(filename, "w")
print("Successfully created the directory %s" % filename)
# --------------------------------------
# Simulation loop
# --------------------------------------
for i in range(Nsim):
    for event in pygame.event.get():
        uy=4

    # --------------------------------------
    # update initial condition
    # --------------------------------------
    robot_nmpc.set(0, "lbx", xcurrent)
    robot_nmpc.set(0, "ubx", xcurrent)

    # --------------------------------------
    # solve
    # --------------------------------------
    status = robot_nmpc.solve()

    # --------------------------------------
    # get CPU time
    # --------------------------------------
    cputime_robot = robot_nmpc.get_stats("time_tot")

    # --------------------------------------
    # get open-loop state/control
    # trajectories
    # --------------------------------------
    for j in range(N):
        simX_robot[i, j, :] = robot_nmpc.get(j, "x")
        simU_robot[i, j, :] = robot_nmpc.get(j, "u")

    # --------------------------------------
    # calculate euclidean distance
    # within horizon
    # --------------------------------------
    for k in range(Nt):
        pos_d = robot_yref[k + i, :3]
        pos = simX_robot[i, k, :3]
        horizon_euclidean_distance[i, k, :] = distance.euclidean(pos_d, pos)

    # --------------------------------------
    # get maximum error among all
    # open-loop trajectories
    # --------------------------------------
    box = np.clip(horizon_euclidean_distance[i, :, :], 0, maximum_error)
    d_max = np.amax(box)
    # --------------------------------------
    # calculate correction factor
    # for NMPC cost function
    # --------------------------------------
    # linear function
    # lambda_factor = (maximum_error - mask) / maximum_error
    # decay function
    if i < 5:
        # forcing lambda = 0.5 for the first iterations to reduce
        # the initial chattering and wait until the 2nd-order filter converges
        lambda_factor = 0.5
    else:
        d_sat = t1*(1+xi*np.exp(-t2*(d_max-t3)))**(-1/xi)
        exp_lambda = np.sqrt(maximum_error ** 2 - d_sat ** 2) / maximum_error
        lf = np.clip(exp_lambda, 1e-3, 1)
        lf_buffer = np.insert(lf_buffer[0:lf_buffer.size-1], 0, exp_lambda)
        y = butter_lowpass_filter(lf_buffer, cutoff, fs, order)
        lambda_factor = y[0]

    # --------------------------------------
    # update weight matrices
    # --------------------------------------
    # W_p (position from the aut. motion generator)
    for p in range(3):
        Wf[p, p] = (1 - lambda_factor) * robot_nmpc.acados_ocp.cost.W[p][p]
        Wf_e[p][p] = Wf[p, p]
    # W_h(gamma) (att. human inputs)
    for hg in range(3, 5):
        Wf[hg, hg] = lambda_factor * robot_nmpc.acados_ocp.cost.W[hg][hg]
        Wf_e[hg][hg] = Wf[hg, hg]
    # W_m (yaw, linear velocities and angular velocities from the aut. motion generator)
    for m in range(5, 12):
        Wf[m, m] = (1 - lambda_factor) * robot_nmpc.acados_ocp.cost.W[m][m]
        Wf_e[m][m] = Wf[m, m]
    # W_h(Omega) (propeller speed human inputs)
    for ho in range(12, 16):
        Wf[ho, ho] = lambda_factor * robot_nmpc.acados_ocp.cost.W[ho][ho]
        Wf_e[ho][ho] = Wf[ho, ho]
    # control inputs
    for o in range(16, 20):
        Wf[o, o] = (1 - lambda_factor) * robot_nmpc.acados_ocp.cost.W[o][o]
    # set new weights
    for j in range(N):
        robot_nmpc.cost_set(j, "W", Wf)
    robot_nmpc.cost_set(N, "W", Wf_e)

    # --------------------------------------
    # log lambdas
    # --------------------------------------
    lambdas[i] = lambda_factor

    # --------------------------------------
    # get solution
    # --------------------------------------
    simU[i, :] = robot_nmpc.get(0, "u")


    # Get the value of the 3rd Joystick Axis
    #Initializa First Joystcik whose ID is 0.
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    axis_test = joystick.get_axis(3)
    # print(simU[i, :])
    joystick_count = pygame.joystick.get_count()
    axes = joystick.get_numaxes()

    # --------------------------------------
    # prepare human inputs
    # --------------------------------------
    # h_att = replay_hinputs[i, :2]
    # h_spe = np.sqrt(replay_hinputs[i, 2] / (4 * Ct))
    h_att_joystick_phi=axis_test*0.08
    h_att = replay_hinputs[i, 4:5]
    h_spe = replay_hinputs[i, 12:16]


    #print(axes)
    print(axis_test)
    #print(' \n ')

    # --------------------------------------
    # update robot's reference
    # --------------------------------------
    for j in range(N):
        yref = np.concatenate((robot_yref[j + i, :3],[h_att_joystick_phi], h_att, robot_yref[j + i, 5:12],
                               h_spe, [0.0, 0.0, 0.0, 0.0]))
        robot_nmpc.set(j, "yref", yref)
    yref_e = yref[:nx]
    robot_nmpc.set(N, "yref", yref_e)


    # log human inputs
    h_thr = Ct * (h_spe[0] ** 2 + h_spe[1] ** 2 + h_spe[2] ** 2 + h_spe[3] ** 2)
    simH[i, :] = np.concatenate(([h_att_joystick_phi],h_att, [h_thr]))

    # --------------------------------------
    # simulate system
    # --------------------------------------
    robot_integrator.set("x", xcurrent)
    robot_integrator.set("u", simU[i, :])

    status = robot_integrator.solve()

    # --------------------------------------
    # log
    # --------------------------------------
    if i == 0:
        states = robot_yref0[0:3].reshape(1, 3)
        att = robot_yref0[3:5].reshape(1, 2)
        speed = robot_yref0[12:16].reshape(1, 4)
        data = np.column_stack([states, att, speed, cputime_robot])
        np.savetxt(flog, data, fmt='%4f', delimiter=" ", newline="\n")
        if LEVEL == 0:
            np.savetxt(flog_lambda, [lambdas[i]], fmt='%4f', delimiter=" ", newline="\n")
    else:
        states = simX[i, 0:3].reshape(1, 3)
        att = simX[i, 3:5].reshape(1, 2)
        speed = simX[i, 12:16].reshape(1, 4)
        data = np.column_stack([states, att, speed, cputime_robot])
        np.savetxt(flog, data, fmt='%4f', delimiter=" ", newline="\n")
        if LEVEL == 0:
            np.savetxt(flog_lambda, [lambdas[i]], fmt='%4f', delimiter=" ", newline="\n")

    # --------------------------------------
    # update state
    # --------------------------------------
    xcurrent = robot_integrator.get("x")
    simX[i + 1, :] = xcurrent


    #print(simX[i,:])
   #print(xcurrent)

# --------------------------------------
# plot results
# --------------------------------------
if LEVEL == 0:
    # utils.plot_ph_index('novice', Tf, Nsim, lambdas)
    utils.plot_bounded_var('novice', Tf, Nsim, simX)
    utils.plot_pilots_ph_index(Tf,Nsim,lambdas)
   # utils.plot_artificial_commands('novice', Tf, Nsim, simX, simH)
if LEVEL == 2:
    utils.plot_pilots_ph_index(Tf, Nsim, lambdas)
    utils.plot_bounded_var('experienced', Tf, Nsim, simX)
    utils.plot_artificial_commands('experienced', Tf, Nsim, simX, simH)
    # utils.plot_trajectory_error('mi_tube/horizons/xn_10.txt', Tf, Nsim, robot_yref, simX, maximum_error)
    utils.plot_novice_expert('results/', 'results/horizons/xn_10.txt', Nsim, robot_yref, simX, maximum_error)

# Quit the joystick library
pygame.quit()
