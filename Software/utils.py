import numpy as np
from typing import Sequence
import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d.axes3d import get_test_data
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.axes_grid1.inset_locator import (inset_axes, InsetPosition, mark_inset)
from mpl_toolkits.axes_grid1.inset_locator import TransformedBbox, BboxPatch, BboxConnector
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import LineCollection
from scipy.spatial import distance
from cycler import cycler
import os
from matplotlib.collections import PolyCollection
from matplotlib import colors as mcolors


#color pallete
azul = [0, 0.3, 0.7]
bebe = [0.5, 0.6, 0.9]
rosa = [0.9, 0.5, 0.8]
laranja = [1, 0.6, 0.4]
verde = [0.4, 0.8, 0.7]
vermelho = [0.8, 0.1, 0.1]
cinza = [0.8, 0.8, 0.9]

# file data to numpy array
def parse(filename):
    lines = np.loadtxt(filename)
    return lines


def cc(arg):
    return mcolors.to_rgba(arg, alpha=0.6)


def polygon_under_graph(xlist, ylist):
    """
    Construct the vertex list which defines the polygon filling the space under
    the (xlist, ylist) line graph.  Assumes the xs are in ascending order.
    """
    return [(xlist[0], 0.), *zip(xlist, ylist), (xlist[-1], 0.)]


# input: np.array([roll, pitch, yaw]) and np.array([wx, wy, wz])
# output: np.array([dr, dp, dy])
def e_rates(ea: Sequence[float], w: Sequence[float]):
    """
    Computes de euler rates
    :param ea: Euler angles.
    :param w: Angular rates.
    :return: The euler rates.
    """

    phi = ea[0]
    the = ea[1]
    psi = ea[2]

    # This is the matrix correspondent to the
    # euler rates conversion from
    # BODY angular rates to INERTIAL angular rates
    T11 = 0
    T12 = 0
    T13 = 1 / np.cos(the)
    T21 = 0
    T22 = 1 / np.cos(psi)
    T23 = -(np.sin(psi) * np.sin(the)) / (np.cos(psi) * np.cos(the))
    T31 = 1
    T32 = np.sin(psi) / np.cos(psi)
    T33 = -np.sin(the) / (np.cos(psi) * np.cos(the))

    T = np.array([T11, T12, T13,
                  T21, T22, T23,
                  T31, T32, T33]).reshape(3, 3)

    e_rates = np.dot(T, w)

    return e_rates


# input: np.array([roll, pitch, yaw]) and np.array([wx, wy, wz])
# output: np.array([wx, wy, wz])
def body_rates(ea: Sequence[float], dea: Sequence[float]):
    """
    Computes de body rates
    :param ea: Euler angles.
    :param dea: Euler rates.
    :return: The body rates.
    """

    phi = ea[0]
    the = ea[1]
    psi = ea[2]

    # This is the matrix correspondent to the
    # euler rates conversion from
    # BODY angular rates to INERTIAL angular rates
    T11 = 0
    T12 = 0
    T13 = 1 / np.cos(the)
    T21 = 0
    T22 = 1 / np.cos(psi)
    T23 = -(np.sin(psi) * np.sin(the)) / (np.cos(psi) * np.cos(the))
    T31 = 1
    T32 = np.sin(psi) / np.cos(psi)
    T33 = -np.sin(the) / (np.cos(psi) * np.cos(the))

    T = np.array([T11, T21, T31,
                  T12, T22, T32,
                  T13, T23, T33]).reshape(3, 3)

    body_rates = np.dot(T, dea)

    return body_rates


# input: np.array([qw, qx, qy, qz]) and np.array([vix, viy, viz])
# output: np.array([vbx, vby, vbz])
def I2B(q: Sequence[float], vec: Sequence[float]):
    """
    This is the conversion between quaternion orientation to rotation matrix
    from EARTH to BODY (considering ZYX euler sequence)
    :param q: Attitude quaternion.
    :param vec: 3D vector.
    :return: Rotated vector.
    """

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    S11 = 2 * (qw * qw + qx * qx) - 1
    S12 = 2 * (qx * qy + qw * qz)
    S13 = 2 * (qx * qz - qw * qy)
    S21 = 2 * (qx * qy - qw * qz)
    S22 = 2 * (qw * qw + qy * qy) - 1
    S23 = 2 * (qy * qz + qw * qx)
    S31 = 2 * (qx * qz + qw * qy)
    S32 = 2 * (qy * qz - qw * qx)
    S33 = 2 * (qw * qw + qz * qz) - 1

    Sq = np.array([S11, S12, S13,
                   S21, S22, S23,
                   S31, S32, S33]).reshape(3, 3)

    vb = np.dot(Sq, vec)

    return vb


# input: np.array([qw, qx, qy, qz]) and np.array([vbx, vby, vbz])
# output: np.array([vix, viy, viz])
def B2I(q: Sequence[float], vec: Sequence[float]):
    """
    This is the conversion between quaternion orientation to rotation matrix
    from BODY to EARTH (considering ZYX euler sequence)
    :param q: Attitude quaternion.
    :param vec: 3D vector.
    :return: Rotated vector.
    """

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    S11 = 2 * (qw * qw + qx * qx) - 1
    S12 = 2 * (qx * qy + qw * qz)
    S13 = 2 * (qx * qz - qw * qy)
    S21 = 2 * (qx * qy - qw * qz)
    S22 = 2 * (qw * qw + qy * qy) - 1
    S23 = 2 * (qy * qz + qw * qx)
    S31 = 2 * (qx * qz + qw * qy)
    S32 = 2 * (qy * qz - qw * qx)
    S33 = 2 * (qw * qw + qz * qz) - 1

    Sq = np.array([S11, S12, S13,
                   S21, S22, S23,
                   S31, S32, S33]).reshape(3, 3)

    vi = np.dot(Sq.transpose(), vec)

    return vi


# input: roll, pitch, yaw or np.array([roll, pitch, yaw])
# output: np.array([qw, qx, qy, qz])
def euler2quatern(e: Sequence[float]):
    """
    Retrieve the quaternions q given the euler angles ea
    :param e: Attitude quaternion.
    :return: Quaternions.
    """

    e = np.asarray(e[0]) if len(e) == 1 else np.asarray(e)
    cr = np.cos(e[0] * 0.5)
    sr = np.sin(e[0] * 0.5)
    cp = np.cos(e[1] * 0.5)
    sp = np.sin(e[1] * 0.5)
    cy = np.cos(e[2] * 0.5)
    sy = np.sin(e[2] * 0.5)

    q = np.zeros((4,))
    q[0] = cr * cp * cy + sr * sp * sy
    q[1] = -(cy * cp * sr - sy * sp * cr)
    q[2] = -(cy * sp * cr + sy * cp * sr)
    q[3] = -(sy * cp * cr - cy * sp * sr)

    if q[0] < 0:
        q[0] = -q[0]
        q[1] = -q[1]
        q[2] = -q[2]
        q[3] = -q[3]

    return q


# input: np.array([qw, qx, qy, qz])
# output: np.array([roll, pitch, yaw])
def quatern2euler(q):
    """
    Retrieve the euler angles ea given the quaternions q
    :param q: Attitude quaternion.
    :return: Euler angles.
    """

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    R11 = 2 * (qw * qw + qx * qx) - 1
    R21 = 2 * (qx * qy - qw * qz)
    R31 = 2 * (qx * qz + qw * qy)
    R32 = 2 * (qy * qz - qw * qx)
    R33 = 2 * (qw * qw + qz * qz) - 1

    phi = np.arctan2(R32, R33)
    theta = -np.arcsin(R31)
    psi = np.arctan2(R21, R11)

    return np.array((phi, theta, psi))


def quat2mat(q):
    """
    Constructs a rotation matrix R given the quaternions q
    :param q: Attitude quaternion.
    :return: The quaternion rotation matrix R.
    """
    w, x, y, z = q

    q1q2 = x * y
    q0q3 = w * z
    q1q3 = x * z
    q0q2 = w * y
    q2q3 = y * z
    q0q1 = w * x

    a11 = 2 * (w * w + x * x) - 1
    a12 = 2 * (q1q2 + q0q3)
    a13 = 2 * (q1q3 - q0q2)
    a21 = 2 * (q1q2 - q0q3)
    a22 = 2 * (w * w + y * y) - 1
    a23 = 2 * (q2q3 + q0q1)
    a31 = 2 * (q1q3 + q0q2)
    a32 = 2 * (q2q3 - q0q1)
    a33 = 2 * (w * w + z * z) - 1

    return np.array(
        [[a11, a12, a13],
         [a21, a22, a23],
         [a31, a32, a33]])


def mark_inset(parent_axes, inset_axes, loc1a=1, loc1b=1, loc2a=2, loc2b=2, **kwargs):
    rect = TransformedBbox(inset_axes.viewLim, parent_axes.transData)

    pp = BboxPatch(rect, fill=False, **kwargs)
    parent_axes.add_patch(pp)

    p1 = BboxConnector(inset_axes.bbox, rect, loc1=loc1a, loc2=loc1b, **kwargs)
    inset_axes.add_patch(p1)
    p1.set_clip_on(False)
    p2 = BboxConnector(inset_axes.bbox, rect, loc1=loc2a, loc2=loc2b, **kwargs)
    inset_axes.add_patch(p2)
    p2.set_clip_on(False)

    return pp, p1, p2


def plot_workspace(directory, pilot, Tf, N, i, robot_yref, simX: np.ndarray,
                            lambdas: np.ndarray, xn: np.ndarray, threshold):
    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    N_acados = 50
    dt = Tf / N
    inset_idx = 20
    horizon_idx = N_acados - lambdas * N_acados
    vermelho = [0.815, 0.122, 0.188]

    # ------- Font size ------- #
    tck = 18
    lab = 25
    mks = 7.0
    # ------------------------- #

    fig = plt.figure(figsize=(20, 10))

    # ===========================
    #  lambda
    # ===========================
    ax1 = fig.add_subplot(221)
    ax1.plot(t, lambdas, color=vermelho)
    ax1.grid(b=True, which='major', linestyle='--')
    ax1.set_xlabel(r'$t$ (s)', fontsize=lab)
    ax1.set_ylabel(r'$\lambda$', fontsize=lab+2)
    ax1.xaxis.set_tick_params(labelsize=tck)
    ax1.yaxis.set_tick_params(labelsize=tck)
    ax1.autoscale(enable=True, axis='x', tight=True)

    # ===========================
    #  3D position plot
    # ===========================
    ax2 = fig.add_subplot(223, projection='3d')
    ax2.plot3D([robot_yref[0]], [robot_yref[1]], [robot_yref[2]], '*', color='k', markersize=10.0)
    ax2.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color=vermelho, linewidth=2.0)
    ax2.view_init(27, 122)
    ax2.set_zticklabels([])
    ax2.set_yticklabels([])
    ax2.set_xticklabels([])
    # plot workspace box
    cube_definition = [(-2, -2, 0.5), (-2, 2, 0.5), (2, -2, 0.5), (-2, -2, 3)]
    plot_cube(cube_definition, ax2)

    ax2.axes.set_xlim3d(left=-2.1, right=2.1)
    ax2.axes.set_ylim3d(bottom=-2.1, top=2.1)
    ax2.axes.set_zlim3d(bottom=-2.1, top=2.1)

    # make the panes transparent
    ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # make the grid lines transparent
    ax2.xaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax2.yaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax2.zaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)

    plt.axis('off')

    # ===========================
    #  lambda magnified
    # ===========================
    ax3 = fig.add_subplot(322)
    mark_inset(ax1, ax3, loc1a=3, loc1b=4, loc2a=2, loc2b=1, fc="none", ec='0.5')
    tl = np.linspace(i * dt, (inset_idx + i) * dt, inset_idx)
    ax3.plot(tl, lambdas[i:(i + inset_idx)], color=vermelho, marker='D', markersize=mks)
    ax3.set_ylabel(r'$\lambda$', fontsize=lab+2)
    ax3.xaxis.set_tick_params(labelsize=tck)
    ax3.yaxis.set_tick_params(labelsize=tck)

    # ===========================
    #  maximum FC tube excursion
    # ===========================
    ax4 = fig.add_subplot(324)
    th = np.linspace(i * dt, (inset_idx + i) * dt, inset_idx)
    ax4.plot(th, horizon_idx[i:(inset_idx + i)], color=cinza, linestyle='--')
    for k in range(i, i + inset_idx, 1):
        if horizon_idx[k] == 0.0:
            ax4.plot(th[k-i], horizon_idx[k], color=bebe, linestyle='--', marker='D', markersize=mks)
        if horizon_idx[k] == 50.0:
            ax4.plot(th[k-i], horizon_idx[k], color=verde, linestyle='--', marker='D', markersize=mks)
        if 1.0 <= horizon_idx[k] < 50.0:
            ax4.plot(th[k-i], horizon_idx[k], color=rosa, linestyle='--', marker='D', markersize=mks)
    ax4.set_ylabel(r'$d_{\textrm{sat}}$', fontsize=lab+2)
    ax4.xaxis.set_tick_params(labelsize=tck)
    ax4.yaxis.set_tick_params(labelsize=tck)

    # ===========================
    #  open-loop trajectories
    # ===========================
    ax5 = fig.add_subplot(326)
    for k in range(i, i + inset_idx, 1):
        dt = Tf / N
        ti = np.linspace(k * dt, (N_acados + k) * dt, N_acados)
        lower_bound = 0.0
        upper_bound = threshold
        if horizon_idx[k] == 0.0:
            ax5.plot(ti, xn[k, :, :], color=bebe)
        if horizon_idx[k] == 50.0:
            ax5.plot(ti, xn[k, :, :], color=verde)
        if 1.0 <= horizon_idx[k] < 50.0:
            ax5.plot(ti, xn[k, :, :], color=rosa)
        ax5.plot(ti, upper_bound * np.ones(len(ti)), lw=1, color='black', ls='--')
        ax5.plot(ti, lower_bound * np.ones(len(ti)), lw=1, color='black', ls='--')
    ax5.set_ylabel(r'$\|\eta_i^\star-\eta_i^r\|$ (m)', fontsize=lab+2)
    ax5.set_xlabel(r'$t$ (s)', fontsize=lab)
    ax5.xaxis.set_tick_params(labelsize=tck)
    ax5.yaxis.set_tick_params(labelsize=tck)

    plt.savefig(directory + pilot + '_position' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_ph_index(pilot, Tf, N, lambdas: np.ndarray):
    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    vermelho = [0.815, 0.122, 0.188]
    fig = plt.figure(figsize=(6, 6))

    # ===========================
    #  lambda
    # ===========================
    ax1 = fig.add_subplot(111)
    ax1.plot(t, lambdas, color=vermelho)
    ax1.grid(b=True, which='major', linestyle='--')
    ax1.set_xlabel(r'$t$ (s)', fontsize=22)
    ax1.set_ylabel(r'$\lambda$', fontsize=22)
    ax1.xaxis.set_tick_params(labelsize=13)
    ax1.yaxis.set_tick_params(labelsize=13)
    ax1.autoscale(enable=True, axis='x', tight=True)

    fig.tight_layout()
    plt.savefig('results/' + pilot + '_ph_index' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_pilots_ph_index(Tf, N, lambdas: np.ndarray):
    #rc('text', usetex=True)
    #rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    azul = [0.141, 0.572, 1]
    vermelho = [0.815, 0.122, 0.188]
    fig = plt.figure(figsize=(5, 5))

    novice = parse("results/xn_lambda.txt")
    line_labels = [r'novice', r'experienced']

    # ===========================
    #  lambda
    # ===========================
    ax = fig.add_subplot(111)
    #ax.plot(t, novice, color=azul)
    ax.plot(t, lambdas, color=vermelho)
    ax.autoscale(enable=True, axis='x', tight=True)
    ax.grid(b=True, which='major', linestyle='--')
    #ax.set_xlabel(r'$t$ (s)', fontsize=18)
    #ax.set_ylabel(r'$\lambda$', fontsize=18)
    # ax.xaxis.set_tick_params(labelsize=13)
    # ax.yaxis.set_tick_params(labelsize=13)
    #fig.legend(labels=line_labels, loc='upper center', bbox_to_anchor=(0.544, 1.1), ncol=2, fontsize=18)

    fig.tight_layout()
    plt.savefig('results/ph_index' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_max_tube_excursion(directory, pilot, Tf, N, i, robot_yref: np.ndarray, simX: np.ndarray,
                            lambdas: np.ndarray, xn: np.ndarray, threshold, Nt):
    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    N_acados = Nt
    dt = Tf / N
    inset_idx = 20
    horizon_idx = N_acados - lambdas * N_acados
    vermelho = [0.815, 0.122, 0.188]

    # ------- Font size ------- #
    tck = 18
    lab = 25
    mks = 7.0
    # ------------------------- #

    fig = plt.figure(figsize=(20, 10))

    # ===========================
    #  lambda
    # ===========================
    ax1 = fig.add_subplot(221)
    ax1.plot(t, lambdas, color=vermelho)
    ax1.grid(b=True, which='major', linestyle='--')
    ax1.set_xlabel(r'$t$ (s)', fontsize=lab)
    ax1.set_ylabel(r'$\lambda$', fontsize=lab+2)
    ax1.xaxis.set_tick_params(labelsize=tck)
    ax1.yaxis.set_tick_params(labelsize=tck)
    ax1.autoscale(enable=True, axis='x', tight=True)

    # ===========================
    #  3D position plot
    # ===========================
    line_labels = [r'reference', r'mixed-initiative NMPC']
    ax2 = fig.add_subplot(223, projection='3d')
    ax2.plot3D(robot_yref[0:N, 0], robot_yref[0:N, 1], robot_yref[0:N, 2], color='k', linewidth=1.5, linestyle='--')
    ax2.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color=vermelho, linewidth=2.0)
    ax2.plot3D([robot_yref[N, 0]], [robot_yref[N, 1]], [robot_yref[N, 2]], '*', color='k', markersize=10.0)
    ax2.view_init(27, 122)
    ax2.set_zticklabels([])
    ax2.set_yticklabels([])
    ax2.set_xticklabels([])
    ax2.grid(True, linewidth=0.5)
    ax2.legend(labels=line_labels, fontsize=lab, loc='upper center', bbox_to_anchor=(0.5, 0.05), ncol=2)

    # ===========================
    #  lambda magnified
    # ===========================
    ax3 = fig.add_subplot(322)
    mark_inset(ax1, ax3, loc1a=3, loc1b=4, loc2a=2, loc2b=1, fc="none", ec='0.5')
    tl = np.linspace(i * dt, (inset_idx + i) * dt, inset_idx)
    ax3.plot(tl, lambdas[i:(i + inset_idx)], color=vermelho, marker='D', markersize=mks)
    ax3.set_ylabel(r'$\lambda$', fontsize=lab+2)
    ax3.xaxis.set_tick_params(labelsize=tck)
    ax3.yaxis.set_tick_params(labelsize=tck)

    # ===========================
    #  maximum FC tube excursion
    # ===========================
    ax4 = fig.add_subplot(324)
    th = np.linspace(i * dt, (inset_idx + i) * dt, inset_idx)
    ax4.plot(th, horizon_idx[i:(inset_idx + i)], color=cinza, linestyle='--')
    for k in range(i, i + inset_idx, 1):
        if horizon_idx[k] == 0.0:
            ax4.plot(th[k-i], horizon_idx[k], color=bebe, linestyle='--', marker='D', markersize=mks)
        if horizon_idx[k] == 50.0:
            ax4.plot(th[k-i], horizon_idx[k], color=verde, linestyle='--', marker='D', markersize=mks)
        if 1.0 <= horizon_idx[k] < 50.0:
            ax4.plot(th[k-i], horizon_idx[k], color=rosa, linestyle='--', marker='D', markersize=mks)
    ax4.set_ylabel(r'$d_{\textrm{sat}}$', fontsize=lab+2)
    ax4.xaxis.set_tick_params(labelsize=tck)
    ax4.yaxis.set_tick_params(labelsize=tck)

    # ===========================
    #  open-loop trajectories
    # ===========================
    ax5 = fig.add_subplot(326)
    for k in range(i, i + inset_idx, 1):
        dt = Tf / N
        ti = np.linspace(k * dt, (N_acados + k) * dt, N_acados)
        lower_bound = 0.0
        upper_bound = threshold
        if horizon_idx[k] == 0.0:
            ax5.plot(ti, xn[k, :, :], color=bebe)
        if horizon_idx[k] == 50.0:
            ax5.plot(ti, xn[k, :, :], color=verde)
        if 1.0 <= horizon_idx[k] < 50.0:
            ax5.plot(ti, xn[k, :, :], color=rosa)
        ax5.plot(ti, upper_bound * np.ones(len(ti)), lw=1, color='black', ls='--')
        ax5.plot(ti, lower_bound * np.ones(len(ti)), lw=1, color='black', ls='--')
    ax5.set_ylabel(r'$\|\eta_i^\star-\eta_i^r\|$ (m)', fontsize=lab+2)
    ax5.set_xlabel(r'$t$ (s)', fontsize=lab)
    ax5.xaxis.set_tick_params(labelsize=tck)
    ax5.yaxis.set_tick_params(labelsize=tck)

    plt.savefig(directory + pilot + '_position' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_novice_expert(directory, filename_nov, N,
                       ref_simX: np.ndarray, simX: np.ndarray, adm_error: np.ndarray):

    rc('text', usetex=True)
    rc('font', family='serif')
    azul = [0.141, 0.572, 1]
    vermelho = [0.815, 0.122, 0.188]

    # get data
    novice_simX = parse(filename_nov)

    # 3D plot
    tube_size = adm_error
    tube_offset = 0.02 * np.array([1, 1, 0])
    cube1 = ref_simX[100, 0:3] + tube_size + tube_offset
    cube2 = ref_simX[250, 0:3] + tube_size + tube_offset

    cube_definition = [[(cube1[0], cube1[1] - 0.1, 0), (cube1[0], cube1[1] + 0.15, 0),
                        (cube1[0] + 0.2, cube1[1] - 0.1, 0), (cube1[0], cube1[1] - 0.1, 1.0)],
                       [(cube2[0] - 0.07, cube2[1] - 0.2, 0), (cube2[0] - 0.07, cube2[1] + 0.15, 0),
                        (cube2[0] + 0.02 + 0.15, cube2[1] - 0.2, 0), (cube2[0] - 0.07, cube2[1] - 0.2, 2.1)],
                       [(0.45, -0.3, 0), (0.45, 0.15, 0), (0.39, -0.3, 0), (0.45, -0.3, 2.6)],
                       [(0.0, 0.6, 0.0), (0.0, 0.37, 0.0), (-0.34, 0.6, 0.0), (0.0, 0.6, 3.2)]]

    fig = plt.figure(figsize=plt.figaspect(0.3))
    ax = fig.add_subplot(1, 3, 1, projection='3d')
    ax.plot3D(ref_simX[:N, 0], ref_simX[0:N, 1], ref_simX[:N, 2], color='k', linewidth=1.5, linestyle='--')
    ax.plot3D(novice_simX[:, 0], novice_simX[:, 1], novice_simX[:, 2], color=azul, linewidth=2.0)
    ax.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color=vermelho, linewidth=2.0)
    ax.plot3D([ref_simX[N, 0]], [ref_simX[N, 1]], [ref_simX[N, 2]], '*', color='k', markersize=10.0)
    ax.view_init(0, 1)
    ax.set_zticklabels([])
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.grid(True, linewidth=0.5)
    for k in range(4):
        plot_cube(cube_definition[k], ax)

    ax = fig.add_subplot(1, 3, 2, projection='3d')
    ax.plot3D(ref_simX[0:N, 0], ref_simX[0:N, 1], ref_simX[0:N, 2], color='k', linewidth=1.5, linestyle='--')
    ax.plot3D(novice_simX[:, 0], novice_simX[:, 1], novice_simX[:, 2], color=azul, linewidth=2.0)
    ax.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color=vermelho, linewidth=2.0)
    ax.plot3D([ref_simX[N, 0]], [ref_simX[N, 1]], [ref_simX[N, 2]], '*', color='k', markersize=10.0)
    ax.view_init(90, 90)
    ax.set_zticklabels([])
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.grid(True, linewidth=0.5)
    for k in range(4):
        plot_cube(cube_definition[k], ax)

    ax = fig.add_subplot(1, 3, 3, projection='3d')
    ax.plot3D(ref_simX[0:N, 0], ref_simX[0:N, 1], ref_simX[0:N, 2], color='k', linewidth=1.5, linestyle='--')
    ax.plot3D(novice_simX[:, 0], novice_simX[:, 1], novice_simX[:, 2], color=azul, linewidth=2.0)
    ax.plot3D(simX[:, 0], simX[:, 1], simX[:, 2], color=vermelho, linewidth=2.0)
    ax.plot3D([ref_simX[N, 0]], [ref_simX[N, 1]], [ref_simX[N, 2]], '*', color='k', markersize=10.0)
    ax.view_init(0, -90)
    ax.set_zticklabels([])
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.grid(True, linewidth=0.5)
    for k in range(4):
        plot_cube(cube_definition[k], ax)

    fig.tight_layout()
    plt.savefig(directory + 'plot_3D' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_artificial_commands(pilot, Tf, Nsim, simX: np.ndarray, replay_hinputs: np.ndarray):

    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, Nsim)
    azul = [0.141, 0.572, 1]
    vermelho = [0.815, 0.122, 0.188]

    line_labels = [r'mixed-initiative NMPC', r'human inputs']
    # ------- Font size ------- #
    tck = 18
    lab = 30
    lw = 3.0
    # ------------------------- #
    fig = plt.figure(figsize=(17, 8))

    phi = fig.add_subplot(311)
    phi.plot(t, simX[:Nsim, 3] * 180 / np.pi, color=vermelho, lw=lw)
    phi.plot(t, replay_hinputs[:Nsim, 0] * 180 / np.pi, color=azul, lw=lw)
    phi.grid(b=True, which='major', linestyle='--')
    phi.set_xticklabels([])
    phi.autoscale(enable=True, axis='x', tight=True)
    phi.set_ylabel(r'$\phi$ (deg)', fontsize=lab)
    phi.grid(True, linewidth=0.5)
    phi.xaxis.set_tick_params(labelsize=tck)
    phi.yaxis.set_tick_params(labelsize=tck)

    the = fig.add_subplot(312)
    the.plot(t, simX[:Nsim, 4] * 180 / np.pi, color=vermelho, lw=lw)
    the.plot(t, replay_hinputs[:Nsim, 1] * 180 / np.pi, color=azul, lw=lw)
    the.grid(b=True, which='major', linestyle='--')
    the.set_xticklabels([])
    the.set_ylabel(r'$\theta$ (deg)', fontsize=lab)
    the.autoscale(enable=True, axis='x', tight=True)
    the.grid(True, linewidth=0.5)
    the.xaxis.set_tick_params(labelsize=tck)
    the.yaxis.set_tick_params(labelsize=tck)

    thrust = fig.add_subplot(313)
    Ct = 5.95e+2  # [N/kHz^2] -- real experiments
    thr_robot = Ct * (simX[:Nsim, 12] ** 2 + simX[:Nsim, 13] ** 2 + simX[:Nsim, 14] ** 2 + simX[:Nsim, 15] ** 2)

    thrust.plot(t, thr_robot, color=vermelho, lw=lw)
    thrust.plot(t, replay_hinputs[:Nsim, 2], color=azul, lw=lw)
    thrust.set_xlabel(r'$t$ (s)', fontsize=lab)
    thrust.set_ylabel(r'$F_z$ (N)', fontsize=lab)
    thrust.grid(b=True, which='major', linestyle='--')
    thrust.autoscale(enable=True, axis='x', tight=True)
    thrust.xaxis.set_tick_params(labelsize=tck)
    thrust.yaxis.set_tick_params(labelsize=tck)
    fig.legend(labels=line_labels, loc='upper center', bbox_to_anchor=(0.445, 1.05), ncol=2, fontsize=lab+2)

    plt.savefig('results/hinputs_' + pilot + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_commands_virtual_horizon(directory, Tf, Nsim):

    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, Nsim)

    # Create a list of colours from the colormap viridis
    color = plt.get_cmap('Dark2')(np.linspace(0, 1, 8))

    prm = {'axes.prop_cycle': cycler('color', color)}
    plt.rcParams.update(prm)

    replay_hinputs_n = parse('output/new_hi/xe.txt')

    # novice pilot
    n10 = parse(directory + 'xe_10.txt')
    n20 = parse(directory + 'xe_20.txt')
    n30 = parse(directory + 'xe_30.txt')
    n40 = parse(directory + 'xe_40.txt')
    n50 = parse(directory + 'xe_50.txt')

    line_labels = [r'$N_t=10$', r'$N_t=20$', r'$N_t=30$', r'$N_t=40$', r'$N_t=50$', r'human inp']
    # ------- Font size ------- #
    tck = 18
    lab = 30
    lw = 3.0
    # ------------------------- #
    fig = plt.figure(figsize=(17, 8))

    phi = fig.add_subplot(311)
    phi.plot(t, n10[:Nsim, 3] * 180 / np.pi, lw=lw)
    phi.plot(t, n20[:Nsim, 3] * 180 / np.pi, lw=lw)
    phi.plot(t, n30[:Nsim, 3] * 180 / np.pi, lw=lw)
    phi.plot(t, n40[:Nsim, 3] * 180 / np.pi, lw=lw)
    phi.plot(t, n50[:Nsim, 3] * 180 / np.pi, lw=lw)
    phi.plot(t, replay_hinputs_n[:Nsim, 3] * 180 / np.pi, color='k', lw=2, ls='--')
    phi.grid(b=True, which='major', linestyle='--')
    phi.set_xticklabels([])
    phi.autoscale(enable=True, axis='x', tight=True)
    phi.set_ylabel(r'$\phi$ (deg)', fontsize=lab)
    phi.grid(True, linewidth=0.5)
    phi.xaxis.set_tick_params(labelsize=tck)
    phi.yaxis.set_tick_params(labelsize=tck)

    the = fig.add_subplot(312)
    the.plot(t, n10[:Nsim, 4] * 180 / np.pi, lw=lw)
    the.plot(t, n20[:Nsim, 4] * 180 / np.pi, lw=lw)
    the.plot(t, n30[:Nsim, 4] * 180 / np.pi, lw=lw)
    the.plot(t, n40[:Nsim, 4] * 180 / np.pi, lw=lw)
    the.plot(t, n50[:Nsim, 4] * 180 / np.pi, lw=lw)
    the.plot(t, replay_hinputs_n[:Nsim, 4] * 180 / np.pi, color='k', lw=2, ls='--')
    the.grid(b=True, which='major', linestyle='--')
    the.set_xticklabels([])
    the.set_ylabel(r'$\theta$ (deg)', fontsize=lab)
    the.autoscale(enable=True, axis='x', tight=True)
    the.grid(True, linewidth=0.5)
    the.xaxis.set_tick_params(labelsize=tck)
    the.yaxis.set_tick_params(labelsize=tck)

    thrust = fig.add_subplot(313)
    Ct = 5.95e+2  # [N/kHz^2] -- real experiments
    thr_robot10 = Ct * (n10[:Nsim, 5] ** 2 + n10[:Nsim, 6] ** 2 + n10[:Nsim, 7] ** 2 + n10[:Nsim, 8] ** 2)
    thr_robot20 = Ct * (n20[:Nsim, 5] ** 2 + n20[:Nsim, 6] ** 2 + n20[:Nsim, 7] ** 2 + n20[:Nsim, 8] ** 2)
    thr_robot30 = Ct * (n30[:Nsim, 5] ** 2 + n30[:Nsim, 6] ** 2 + n30[:Nsim, 7] ** 2 + n30[:Nsim, 8] ** 2)
    thr_robot40 = Ct * (n40[:Nsim, 5] ** 2 + n40[:Nsim, 6] ** 2 + n40[:Nsim, 7] ** 2 + n40[:Nsim, 8] ** 2)
    thr_robot50 = Ct * (n50[:Nsim, 5] ** 2 + n50[:Nsim, 6] ** 2 + n50[:Nsim, 7] ** 2 + n50[:Nsim, 8] ** 2)

    replay_thr = Ct * (replay_hinputs_n[:Nsim, 12] ** 2 + replay_hinputs_n[:Nsim, 13] ** 2 +
                       replay_hinputs_n[:Nsim, 13] ** 2 + replay_hinputs_n[:Nsim, 14] ** 2)

    thrust.plot(t, thr_robot10, lw=lw)
    thrust.plot(t, thr_robot20, lw=lw)
    thrust.plot(t, thr_robot30, lw=lw)
    thrust.plot(t, thr_robot40, lw=lw)
    thrust.plot(t, thr_robot50, lw=lw)
    thrust.plot(t, replay_thr, color='k', lw=2, ls='--')
    thrust.set_xlabel(r'$t$ (s)', fontsize=lab)
    thrust.set_ylabel(r'$F_z$ (N)', fontsize=lab)
    thrust.grid(b=True, which='major', linestyle='--')
    thrust.autoscale(enable=True, axis='x', tight=True)
    thrust.xaxis.set_tick_params(labelsize=tck)
    thrust.yaxis.set_tick_params(labelsize=tck)
    fig.legend(labels=line_labels, loc='upper center', bbox_to_anchor=(0.45, 1.0), ncol=6, fontsize=20)

    plt.savefig('results/Nt_experienced' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_bounded_var(pilot, Tf, N, simX: np.ndarray):
   # rc('text', usetex=True)
    #rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)

    xs = t
    verts = []
    zs = [1.0, 2.0, 3.0, 4.0]

    speed = simX[:, 12:16]
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')

    z1 = zs[0] * np.ones(N)
    z2 = zs[1] * np.ones(N)
    z3 = zs[2] * np.ones(N)
    z4 = zs[3] * np.ones(N)
    upper_bound = 0.09 * np.ones(N)
    for z in zs:
        ys = speed[:, int(z-1)]
        verts.append(polygon_under_graph(xs, ys))

    poly = PolyCollection(verts, facecolors=[cc('r'), cc('g'), cc('b'),
                                             cc('y')])
    poly.set_alpha(0.5)
    ax.add_collection3d(poly, zs=zs, zdir='y')

    ax.plot(xs, z1, speed[:N, 0], color='k')
    ax.plot(xs, z1, upper_bound, color='k', ls='-.')
    ax.plot(xs, z2, speed[:N, 1], color='k')
    ax.plot(xs, z2, upper_bound, color='k', ls='-.')
    ax.plot(xs, z3, speed[:N, 2], color='k')
    ax.plot(xs, z3, upper_bound, color='k', ls='-.')
    ax.plot(xs, z4, speed[:N, 3], color='k')
    ax.plot(xs, z4, upper_bound, color='k', ls='-.')

    #ax.set_xlabel(r'$t$ (s)', fontsize=13)
    #ax.set_xlim3d(0, 9)
    #ax.set_ylabel(r'$i^{\textrm{th}}$ propeller', fontsize=13)
    #ax.set_ylim3d(0, 5)
    #ax.set_zlabel(r'$\Omega$ (krpm)', fontsize=13)
    #ax.set_zlim3d(0, 0.1)
    #ax.view_init(65, -135)

    # make the panes transparent
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # make the grid lines transparent
    ax.xaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.yaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.zaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)

    plt.savefig('results/propeller_speed_' + pilot + '.pdf', dpi=400, bbox_inches="tight")
    fig.tight_layout()
    plt.show()


def plot_trajectory_error(filename_novice, Tf,  N, ref_simX: np.ndarray, simX: np.ndarray, error):
    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    azul = [0.141, 0.572, 1]
    vermelho = [0.815, 0.122, 0.188]

    line_labels = [r'novice', r'experienced']

    # get data
    novice_simX = parse(filename_novice)
    novice = np.ndarray((N,))
    experienced = np.ndarray((N,))
    reference = np.ndarray((N,))
    for i in range(N):
        novice[i] = distance.euclidean(novice_simX[i, 0:3], ref_simX[i, 0:3])
        experienced[i] = distance.euclidean(simX[i, 0:3], ref_simX[i, 0:3])
        reference[i] = error

    fig = plt.figure(figsize=(5, 5))

    ea = fig.add_subplot(111)
    ea.plot(t, novice, color=azul)
    ea.plot(t, experienced, color=vermelho)
    ea.plot(t, reference, color='k', ls='--')
    ea.autoscale(enable=True, axis='x', tight=True)
    ea.grid(b=True, which='major', linestyle='--')
    ea.set_xlabel(r'$t$ (s)', fontsize=18)
    ea.set_ylabel(r'$Distance$ (m)', fontsize=18)
    # ea.xaxis.set_tick_params(labelsize=13)
    # ea.yaxis.set_tick_params(labelsize=13)
    fig.legend(labels=line_labels, loc='upper center', bbox_to_anchor=(0.549, 1.05), ncol=2, fontsize=18)

    plt.savefig('results/trajectory_error' + '.pdf', dpi=400, bbox_inches="tight")
    plt.show()


def plot_cube(cube_definition, ax):
    cube_definition_array = [
        np.array(list(item))
        for item in cube_definition
    ]

    points = []
    points += cube_definition_array
    vectors = [
        cube_definition_array[1] - cube_definition_array[0],
        cube_definition_array[2] - cube_definition_array[0],
        cube_definition_array[3] - cube_definition_array[0]
    ]

    points += [cube_definition_array[0] + vectors[0] + vectors[1]]
    points += [cube_definition_array[0] + vectors[0] + vectors[2]]
    points += [cube_definition_array[0] + vectors[1] + vectors[2]]
    points += [cube_definition_array[0] + vectors[0] + vectors[1] + vectors[2]]

    points = np.array(points)

    edges = [
        [points[0], points[3], points[5], points[1]],
        [points[1], points[5], points[7], points[4]],
        [points[4], points[2], points[6], points[7]],
        [points[2], points[6], points[3], points[0]],
        [points[0], points[2], points[4], points[1]],
        [points[3], points[6], points[7], points[5]]
    ]

    gray = [0.7, 0.7, 0.7]
    faces = Poly3DCollection(edges, linewidths=1, edgecolors=gray)
    faces.set_facecolor((0, 0, 1, 0.04))

    ax.add_collection3d(faces)

    # Plot the points themselves to force the scaling of the axes
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=0)


def plot_mpc_box(N, simX: np.ndarray, ref: np.ndarray):
    rc('text', usetex=True)
    rc('font', family='serif')
    azul = [0.141, 0.572, 1]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(simX[:N, 0], simX[:N, 1], simX[:N, 2], color=azul, linewidth=2.0)
    ax.plot3D([ref[0]], [ref[1]], [ref[2]], '*', color='k', markersize=10.0)
    ax.view_init(15, -149)
    ax.set_xlabel(r'$x$ (m)', fontsize=15)
    ax.set_ylabel(r'$y$ (m)', fontsize=15)
    ax.set_zlabel(r'$z$ (m)', fontsize=15)
    # ax.set_zticklabels([])
    # ax.set_yticklabels([])
    # ax.set_xticklabels([])
    ax.grid(True, linewidth=0.5)

    cube_definition = [(0, -0.3, 0.2), (0, 0.3, 0.2), (0.6, -0.3, 0.2), (0, -0.3, 0.55)]
    plot_cube(cube_definition, ax)
    plt.show()


def plot_mpc_obstacles(Tf, N, simX: np.ndarray, ref: np.ndarray, xyzr):

    rc('text', usetex=True)
    rc('font', family='serif')
    t = np.linspace(0.0, Tf, N)
    azul = [0.141, 0.572, 1]
    vermelho = [0.815, 0.122, 0.188]

    # Create a list of colours from the colormap viridis
    color = plt.get_cmap('Set1')(np.linspace(0, 1, 9))

    prm = {'axes.prop_cycle': cycler('color', color)}
    plt.rcParams.update(prm)

    fig = plt.figure(figsize=plt.figaspect(0.4))
    ax = fig.add_subplot(121, projection='3d')

    x = xyzr[0][:]
    y = xyzr[1][:]
    z = xyzr[2][:]
    r = xyzr[3][:]

    # plot obstacles
    # draw a sphere for each data point
    transparency = 0.2
    for (xi, yi, zi, ri) in zip(x, y, z, r):
        (xs, ys, zs) = drawSphere(xi, yi, zi, ri)
        ax.plot_surface(xs, ys, zs, color='r', alpha=transparency)


    ax.plot3D(simX[:N, 0], simX[:N, 1], simX[:N, 2], color=azul, linewidth=2.0)
    ax.plot3D([ref[0]], [ref[1]], [ref[2]], '*', color='k', markersize=10.0)
    ax.view_init(19, -170)
    # ax.set_xlabel(r'$x$ (m)', fontsize=15)
    # ax.set_ylabel(r'$y$ (m)', fontsize=15)
    # ax.set_zlabel(r'$z$ (m)', fontsize=15)
    ax.set_zticklabels([])
    ax.set_yticklabels([])
    ax.set_xticklabels([])

    # plot workspace box
    cube_definition = [(-2, -2, 0.5), (-2, 2, 0.5), (2, -2, 0.5), (-2, -2, 3)]
    plot_cube(cube_definition, ax)

    ax.axes.set_xlim3d(left=-2.1, right=2.1)
    ax.axes.set_ylim3d(bottom=-2.1, top=2.1)
    ax.axes.set_zlim3d(bottom=-2.1, top=2.1)

    # make the panes transparent
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # make the grid lines transparent
    ax.xaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.yaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.zaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)

    plt.axis('off')

    # Euclidean distance
    ax2 = fig.add_subplot(122)
    distance1 = np.ndarray((N,))
    distance2 = np.ndarray((N,))
    distance3 = np.ndarray((N,))
    distance4 = np.ndarray((N,))
    distance5 = np.ndarray((N,))
    distance6 = np.ndarray((N,))
    distance7 = np.ndarray((N,))
    distance8 = np.ndarray((N,))

    clearance = 0.4 * np.ones(N)
    for j in range(N):
        distance1[j] = np.sqrt((simX[j, 0] - x[0]) ** 2 + (simX[j, 1] - y[0]) ** 2 + (simX[j, 2] - z[0]) ** 2)
        distance2[j] = np.sqrt((simX[j, 0] - x[1]) ** 2 + (simX[j, 1] - y[1]) ** 2 + (simX[j, 2] - z[1]) ** 2)
        distance3[j] = np.sqrt((simX[j, 0] - x[2]) ** 2 + (simX[j, 1] - y[2]) ** 2 + (simX[j, 2] - z[2]) ** 2)
        distance4[j] = np.sqrt((simX[j, 0] - x[3]) ** 2 + (simX[j, 1] - y[3]) ** 2 + (simX[j, 2] - z[3]) ** 2)
        distance5[j] = np.sqrt((simX[j, 0] - x[4]) ** 2 + (simX[j, 1] - y[4]) ** 2 + (simX[j, 2] - z[4]) ** 2)
        distance6[j] = np.sqrt((simX[j, 0] - x[5]) ** 2 + (simX[j, 1] - y[5]) ** 2 + (simX[j, 2] - z[5]) ** 2)
        distance7[j] = np.sqrt((simX[j, 0] - x[6]) ** 2 + (simX[j, 1] - y[6]) ** 2 + (simX[j, 2] - z[6]) ** 2)
        distance8[j] = np.sqrt((simX[j, 0] - x[7]) ** 2 + (simX[j, 1] - y[7]) ** 2 + (simX[j, 2] - z[7]) ** 2)
    ax2.plot(t, distance1, lw=2.0)
    ax2.plot(t, distance2, lw=2.0)
    ax2.plot(t, distance3, lw=2.0)
    ax2.plot(t, distance4, lw=2.0)
    ax2.plot(t, distance5, lw=2.0)
    ax2.plot(t, distance6, lw=2.0)
    ax2.plot(t, distance7, lw=2.0)
    ax2.plot(t, distance8, lw=2.0)
    ax2.plot(t, clearance, color='k', ls='--')
    ax2.autoscale(enable=True, axis='x', tight=True)
    ax2.grid(b=True, which='major', linestyle='--')

    plt.tight_layout()

    plt.show()


def plot_mpc_tube(N, simX: np.ndarray, ref: np.ndarray):
    rc('text', usetex=True)
    rc('font', family='serif')
    azul = [0.141, 0.572, 1]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(ref[0:N, 0], ref[0:N, 1], ref[0:N, 2], color='k', linewidth=1.5, linestyle='--')
    ax.plot3D([ref[N, 0]], [ref[N, 1]], [ref[N, 2]], '*', color='k', markersize=10.0)
    ax.plot3D(simX[:N, 0], simX[:N, 1], simX[:N, 2], color=azul, linewidth=2.0)
    ax.view_init(43, -148)
    ax.set_xlabel(r'$x$ (m)', fontsize=15)
    ax.set_ylabel(r'$y$ (m)', fontsize=15)
    ax.set_zlabel(r'$z$ (m)', fontsize=15)
    # ax.set_zticklabels([])
    # ax.set_yticklabels([])
    # ax.set_xticklabels([])
    ax.grid(True, linewidth=0.5)

    plt.show()


def drawSphere(xCenter, yCenter, zCenter, r):

    # draw sphere
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = np.cos(u)*np.sin(v)
    y = np.sin(u)*np.sin(v)
    z = np.cos(v)
    # shift and scale sphere
    x = r*x + xCenter
    y = r*y + yCenter
    z = r*z + zCenter

    return x, y, z


def gen_random_obs(num_spheres):
    spheres = []
    r1 = 0.4
    r = r1 * np.ones((num_spheres,))
    while len(spheres) < num_spheres * 4:
        r1 = r1
        x1 = np.random.uniform(-1.5 + r1, 2.0 - r1, 1)
        y1 = np.random.uniform(-2.0 + r1, 2.0 - r1, 1)
        z1 = np.random.uniform(0.5 + r1, 2.75 - r1, 1)
        vec = [x1, y1, z1, r1]
        spheres += vec
    x = spheres[0::4]
    y = spheres[1::4]
    z = spheres[2::4]

    return x, y, z, r


def draw_sphere_box(x, y, z, r):

    rc('text', usetex=True)
    rc('font', family='serif')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # plot obstacles
    # draw a sphere for each data point
    for (xi, yi, zi, ri) in zip(x, y, z, r):
        (xs, ys, zs) = drawSphere(xi, yi, zi, ri)
        ax.plot_wireframe(xs, ys, zs, color="r")

    # plot workspace box
    cube_definition = [(-3, -3, 0.4), (-3, 3, 0.4), (3, -3, 0.4), (-3, -3, 3)]
    plot_cube(cube_definition, ax)

    ax.set_xticks([-3, -1.5, 0, 1.5, 3.0])
    ax.set_yticks([-3, -1.5, 0, 1.5, 3.0])
    ax.set_zticks([-3, -1.5, 0, 1.5, 3.0])

    ax.axes.set_xlim3d(left=-2.9, right=2.9)
    ax.axes.set_ylim3d(bottom=-2.9, top=2.9)
    ax.axes.set_zlim3d(bottom=-2.9, top=2.9)

    # make the panes transparent
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # make the grid lines transparent
    ax.xaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.yaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)
    ax.zaxis._axinfo["grid"]['color'] = (1, 1, 1, 0)

    plt.show()


def solution_times(file):
    s1 = parse(file)
    avg_solution_time = np.mean(s1[:, 9])*1000.0

    return avg_solution_time


def boxplot_solution_times():

    rc('text', usetex=True)
    rc('font', family='serif')
    xn10 = parse('results/horizons/xn_10.txt')
    xn20 = parse('results/horizons/xn_20.txt')
    xn30 = parse('results/horizons/xn_30.txt')
    xn40 = parse('results/horizons/xn_40.txt')
    xn50 = parse('results/horizons/xn_50.txt')

    xe10 = parse('results/horizons/xe_10.txt')
    xe20 = parse('results/horizons/xe_20.txt')
    xe30 = parse('results/horizons/xe_30.txt')
    xe40 = parse('results/horizons/xe_40.txt')
    xe50 = parse('results/horizons/xe_50.txt')

    fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(6, 8))

    box_plot_datan = [xn10[:, 9]*1000, xn20[:, 9]*1000, xn30[:, 9]*1000, xn40[:, 9]*1000, xn50[:, 9]*1000]
    axes[0].violinplot(box_plot_datan, showmeans=False, showmedians=True)
    axes[0].set_xticklabels([])

    box_plot_datae = [xe10[:, 9]*1000, xe20[:, 9]*1000, xe30[:, 9]*1000, xe40[:, 9]*1000, xe50[:, 9]*1000]
    axes[1].violinplot(box_plot_datae, showmeans=False, showmedians=True)
    axes[1].set_xlabel(r'$N_b$', fontsize=14)

    # adding horizontal grid lines
    for ax in axes:
        ax.set_ylabel(r'Solution time (ms)', fontsize=14)

    # add x-tick labels
    plt.setp(axes[1], xticks=[y + 1 for y in range(len(box_plot_datae))],
             xticklabels=[r'$10$', r'$20$', r'$30$', r'$40$', r'$50$'])

    plt.savefig('results/violin.pdf', dpi=400, bbox_inches="tight")
    fig.tight_layout()

    plt.show()
