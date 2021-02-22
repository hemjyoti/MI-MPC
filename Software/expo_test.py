import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from scipy.spatial import distance

Tf = 9
Ts = 0.015
N = int(Tf / Ts)
maximum_error = np.array([0.1, 0.1, 0.1])
admissible_euclidean_distance = distance.euclidean(maximum_error, [0, 0, 0])

tube_error = np.linspace(0, admissible_euclidean_distance, N)

exp_W = np.ndarray((N,))
lin_W = np.ndarray((N,))
rate = np.ndarray((N,))
d_sat = np.ndarray((N,))
d_sat2 = np.ndarray((N,))

mu = 0.99
t1 = mu*admissible_euclidean_distance
t2 = 50
t3 = 0.3*admissible_euclidean_distance
xi = 1e-13

for j in range(N):
    arg = (tube_error[j]/admissible_euclidean_distance) * np.tan(mu*(np.pi/2))
    d_sat[j] = admissible_euclidean_distance * (2 / np.pi) * np.arctan(arg)

    d_sat2[j] = t1*(1+xi*np.exp(-t2*(tube_error[j]-t3)))**(-1/xi)

    exp_W[j] = np.sqrt(admissible_euclidean_distance ** 2 - d_sat2[j] ** 2) / admissible_euclidean_distance

    print(exp_W[j], ', ', tube_error[j], ', ', d_sat2[j])

    # rate[j] = tube_error[j]/exp[j]
    # print(tube_error[j], ', ', rate[j])

rc('text', usetex=True)
rc('font', family='serif')
fig = plt.figure(figsize=(5, 5))
exp = fig.add_subplot(111)
exp.plot(exp_W, d_sat2, lw=2, color='k')
# # exp.plot(lin_W, tube_error, lw=2, color='r')
exp.grid(b=True, which='major', linestyle='--')
exp.set_ylabel(r'$d_{\rm sat}$', fontsize=21)
exp.set_xlabel(r'$\lambda$', fontsize=21)
labels = [item.get_text() for item in exp.get_yticklabels()]
labels[1] = r'0.0'
labels[8] = r'$\|\alpha\|$'
exp.set_yticklabels(labels)
exp.xaxis.set_tick_params(labelsize=12)
exp.yaxis.set_tick_params(labelsize=12)
fig.tight_layout()
plt.savefig('mi_tube/decay_fcn' + '.pdf', dpi=400, bbox_inches="tight")
plt.show()

fig2 = plt.figure(figsize=(5, 5))
tube_dsat = fig2.add_subplot(111)
# tube_dsat.plot(tube_error, d_sat, lw=2, color='pink')
tube_dsat.plot(tube_error, d_sat2, lw=2, color='k')
tube_dsat.grid(b=True, which='major', linestyle='--')
tube_dsat.set_ylabel(r'$d_{\rm sat}$', fontsize=21)
tube_dsat.set_xlabel(r'$d_{\rm max}$', fontsize=21)
labels = [item.get_text() for item in exp.get_yticklabels()]
labels[1] = r'0.0'
labels[8] = r'$\|\alpha\|$'
tube_dsat.set_yticklabels(labels)
tube_dsat.xaxis.set_tick_params(labelsize=12)
tube_dsat.yaxis.set_tick_params(labelsize=12)
plt.savefig('mi_tube/dsat_fcn' + '.pdf', dpi=400, bbox_inches="tight")

plt.show()
