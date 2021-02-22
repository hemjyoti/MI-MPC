clear all;
clc;

N = 1000;
N_acados = 50;
Tf = 6.0;

g0 = 9.8066;     % [m.s^2]
mq = 1.04;       % [Kg] -- for Gazebo simulation
Ct = 5.95e+2;    % [N/kHz^2] -- real experiments
hov_w = sqrt((mq * g0) / (4 * Ct));

r = 0.3;
h = 0.035;
i = 1;
tsim = linspace(0, 15, N);

for t = linspace(0, 15, N)
    h = h + 0.005;
    x(i)          = r * cos(t); 
    y(i)          = r * sin(t);
    z(i)          = h;
    i = i+1;
end

n_steps = length(x);

ea = zeros(n_steps,3);
vb = zeros(n_steps,3);
wb = zeros(n_steps,3);
u  = hov_w*ones(n_steps,4);
tau = zeros(n_steps,4);

traj = [x.' y.' z.' ea vb wb u tau];

extra_window = repmat(traj(end,:),N_acados,1);

ref_trajectory = [traj;extra_window];

fileID = fopen('helix_traj.txt','w');
for i=1:length(ref_trajectory)
    fprintf(fileID, '%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n',ref_trajectory(i,:));
end
fclose(fileID);