N_acados = 50;
g0 = 9.8066;     % [m.s^2]
mq = 1.04;       % [Kg] -- for Gazebo simulation
Ct = 5.95e+2;    % [N/kHz^2] -- real experiments
hov_w = sqrt((mq * g0) / (4 * Ct));


f=0.3;
Amp=1;
ts=0.015;
T=8;
t=0:ts:T;
yp=Amp*sin(2*pi*f*t);
x = t-1.75;
y = yp;
plot(x,y);

n_steps = length(t);
z = 1.5*ones(n_steps, 1);
ea = zeros(n_steps,3);
vb = zeros(n_steps,3);
wb = zeros(n_steps,3);
u  = hov_w*ones(n_steps,4);
tau = zeros(n_steps,4);

traj = [x.' y.' z ea vb wb u tau];

extra_window = repmat(traj(end,:),N_acados,1);

ref_trajectory = [traj;extra_window];
 
fileID = fopen('trajectory/sin_traj.txt','w');
for i=1:length(ref_trajectory)
    fprintf(fileID, '%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n',ref_trajectory(i,:));
end
fclose(fileID);