clear all;
clc;

syms x y z vbx vby vbz wx wy wz w1 w2 w3 w4 ...
    Ixx Iyy Izz Ct Cd g0 l mq phi theta psi dphi dtheta dpsi real;

assume(x,'real'); assume(y,'real'); assume(z,'real');
assume(phi,'real'); assume(theta,'real'); assume(psi,'real');
assume(dphi,'real'); assume(dtheta,'real'); assume(dpsi,'real');
assume(vbx,'real'); assume(vby,'real'); assume(vbz,'real');
assume(wx,'real'); assume(wy,'real'); assume(wz,'real');
assume(w1,'real'); assume(w2,'real'); assume(w3,'real'); assume(w4,'real');
assume(Ixx,'real'); assume(Iyy,'real'); assume(Izz,'real'); assume(l,'real');
assume(g0,'real'); assume(mq,'real'); assume(Ct,'real'); assume(Cd,'real');

% Rotation matrix form inertial {I} to body {B} frame (XYZ)
Rx      = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Ry      = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
Rz      = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0; 0 0 1];
R       = Rz*Ry*Rx;

%%%%%%%%%%%%%%% Inertia Tensor  in the body frame {B} %%%%%%%%%%%%%%%%%%%%
I  = diag([Ixx,Iyy,Izz]);

%%%%%%%%%%%%%%%%%%%% Thrust force in the body frame {B} %%%%%%%%%%%%%%%%%%
Fz = Ct*(w1*w1+w2*w2+w3*w3+w4*w4);

%%%%%%%%%%%%%%%%%% Linear acceleration vector in the body frame {B} %%%%%%
syms  vbx vby vbz wx wy wz
k = cross([wx;wy;wz],[vbx;vby;vbz]);
ab = [0;0;Fz/mq]-R*[0;0;g0]-k;

%%%%%%%%%%%%%%%% Linear velocity vector in the inertial frame {I} %%%%%%%%
vi = R.'*[vbx;vby;vbz];

%%%%%%%%%%%%%%%%%%%%%% Moments in the body frame {B} %%%%%%%%%%%%%%%%%%%%%
Mx = (l*Ct)*(w2*w2-w4*w4);
My = (l*Ct)*(w3*w3-w1*w1);
Mz = Cd*(w1*w1+w3*w3-w2*w2-w4*w4);

%%%%%%%%%%%%%%%%%% Angular acceleration in the body frame {B} %%%%%%%%%%%%
awb = inv(I)*([Mx;My;Mz]-cross([wx;wy;wz],I*[wx;wy;wz]));

%%%%%%%%% Angular velocity propagation in the body frame  {B} %%%%%%%%%%%%
omega = Rz*Ry*[0;0;dphi]+Rz*[0;dtheta;0]+[dpsi;0;0];

Momega = jacobian(omega,[dphi;dtheta;dpsi]);
prop_euler = simplify(inv(Momega),'Steps', 15);
deuler = prop_euler*[wx;wy;wz];
%%%%%%%%%%%%%%%%%%%% Motor dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms t1 t2 t3 t4 b Jm
dw = ([t1;t2;t3;t4] - b * [w1;w2;w3;w4])/Jm;
%% state vector
dot.x = vi(1);
dot.y = vi(2);
dot.z = vi(3);
dot.phi = deuler(1);
dot.theta = deuler(2);
dot.psi = deuler(3);
dot.vbx = ab(1);
dot.vby = ab(2);
dot.vbz = ab(3);
dot.wx = awb(1);
dot.wy = awb(2);
dot.wz = awb(3);
dot.w1 = dw(1);
dot.w2 = dw(2);
dot.w3 = dw(3);
dot.w4 = dw(4);
dot.fz = 2*Ct*(dot.w1+dot.w2+dot.w3+dot.w4);
%% motor mixing
% fz = Ct*(w1+w2+w3+w4);
% mx = (l*Ct)*(-w1-w2+w3+w4);
% my = (l*Ct)*(-w1+w2+w3-w4);
% mz = Cd*(-w1+w2-w3+w4);
% 
% omega = [w1;w2;w3;w4];
% %G = [fz;mx;my;mz];
% G = [Fz;Mx;My;Mz];
% Gamma = jacobian(G,omega);
% Gammainv = inv(Gamma);
% 
% %% linear model
% syms fh
% % nonlinear system, dx(t)/dt = f(x,u,t)
% xDot = [dot.x;dot.y;dot.z;
%          dot.phi;dot.theta;dot.psi;
%          dot.vbx;dot.vby;dot.vbz;
%          dot.wx;dot.wy;dot.wz;
%          dot.w1;dot.w2;dot.w3;dot.w4;
%          dot.phi;dot.theta;dot.psi;dot.fz];
% X = [x;y;z;phi;theta;psi;vbx;vby;vbz;wx;wy;wz;w1;w2;w3;w4;phi;theta;psi;fh];
% U = [t1;t2;t3;t4];
% 
% % linear system
% A_symbolic = simplify(jacobian(xDot, X));
% B_symbolic = jacobian(xDot, U);
% C_symbolic = jacobian(X, X);
% D_symbolic = jacobian(X, U);
% 
% xDot2 = [dot.x;dot.y;dot.z;
%          dot.phi;dot.theta;dot.psi;
%          dot.vbx;dot.vby;dot.vbz;
%          dot.wx;dot.wy;dot.wz;
%          dot.w1;dot.w2;dot.w3;dot.w4;
%          dot.phi;dot.theta;dot.psi;fh];
% X2 = [phi;theta;wz;fh];
% A2_symbolic = simplify(jacobian(xDot2, X2));
% 
% xdt = A_symbolic*X + B_symbolic*U;
% %% lyap
% syms lambda_factor
% 
% %% f_expr
% 
% A_alg = simplify(subs(A_symbolic, {g0 Ixx Iyy Izz phi theta psi vbx vby vbz wx wy wz Jm b mq Ct Cd w1 w2 w3 w4 l}, ...
%                 [sym(9.8066) sym(0.010) sym(0.010) sym(0.070) sym(0.0) sym(0.0) sym(0.0) ...
%                 sym(0.0) sym(0.0) sym(0.0) sym(0.0) sym(0.0) sym(0.0) sym(0.08) sym(5e-4)...
%                 sym(1.04) sym(5.95e+2) sym(1.0e+1) sym(0.0655) sym(0.0655) sym(0.0655) sym(0.0655) sym(0.23)]));
% % compute numerical values            
% A = eval(A_alg); 
% nx = 20;
% g0 = 9.8066; 
% mq = 1.04; 
% Ixx = 0.010;  
% Iyy = 0.010; 
% Izz = 0.070;  
% Cd = 1.0e+1;
% Ct = 5.95e+2;  
% l = 0.23;  
% Jm = 0.08;  
% b = 5e-4; 
% 
% Q = eye(nx);
% Q(1,1) = 250.0;  
% Q(2,2) = 250.0;  
% Q(3,3) = 300.0;  
% Q(4,4) = 30.0;  
% Q(5,5) = 30.0;  
% Q(6,6) = 30.0;  
% Q(7,7) = 5.0;  
% Q(8,8) = 5.0;  
% Q(9,9) = 5.0;  
% Q(10,10) = 5.0;  
% Q(11,11) = 5.0;  
% Q(12,12) = 5.0;  
% Q(13,13) = 30.0;  
% Q(14,14) = 30.0;  
% Q(15,15) = 30.0; 
% Q(16,16) = 30.0;  
% 
% Q(17,17) = 30.0; 
% Q(18,18) = 30.0; 
% Q(19,19) = 30.0;  
% Q(20,20) = Ct * Q(13,13);  
% 
% %%
% M1 = eye(16);
% M2 = zeros(4,4);
% M = blkdiag(M1,M2);
% 
% S1 = zeros(16,16);
% S2 = eye(4);
% S = blkdiag(S1,S2);
% lambda_factor = 1;
% 
% Pprime = (1-lambda_factor)*M*Q + lambda_factor*S*Q;
% Qprime = Pprime;
% 
% eq = Pprime - A.'* Pprime * A - Qprime; 
% 
% 
% 
% %eq = ((1-lambda_factor) * Qxi + lambda_factor * Qnu) - A.' * ((1-lambda_factor) * Pxi + lambda_factor * Pnu).* A - ((1-lambda_factor) * Qxi + lambda_factor * Qnu);
% 
% 
