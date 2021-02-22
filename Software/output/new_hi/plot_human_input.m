clear all;
clc;
filename = 'xn.txt';
% filename = 'helix_traj.txt';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
% for k = [3, 5]
%    disp(A.colheaders{1, k})
%    disp(A.data(:, k))
%    disp(' ')
% end
[A,delimiterOut]=importdata(filename);
figure(1115);
t=linspace(0,80,length(A(:,1)));
% subplot(4,1,1)
plot(A(:,4),'LineWidth',5);
% hold on;
% plot(A(:,8)/1000000,-A(:,2)/60, 'LineWidth',4);
% hold on;
% plot(A(:,8)/1000000,9710/120*ones(1,length(A(:,1))), 'LineWidth',4);
% hold off;
% xlabel('Time (sec)');
% ylabel('speed [Hz]') ;
% legend('\Omega [Hz]','\Omega_d [Hz]')
% propertyeditor('on');
% grid on;
% set(gca,'Fontsize',20);

