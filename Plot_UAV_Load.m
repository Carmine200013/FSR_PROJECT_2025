disp('Computing State Variables and Configuration Errors ') ;
t=out.time;
x=squeeze(out.x);
x=x';
xLd_=squeeze(out.xLd);
xLd=xLd_';
e_p = squeeze(out.err_p);
e_v = squeeze(out.err_dot_p);
e_q = squeeze(out.err_q);
e_om = squeeze(out.err_om);
e_R = squeeze(out.err_R);
e_Om = squeeze(out.err_Om);
f = squeeze(out.f);
M = squeeze(out.M);

%% Plot
index = round(linspace(1, length(t), round(0.1*length(t)))) ;
folderPath = '.\Plot';

p = figure('Renderer','painters','Position',[10 10 900 550]);  
subplot(3,1,1);
hold on
plot(t(index),x(index,1),'-b','LineWidth', 1.3);
plot(t(index),xLd(index,1),':r','LineWidth', 1.5);
plot(t(index),x(index,1)-x(index,4),'-g','LineWidth', 1.3);
hold off
set(gca, 'FontSize',12); grid on; 
legend('$x_L$','$x_{L,d}$','$x_Q$','FontSize', 10,'Interpreter', 'latex', 'Location','southeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('x [m]', 'Interpreter', 'latex');
xlim([min(t) max(t)])
%ylim([min(x(:,1))-0.2 max(x(:,1))+0.2])

subplot(3,1,2);
hold on
plot(t(index),x(index,2),'-b', 'LineWidth', 1.3);
plot(t(index),xLd(index,2),':r', 'LineWidth', 1.5);
plot(t(index),x(index,2)-x(index,5),'-g', 'LineWidth', 1.3);
hold off
set(gca, 'FontSize',12); grid on;
legend('$y_L$','$y_{L,d}$','$y_Q$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('y [m]', 'Interpreter', 'latex');
xlim([min(t) max(t)])
%ylim([min(x(:,2))-0.2 max(x(:,2))+0.2])
%ylim([-5 3])
%ylim([-1 1])

subplot(3,1,3);
hold on
plot(t(index),x(index,3),'-b', 'LineWidth', 1.3);
plot(t(index),xLd(index,3),':r', 'LineWidth', 1.5);
plot(t(index),x(index,3)-x(index,6),'-g', 'LineWidth', 1.3);
hold off
set(gca, 'FontSize',12); grid on;
legend('$z_L$','$z_{L,d}$','$z_Q$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('z [m]', 'Interpreter', 'latex');
xlim([min(t) max(t)])
%ylim([-5 3])
%ylim([0 2.5])
%ylim([min(x(:,3))-0.2 max(x(:,3))+0.2])
sgtitle('\textbf{UAV and Load Position}', 'FontSize', 16, 'Interpreter', 'latex');

a = figure('Renderer','painters','Position',[10 10 900 550]);
plot3(x(index,1),x(index,2),x(index,3),'-b',xLd(index,1),xLd(index,2),xLd(index,3),':r',x(index,1)-x(index,4),x(index,2)-x(index,5),x(index,3)-x(index,6),'-g', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Trajectory}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$traj_L$','$traj_d$','$traj_Q$', 'FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('x-axis', 'Interpreter', 'latex');ylabel('y-axis', 'Interpreter', 'latex');zlabel('z-axis', 'Interpreter', 'latex');

ep = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_p(1,index),'-g',t(index),e_p(2,index),'-r',t(index),e_p(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Load Position Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{p,x}$','$e_{p,y}$','$e_{p,z}$','FontSize', 10,'Interpreter', 'latex', 'Location','east');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [m]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

ev = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_v(1,index),'-g',t(index),e_v(2,index),'-r',t(index),e_v(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Load Linear Velocity Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{v,x}$','$e_{v,y}$','$e_{v,z}$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [m/s]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

eq = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_q(1,index),'-g',t(index),e_q(2,index),'-r',t(index),e_q(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Load Orientation Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{q,x}$','$e_{q,y}$','$e_{q,z}$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [rad]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

eom = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_om(1,index),'-g',t(index),e_om(2,index),'-r',t(index),e_om(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Load Angular Velocity Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{\dot{q},x}$','$e_{\dot{q},y}$','$e_{\dot{q},z}$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [rad/s]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

eR = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_R(1,index),'-g',t(index),e_R(2,index),'-r',t(index),e_R(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{UAV Orientation Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{R,x}$','$e_{R,y}$','$e_{R,z}$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [rad]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

eOm = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),e_Om(1,index),'-g',t(index),e_Om(2,index),'-r',t(index),e_Om(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{UAV Angular Velocity Error}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_{\Omega,x}$','$e_{\Omega,y}$','$e_{\Omega,z}$','FontSize', 10,'Interpreter', 'latex', 'Location','northeast');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('Error [rad/s]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

uT = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),f(index), '-b' ,'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Thrust}', 'FontSize', 16, 'Interpreter', 'latex');
xlabel('time [s]', 'Interpreter', 'latex');ylabel('f [N]', 'Interpreter', 'latex');
xlim([min(t) max(t)])
ylim([min(f) max(f)])

tau = figure('Renderer','painters','Position',[10 10 900 350]);
plot(t(index),M(1,index),'-g',t(index),M(2,index),'-r',t(index),M(3,index),'-b', 'LineWidth', 1.5);
set(gca, 'FontSize',12); grid on; title('\textbf{Torque}', 'FontSize', 16, 'Interpreter', 'latex');
legend('$M_{x}$','$M_{y}$','$M_{z}$','FontSize', 10,'Interpreter', 'latex', 'Location','east');%axis equal;
xlabel('time [s]', 'Interpreter', 'latex');ylabel('M [Nm]', 'Interpreter', 'latex');
xlim([min(t) max(t)])

if type_traj == 0
    exportgraphics(p, fullfile(folderPath, 'position_ellisse.pdf'));
    exportgraphics(a, fullfile(folderPath,'Trajectory_XYZ_ellisse.pdf'));
    exportgraphics(ep, fullfile(folderPath,'Load_Position_Error_ellisse.pdf'));
    exportgraphics(ev, fullfile(folderPath,'Load_Linear_Velocity_Error_ellisse.pdf'));
    exportgraphics(eq, fullfile(folderPath,'Load_Orientation_Error_ellisse.pdf'));
    exportgraphics(eom, fullfile(folderPath,'Load_Angular_Velocity_Error_ellisse.pdf'));
    exportgraphics(eR, fullfile(folderPath,'UAV_Orientation_Error_ellisse.pdf'));
    exportgraphics(eOm, fullfile(folderPath,'UAV_Angular_Velocity_Error_ellisse.pdf'));
    exportgraphics(uT, fullfile(folderPath,'Thrust_ellisse.pdf'));
    exportgraphics(tau, fullfile(folderPath,'Torque_ellisse.pdf'));

else
    exportgraphics(p, fullfile(folderPath, 'position_sinusoidale.pdf'));
    exportgraphics(a, fullfile(folderPath,'Trajectory_XYZ_sinusoidale.pdf'));
    exportgraphics(ep, fullfile(folderPath,'Load_Position_Error_sinusoidale.pdf'));
    exportgraphics(ev, fullfile(folderPath,'Load_Linear_Velocity_Error_sinusoidale.pdf'));
    exportgraphics(eq, fullfile(folderPath,'Load_Orientation_Error_sinusoidale.pdf'));
    exportgraphics(eom, fullfile(folderPath,'Load_Angular_Velocity_Error_sinusoidale.pdf'));
    exportgraphics(eR, fullfile(folderPath,'UAV_Orientation_Error_sinusoidale.pdf'));
    exportgraphics(eOm, fullfile(folderPath,'UAV_Angular_Velocity_Error_sinusoidale.pdf'));
    exportgraphics(uT, fullfile(folderPath,'Thrust_sinusoidale.pdf'));
    exportgraphics(tau, fullfile(folderPath,'Torque_sinusoidale.pdf'));
end


%% Animation
animation(t, x, t(index), xLd(index,:), type_traj);