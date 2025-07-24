close all;
clear;
clc;

disp('Generazione delle traiettorie desiderate per diverse frequenze...');

%% --- 1. PARAMETRI FISICI ---
mL = 0.1;
g = 9.81;
e3 = [0;0;1];
l = 1.0;

%% --- 2. PARAMETRI DELLA TRAIETTORIA BASE ---
Ax_base = 0.5; % Amplitude X (m)
Ay_base = 0.8; % Amplitude Y (m)
Az_base = 1;   % Offset Z (m)

%% --- 3. Definisci le Frequenze (o Periodi T0) da Plottare ---

T0_values = [10,9,8,7,6,5,4,3,2.8,2.7,2.5,2.3,2,1.9,1.8,1.7,1.6,1.5,1.4,1.3,1.2,1.1,1]; % Periodi in secondi

% --- Tempo di Plotting ---
% Plottiamo per un singolo ciclo del periodo più lungo
t_plot_duration = max(T0_values); % Plot per la durata del periodo più lungo
t_plot_step = 0.01; % Passo temporale per il plotting (per avere curve lisce)
t_plot_vector = 0:t_plot_step:t_plot_duration + t_plot_step/2; % +step/2 per includere l'ultimo punto

%% --- 4. Preparazione Figura per i 4 Subplot (come nel paper) ---
fig_handle = figure('WindowState', 'maximized','Renderer','painters','Position',[10 10 900 700]);

% Layout per i 4 subplot (a) (b) (c) (d)
subplot(2,2,1); ax_a = gca; hold on; grid on; view(ax_a,172,20);
xlabel('x [m]','Interpreter', 'latex'); ylabel('y [m]','Interpreter', 'latex'); zlabel('z [m]','Interpreter', 'latex'); title('\textbf{3D View}','Interpreter', 'latex');

subplot(2,2,2); ax_b = gca; hold on; grid on; view(2); % Vista dall'alto (XY)
xlabel('x [m]','Interpreter', 'latex'); ylabel('y [m]','Interpreter', 'latex'); title('\textbf{XY View}','Interpreter', 'latex');

subplot(2,2,3); ax_c = gca; hold on; grid on;  view(180,-90); % Vista laterale (XZ)
xlabel('x [m]','Interpreter', 'latex'); ylabel('z [m]','Interpreter', 'latex'); title('\textbf{XZ View}','Interpreter', 'latex');

subplot(2,2,4); ax_d = gca; hold on; grid on;  view(0,90); % Vista laterale (YZ)
xlabel('y [m]','Interpreter', 'latex'); ylabel('z [m]','Interpreter', 'latex'); title('\textbf{YZ View}','Interpreter', 'latex');

sgtitle({'Geometric Control and Differential Flatness','of a Quadrotor UAV with a Cable-Suspended Load'},'FontSize', 16, 'Interpreter', 'latex');

%% --- 5. Loop per Ogni Frequenza (Calcola e Plotta) ---
disp('Generazione e plotting delle traiettorie...');

for idx = 1:length(T0_values)
    current_T0 = T0_values(idx);
    current_w = 2*pi / current_T0;

    % Vettori per memorizzare le traiettorie per questa frequenza
    xLd_plot_curr = zeros(length(t_plot_vector), 3);
    xQd_plot_curr = zeros(length(t_plot_vector), 3);

    for k = 1:length(t_plot_vector)
        t_instant = t_plot_vector(k);

        current_load_traj.xL = [Ax_base*sin(current_w*t_instant); Ay_base*cos(current_w*t_instant); Az_base];
        current_load_traj.dxL = [Ax_base*current_w*cos(current_w*t_instant); -Ay_base*current_w*sin(current_w*t_instant); 0];
        current_load_traj.d2xL = [Ax_base*(-current_w^2)*sin(current_w*t_instant); -Ay_base*(current_w^2)*cos(current_w*t_instant); 0];
        current_load_traj.d3xL = [Ax_base*(-current_w^3)*cos(current_w*t_instant); Ay_base*(current_w^3)*sin(current_w*t_instant); 0];
        current_load_traj.d4xL = [Ax_base*(current_w^4)*sin(current_w*t_instant); Ay_base*(current_w^4)*cos(current_w*t_instant); 0];
        current_load_traj.d5xL = [Ax_base*(current_w^5)*cos(current_w*t_instant); -Ay_base*(current_w^5)*sin(current_w*t_instant); 0];
        current_load_traj.d6xL = [Ax_base*(-current_w^6)*sin(current_w*t_instant); -Ay_base*(-current_w^6)*cos(current_w*t_instant); 0];

        xLd = current_load_traj.xL;
        aLd = current_load_traj.d2xL;
        
        % Tension Cable and Directional Vector
        Tq = -mL*(aLd + g*e3) ;
        norm_Tp = norm(Tq);
        q = Tq / norm_Tp;
        xQ = xLd - l*q;

        xLd_plot_curr(k,:) = xLd';
        xQd_plot_curr(k,:) = xQ';
    end

    % --- Plot delle Traiettorie Desiderate per la Frequenza Corrente ---
    
    % Subplot (a) - 3D View
    plot3(ax_a, xLd_plot_curr(:,1), xLd_plot_curr(:,2), xLd_plot_curr(:,3), 'Color', [0 0 1], 'LineWidth', 2); % Load (Blue, thick)
    plot3(ax_a, xQd_plot_curr(:,1), xQd_plot_curr(:,2), xQd_plot_curr(:,3), 'Color', [1 0 0], 'LineStyle', '-', 'LineWidth', 1); % Drone (Red, thin)
    set(ax_a, 'XLim',[-0.5 0.5] ,'XTick', -0.5:0.1:0.5, 'YLim', [-1 1], 'ZLim', [1 2]); 
    % Subplot (b) - XY View
    plot(ax_b, xLd_plot_curr(:,1), xLd_plot_curr(:,2), 'Color', [0 0 1], 'LineWidth', 2); % Load (Blue, thick)
    plot(ax_b, xQd_plot_curr(:,1), xQd_plot_curr(:,2), 'Color', [1 0 0], 'LineStyle', '-', 'LineWidth', 1); % Drone (Red, thin)
    set(ax_b, 'XLim', [-0.5 0.5], 'YLim', [-1 1]);
    % Subplot (c) - XZ View
    plot(ax_c, xLd_plot_curr(:,1), xLd_plot_curr(:,3), 'Color', [0 0 1], 'LineWidth', 2); % Load (Blue, thick)
    plot(ax_c, xQd_plot_curr(:,1), xQd_plot_curr(:,3), 'Color', [1 0 0], 'LineStyle', '-', 'LineWidth', 1); % Drone (Red, thin)
    set(ax_c, 'XLim', [-0.5 0.5], 'YLim', [1 2]);
    % Subplot (d) - YZ View
    plot(ax_d, xLd_plot_curr(:,2), xLd_plot_curr(:,3), 'Color', [0 0 1], 'LineWidth', 2); % Load (Blue, thick)
    plot(ax_d, xQd_plot_curr(:,2), xQd_plot_curr(:,3), 'Color', [1 0 0], 'LineStyle', '-', 'LineWidth', 1); % Drone (Red, thin)
    set(ax_d, 'XLim', [-1 1], 'YLim', [1 2]);
    
end
exportgraphics(fig_handle, 'Plot/sensitivity_to_load_freq.pdf');
disp('Plotting completato.');