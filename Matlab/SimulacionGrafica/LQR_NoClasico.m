clc;
clear all;

%% Parameters
g = 9.81;
Mw = 0.55;
Mp = 0.14;
Jw = 4.36*10^(-3);
Jp = 0.69*10^(-3);
L = 0.22;
Lp = 0.11;
Jt = Mp*(Lp^(2)) + Mw*(L^(2)) + Jp;
Mt = (Lp*Mp+L*Mw)*g;
r = (L - Lp)/2;

%% Condiciones iniciales
theta_0 = deg2rad(90);
dtheta_0 = 0;
beta_0 = 0;
dbeta_0 = 0;
x0 = [theta_0; dtheta_0; beta_0; dbeta_0];

error_prev = 0;
integral_error = 0;

%% Matrices del sistema
A = [0, 1, 0, 0;
     Mt/Jt, 0, 0, 0;
     0, 0, 0, 1;
     0, 0, 1/Jw, 0];

B = [0;
     -1/Jt;
     0;
     1/Jw];

C = eye(4);
D = zeros(4,1);

%% Diseño del controlador de estado
Q = diag([100, 1, 100, 1]); % Matriz de pesos del estado
R = 0.1; % Peso de la entrada
K = lqr(A, B, Q, R);

%% Simulación
tspan = 0.01;
tiempo = 0;
cap = 25;

% Inicialización del gráfico
f1 = figure;
hold on;
grid on;
axis equal
xlabel('X (m)');
ylabel('Y (m)');
title('Reaction Wheel Inverted Pendulum')

[xw, yw] = PosCWheel(L, theta_0);
[xw_end, yw_end] = PosEWheel(r, beta_0, xw, yw);
pos_wheel = [xw-r, yw-r, r*2, r*2];

base = plot([-0.5, 0.5], [0, 0], 'k', 'LineWidth', 2);
radious = plot([xw, xw_end], [yw, yw_end], 'r', 'LineWidth', 1.5);
pendulum = plot([0, xw], [0, yw], 'b', 'LineWidth', 1.5);
wheel = rectangle('Position', pos_wheel, 'Curvature', [1 1], 'LineWidth', 1.5);

axis equal
xlim([-0.6 0.6]);
ylim([-0.4 0.6]);

texto_handle = text(0.05, 0.95, 'Time: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto2_handle = text(0.75, 0.95, 'Tin: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto3_handle = text(0.75, 0.85, 'error: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto4_handle = text(0.75, 0.75, 'error_acum: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto5_handle = text(0.75, 0.65, 'error_prev: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

cerrar = false;

while ~cerrar
    tiempo = tiempo + tspan;
    time_texto = ['Time: ', num2str(tiempo)];

    % Controlador de estado
    u = -K * x0; % Ley de control: u = -Kx
    u = max(min(u, cap), -cap); % Saturación del control
    tin_texto = ['Tin: ', num2str(u)];
    
    % Calcular error
    error = -x0(1); % Deseamos que el ángulo del péndulo sea 0
    error_texto = ['error: ', num2str(error)];
    integral_error = integral_error + error * tspan;
    error_ac_texto = ['error acum: ', num2str(integral_error)];
    derivative_error = (error - error_prev) / tspan;
    error_ant_texto = ['error prev: ', num2str(error_prev)];
    
    error_prev = error;

    % Solución de ODE
    [~, f] = ode45(@(t, x) stateSpaceODE(t, x, A, B, u), [0 tspan], x0);
    x = f(end, :)';
    theta = wrapToPi(x(1));
    beta = wrapToPi(x(3));

    % Calculo de nuevas posiciones
    [xw, yw] = PosCWheel(L, theta);
    [xw_end, yw_end] = PosEWheel(r, beta, xw, yw);
    pos_wheel = [xw-r, yw-r, r*2, r*2];

    % Actualizar condiciones iniciales
    x0 = x;

    % Dibujar
    set(texto_handle, 'String', time_texto);
    set(texto2_handle, 'String', tin_texto);
    set(texto3_handle, 'String', error_texto);
    set(texto4_handle, 'String', error_ac_texto);
    set(texto5_handle, 'String', error_ant_texto);
    set(pendulum, 'XData', [0, xw], 'YData', [0, yw]);
    set(radious, 'XData', [xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position', pos_wheel, 'Curvature', [1 1]);
    drawnow;

    pause(1/100);
end

%% Funciones auxiliares
function dxdt = stateSpaceODE(t, x, A, B, u)
    dxdt = A * x + B * u;
end

function [x_cwheel, y_cwheel] = PosCWheel(arm, theta)
    x_cwheel = -arm * sin(theta);
    y_cwheel = arm * cos(theta);
end

function [x_ewheel, y_ewheel] = PosEWheel(radius, beta, xc, yc)
    x_ewheel = -radius * sin(beta) + xc;
    y_ewheel = radius * cos(beta) + yc;
end
