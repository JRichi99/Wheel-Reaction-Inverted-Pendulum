% main
clc;
clear all;

%% Definicion de parametros

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

% Parametros del controlador
Kp1= -53.67;  % Opciones -55 | -53.67
Ki1= -15.657; % Opciones 0 | -15.657
Kd1= -4.68635; % Opciones -2.2 | -4.68635
cap = 50; % Limite de la accion del controlador
agp = 1;
agi = 1;
agd = 1;

% Condiciones iniciales del sistema
theta_0 = deg2rad(180);
ref = deg2rad(0);

tin = 0;

dtheta_0 = 0;
beta_0 = 0;
dbeta_0 = 0;
vtheta_0 = [theta_0 dtheta_0];
vbeta_0 =[beta_0 dbeta_0];

% Almacenar variables en el tiepo

ref_values = [ref];
error_values = [ref-theta_0];
theta_values = [theta_0];
tc_values = [0];
pert_values = [0];
time_values = [0];

% Condiciones iniciales del error
error_prev = 0;
integral_error = 0;

%% Initialize Graphic
f1 = figure;
subplot(2,2,1);  % Create subplot for drawing
hold on;
grid on;

axis equal
xlabel('X (m)');
ylabel('Y (m)');
title('Reaction Wheel Inverted Pendulum')

% Obtener las posiciones iniciales de los elementos
[xw,yw] = PosCWheel(L,theta_0);
[xw_end,yw_end] = PosEWheel(r,beta_0, xw, yw);
pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]

% Dibujar las posiciones iniciales de los elemenos
base = plot([-0.5, 0.5],[0, 0],'k','LineWidth',2); % base line
radious = plot([xw, xw_end],[yw, yw_end],'r','LineWidth',1.5); 
pendulum = plot([0, xw],[0, yw],'b','LineWidth',1.5); 
wheel = rectangle('Position',pos_wheel,'Curvature',[1 1], 'LineWidth',1.5);

axis equal
axis(gca,'equal');
xlim([-0.6 0.6]);
ylim([-0.3 0.6]);

% Mostrar el tiempo de la simulacion
texto_handle = text(0.05, 0.95, 'Time: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

%% Crear los graficos para cada cosa.
subplot(2,2,2); 
hold on;
theta_plot = plot(time_values, theta_values, 'b');
ref_plot = plot(time_values, ref_values, 'Color', [0.1725 0.4784 0.1059], 'LineStyle','--');
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Theta vs Time');
grid on;

subplot(2,2,3);
hold on;
error_plot = plot(time_values, error_values, 'Color', [0.6350 0.0780 0.1840]);
xlabel('Time (s)');
ylabel('Error (rad)');
title('Error vs Time');
grid on;

subplot(2,2,4);
hold on;
tc_plot = plot(time_values, tc_values, 'Color', [0.8500 0.3250 0.0980]);
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Control Signal vs Time');
grid on;


%% Parametros de la simulacion

tspan = 0.01; % Discretizacion del tiempo
tiempo = 0; % Tiempo inicial

%% Parametros del ruido
Wp = 50; % Frecuencia de la onda sinusoidal
Psin = 1; % Ganancia onda sinusoidal
Pimp = 0; % Magnitud impulso 



while tiempo < 5
    %% Tiempo
    tiempo = tiempo + tspan;
    time_texto = ['Time: ', num2str(tiempo)];

    % Esperar un poco antes de comenzar la simulacion
    if tiempo < 0.05
        % Dibujar
        set(texto_handle, 'String', time_texto);  
        set(pendulum,'XData',[0, xw],'YData',[0, yw]);
        set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
        set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
        drawnow;
        continue
    end

    %% Controlador PID

    % Calculo de los errores
    error = ref-vtheta_0(1); % Deseamos que el ángulo del péndulo sea 0
    integral_error = integral_error + error * tspan;
    derivative_error = (error - error_prev) / tspan;
    
    % Determinacion de accion de control
    tc = (Kp1*agp * error + Ki1*agi * integral_error + Kd1*agd * derivative_error);
    tc = max(min(tc, cap), -cap);
    
    % Actualizacion error previo
    error_prev = error;

    % Determinar perturbaciones impulso aleatorias
    random_number = rand;
    timp = 0;

    if random_number < 0.02
        timp = Pimp;
    end 
    
    if random_number > 0.98
        timp = Pimp *-1;
    end
    
    % Calculo de las perturbaciones
    tp = timp + sin(Wp*tiempo)*Psin;

    % Calculo de la entrada al sistema
    tin = tc + tp;

    % Integracion discreta de las variables dinamicas
    [~,ftheta] = ode45(@(t,y) pendulumODE(t,y,Jt,Mt,tin),[0 tspan], vtheta_0);
    [~,fbeta] = ode45(@(t,y) wheelODE(t,y,Jw,tin),[0 tspan], vbeta_0);
    
    % Obtencion de las variables de interes
    vtheta = ftheta(end, :);
    vbeta = fbeta(end, :);
    theta = wrapToPi(vtheta(1));
    beta = wrapToPi(vbeta(1));

    % Calculo de nuevas posiciones:
    [xw,yw] = PosCWheel(L,theta);
    [xw_end,yw_end] = PosEWheel(r,beta, xw, yw);
    pos_wheel = [xw-r, yw-r, r*2, r*2]; 
    
    % Actualizar condiciones iniciales
    vtheta_0 = vtheta;
    vbeta_0 = vbeta;
    
    % Guardar variables de interes
    ref_values = [ref_values,ref];
    error_values = [error_values, error_prev];
    theta_values = [theta_values, vtheta_0(1)];
    tc_values = [tc_values, tc];
    time_values = [time_values, tiempo];
    pert_values = [pert_values, tp];

    % Update theta values for the plot

    set(theta_plot, 'XData', time_values, 'YData', theta_values);
    set(ref_plot, 'XData', time_values, 'YData', ref_values);
    set(error_plot, 'XData', time_values, 'YData', error_values);
    set(tc_plot, 'XData', time_values, 'YData', tc_values);

    % Dibujar
    set(texto_handle, 'String', time_texto);
    set(pendulum,'XData',[0, xw],'YData',[0, yw]);
    set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
    drawnow;

    pause(1/100);
end

% Mostrar graficos globales.
figure(2);
plot(time_values, pert_values);
title('perturbacion vs tiempo');

%% Funciones que permiten calcular la posicion de los elementos del sistema. 
function [x_cwheel, y_cwheel] = PosCWheel(arm,theta)
    x_cwheel = -arm*sin(theta); 
    y_cwheel = arm*cos(theta);
end 

function [x_ewheel, y_ewheel] = PosEWheel(radius,beta, xc, yc)
    x_ewheel = -radius*sin(beta) + xc; 
    y_ewheel = radius*cos(beta) + yc; 
end 

%% Estas son las funciones diferenciales que luego se integran. 
function dthetadt = pendulumODE(t, y, Jt, Mt, tin)
    dthetadt = zeros(2,1);
    dthetadt(1) = y(2);
    dthetadt(2) = (-tin/Jt) + (Mt/Jt)*sin(y(1));
end
function dbetadt = wheelODE(t,y,Jw,tin)
    dbetadt = zeros(2,1);
    dbetadt(1) = y(2);
    dbetadt(2) = tin/Jw;
end