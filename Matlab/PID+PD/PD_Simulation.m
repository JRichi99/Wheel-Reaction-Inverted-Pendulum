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

%% Lead-Lag Controller Parameters

K = 1.26;
KP = -5.5;
KD = -0.55;

%% Condiciones iniciales

theta_0 = deg2rad(90);
tin = 0;

dtheta_0 = 0;
beta_0 = 0;
dbeta_0 = 0;
vtheta_0 = [theta_0 dtheta_0];
vbeta_0 =[beta_0 dbeta_0];

error_prev = 0;

%% Seteo inicial del grafico

f1 = figure;
hold on;
grid on;

axis equal
xlabel('X (m)');
ylabel('Y (m)');
title('Reaction Wheel Inverted Pendulum')

[xw,yw] = PosCWheel(L,theta_0);
[xw_end,yw_end] = PosEWheel(r,beta_0, xw, yw);
pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]

base = plot([-0.5, 0.5],[0, 0],'k','LineWidth',2); % base line

radious = plot([xw, xw_end],[yw, yw_end],'r','LineWidth',1.5); 
pendulum = plot([0, xw],[0, yw],'b','LineWidth',1.5); 
wheel = rectangle('Position',pos_wheel,'Curvature',[1 1], 'LineWidth',1.5);

axis equal
axis(gca,'equal');
xlim([-0.6 0.6]);
ylim([-0.4 0.6]);

texto_handle = text(0.05, 0.95, 'Time: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto2_handle = text(0.75, 0.95, 'Tin: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto3_handle = text(0.75, 0.85, 'error: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
texto5_handle = text(0.75, 0.75, 'error prev: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

tspan = 0.01;
tiempo = 0;

cerrar = false;
cap = 100;

while ~cerrar
    % Tiempo
    tiempo = tiempo + tspan;
    time_texto = ['Time: ', num2str(tiempo)];
    if tiempo < 3
        % Dibujar
        set(texto_handle, 'String', time_texto);  
        set(pendulum,'XData',[0, xw],'YData',[0, yw]);
        set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
        set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
        drawnow;
        continue
    end

    % Lead-Lag Controller
    error = 0-vtheta_0(1); % Desired angle of the pendulum is 0
    error_texto = ['error: ', num2str(error)];
    derivative_error = (error - error_prev) / tspan;
    error_ant_texto = ['error prev: ', num2str(derivative_error)];
    
    % Lead-Lag control action
    tin = K*(KD*derivative_error + KP*error);
    tin = max(min(tin, cap), -cap);
    tin_texto = ['Tin: ', num2str(tin)];

    error_prev = error;

    % Obtencion de variables
    [~,ftheta] = ode45(@(t,y) pendulumODE(t,y,Jt,Mt,tin),[0 tspan], vtheta_0);
    [~,fbeta] = ode45(@(t,y) wheelODE(t,y,Jw,tin),[0 tspan], vbeta_0);
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

    % Dibujar
    set(texto_handle, 'String', time_texto);
    set(texto2_handle,'String', tin_texto);
    set(texto3_handle,'String', error_texto);
    set(texto5_handle,'String', error_ant_texto);

    set(pendulum,'XData',[0, xw],'YData',[0, yw]);
    set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
    drawnow;

    pause(1/100);
end


function [x_cwheel, y_cwheel] = PosCWheel(arm,theta)
    x_cwheel = -arm*sin(theta); 
    y_cwheel = arm*cos(theta);
end 

function [x_ewheel, y_ewheel] = PosEWheel(radius,beta, xc, yc)
    x_ewheel = -radius*sin(beta) + xc; 
    y_ewheel = radius*cos(beta) + yc; 
end 

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
