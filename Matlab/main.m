%% main

%% Parameters
clc;
clear all;

Mw = 0.55;
Mp = 0.14;
Jw = 4.36*10^(-3);
Jp = 0.69*10^(-3);
L = 0.22;
Lp = 0.11;


theta_Q = 15*pi/180;
beta_Q = 0;

Fs = 5;
dt = 1/Fs;
N = 50;
t = dt*(0:N-1);

theta_0 = 0;
beta_0 = 0;
%%
f1 = figure;
hold on;

axis equal
xlabel('X (m)');
ylabel('Y (m)');
title('Reaction Wheel Inverted Pendulum')
grid on;

r = (L - Lp)/2;

xw = L*sin(theta_0); %wheel x center
yw = L*cos(theta_0); %wheel y center

xw_end = xw +  r*sin(beta_0); %wheel x end
yw_end = yw + r*cos(beta_0); %wheel y end

pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]


base = plot([-0.5, 0.5],[0, 0],'k','LineWidth',2); % base line
hold on;
grid on;
radious = plot([xw, xw_end],[yw, yw_end],'r','LineWidth',1.5); % Pendulum rod
pendulum = plot([0, xw],[0, yw],'b','LineWidth',1.5); % Pendulum rod
wheel = rectangle('Position',pos_wheel,'Curvature',[1 1], 'LineWidth',1.5);
axis equal
axis(gca,'equal');
xlim([-0.6 0.6]);
ylim([-0.1 0.6]);
set(gcf,'Position',[10 900 800 400])

%%

angles = [0:1:45,44:-1:-45,-44:1:45, 44:-1:-45,-44:1:45];

for k=1:length(angles)
    %clf(f1);
    angle = angles(k);
    theta = -angle*pi/180; 
    beta = -3*angle*pi/180;

    r = (L - Lp)/2;

    xw = L*sin(theta); %wheel x center
    yw = L*cos(theta); %wheel y center

    xw_end = xw +  r*sin(beta); %wheel x end
    yw_end = yw + r*cos(beta); %wheel y end

    pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]


    set(pendulum,'XData',[0, xw],'YData',[0, yw]);
    set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
    drawnow;
    % pause(1/1000);
end