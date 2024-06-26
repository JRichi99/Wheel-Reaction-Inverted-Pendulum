

% Lead-Lag Controller Parameters
K = 10; % Adjusted gain
KP = -5.5;
KD = -0.55;

planta_num = -1;
planta_dem = [0.029 0 -1.3381];
sis_num = conv(planta_num, K*[KD KP]);
sis = tf(sis_num, planta_dem);

sist=feedback(sis,1);
stepplot(sist);
