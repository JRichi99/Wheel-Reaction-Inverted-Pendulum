% Lead-Lag Controller Parameters
K = -10; % Adjusted gain
T1 = 0.1; % Lead time constant
T2 = 0.5; % Lag time constant
planta_num = -1;
planta_dem = [0.029 0 -1.3381];
sist_num = conv(planta_num * K, [1 T1]);
sist_dem = conv(planta_dem, [1 T2]);
sist = tf(sist_num, sist_dem);
rlocus(sist);