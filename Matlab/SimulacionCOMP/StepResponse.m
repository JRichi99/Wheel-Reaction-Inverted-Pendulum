

% Lead-Lag Controller Parameters
K = -10; % Adjusted gain
T1 = 0.1; % Lead time constant
T2 = 0.5; % Lag time constant

planta_num = -1;
planta_dem = [0.029 0 -1.3381];
planta_adel_num = conv(planta_num * K, [1 T1]);
planta_adel_dem = conv(planta_dem, [1 T2]);
plant_adel = tf(planta_adel_num, planta_adel_dem);

sist=feedback(plant_adel,1);
stepplot(sist);
