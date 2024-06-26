% PD Controller Parameters
KP = -5.5;
KD = -0.55;
planta_num = -1;
planta_dem = [0.029 0 -1.3381];

planta = tf(planta_num, planta_dem);
%rlocus(planta)

sist_num = conv(planta_num, [KD KP]);
sist = tf(sist_num, planta_dem);
rlocus(sist);