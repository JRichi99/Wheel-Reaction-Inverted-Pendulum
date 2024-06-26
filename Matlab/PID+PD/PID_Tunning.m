clear
clc
planta = tf(-1,[0.029 0 -1.3381]);
controller = pid(-53.67,-15.657,-4.933);
pidTuner(planta,controller);
