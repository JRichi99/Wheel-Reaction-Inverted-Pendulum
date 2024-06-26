clear
clc
planta = tf(-1,[0.029 0 -1.3381]);
%controller = pid(-5.107*gain,0,-0.48*gain);
%% Opcion 1 
controller = pid(-53.67,-15.657,-4.68635);
%% Opcion 2
%controller = pid(-28.89,-8.427,-2.655);
sist = series(controller,planta);
sist_lc=feedback(sist,1);
stepplot(sist_lc,5)
