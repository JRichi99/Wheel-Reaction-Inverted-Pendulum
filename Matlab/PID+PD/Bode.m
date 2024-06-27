planta = tf(-1,[0.029 0 -1.3381]);
PID = pid(-53.67,-15.657,-4.8635);
PD = pid(-55,0,-5.5);
sist_PID = series(PID, planta);
sist_PD = series(PD, planta);

% Graficar el Lugar Geométrico de las Raíces
figure;
bode(sist_PID);
hold on;
bode(sist_PD);

% Añadir título y etiquetas
title('Lugar Geométrico de las Raíces');
grid on;
legend("PID","PD")
hold off;
