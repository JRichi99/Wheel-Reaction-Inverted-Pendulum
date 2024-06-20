planta = tf(-1,[0.029 0 -1.3381]);
controller = pid(-5.107,0,-0.48);
sist = series(controller,planta);
rlocus(sist);


% Graficar el Lugar Geométrico de las Raíces
figure;
rlocus(sist);
hold on;
% Polos objetivo problema 3
polo_objetivo1 = -8.322 + 7.791j;
polo_objetivo2 = -8.322 - 7.791j;
% Marcar los polos objetivo con una 'x'
plot(real(polo_objetivo1), imag(polo_objetivo1), 'rx', 'Markersize', 10,'LineWidth', 2);
plot(real(polo_objetivo2), imag(polo_objetivo2), 'rx', 'Markersize', 10,'LineWidth', 2);
% Añadir título y etiquetas
title('Lugar Geométrico de las Raíces');
xlabel('Real');
ylabel('Imaginario');
grid on;
hold off;
