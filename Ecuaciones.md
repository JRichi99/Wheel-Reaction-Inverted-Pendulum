### 1. Variables
$\overline{q} = [\theta,\beta]$

### 2. Velocidades
$^0\vec{r}{G1} = L_p\hat{j}_1 \quad\wedge\quad ^0\vec{r}{G2} = L\hat{j}_1$\
$\vec{v}{G1} = L_p(\dot{\theta}\hat{k}_1\times\hat{j}_1) = -L_p\dot{\theta}\hat{i}_1 \quad\wedge\quad \vec{v}{G2} = -L\dot{\theta}\hat{i}_1$\
$\vec{w}{G1} = \dot{\theta}\hat{k}_1 \quad\wedge\quad \vec{w}{G2} = \dot{\beta}\hat{k}_1$

### 3. Energía
$E_K = E_T + E_R = \frac{1}{2}(M_pL_p^2\dot{\theta}^2 + M_wL^2\dot{\theta}^2 + J_p\dot{\theta}^2 + J_w\dot{\beta}^2)$\
$E_P = E_{G1} + E_{G2} = (L_pM_p + LM_w)\cos{\theta}g$

### 4. Lagrangeano
$\mathcal{L} = \frac{1}{2}(M_pL_p^2\dot{\theta}^2 + M_wL^2\dot{\theta}^2 + J_p\dot{\theta}^2 + J_w\dot{\beta}^2) - (L_pM_p + LM_w)\cos{\theta}g$

### 5. Euler-Lagrange
$\hat{\theta}:\quad \frac{\delta}{\delta t}\frac{\delta\mathcal{L}}{\delta\dot{\theta}} - \frac{\delta\mathcal{L}}{\delta\theta} = (M_pL_p^2+M_wL^2+J_p)\ddot{\theta} - g(L_pM_p+LM_w)\sin{\theta} = -\tau$\
$\hat{\beta}:\quad \frac{\delta}{\delta t}\frac{\delta\mathcal{L}}{\delta\dot{\beta}} - \frac{\delta\mathcal{L}}{\delta\beta} = J_w\ddot{\beta} = -\tau$

### 6. Función de Transferencia
$\Theta = \frac{-1}{J_ts^2-M_t}$\
$B = \frac{1}{J_ws^2}$
hola
### 7. Diseño del controlador PID
G_c(s) = K_P + \frac{K_I}{s} + K_Ds