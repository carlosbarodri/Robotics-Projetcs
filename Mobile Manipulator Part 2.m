close all 
clear all 
clc
%% Variables e Inicializaciones
p_base = [0 0 0]';
q_arm = [0 pi/4 -pi/4]';
P = [p_base;q_arm];
Pp = [0 0 0 0 0 0]';
td = [-1.0 1.0 0.5]';
s = 5;
t = 0.05;
N = s/t;
K = diag([1.5 1.5 1.5 0])';

%% Jacobiano de nuestro robot
Jacob = @(theta_b,theta_1,theta_2,theta_3)[1, 0, (sin(theta_1 + theta_b)*sin(theta_2)*sin(theta_3))/4 - (3*sin(theta_1 + theta_b)*cos(theta_2))/10 - (sin(theta_1 + theta_b)*cos(theta_2)*cos(theta_3))/4 - sin(theta_b)/4, -(sin(theta_1 + theta_b)*(5*cos(theta_2 + theta_3) + 6*cos(theta_2)))/20, -(cos(theta_1 + theta_b)*(5*sin(theta_2 + theta_3) + 6*sin(theta_2)))/20, -(cos(theta_1 + theta_b)*sin(theta_2 + theta_3))/4;...
                                           0, 1, cos(theta_b)/4 + (3*cos(theta_1 + theta_b)*cos(theta_2))/10 + (cos(theta_1 + theta_b)*cos(theta_2)*cos(theta_3))/4 - (cos(theta_1 + theta_b)*sin(theta_2)*sin(theta_3))/4,  (cos(theta_1 + theta_b)*(5*cos(theta_2 + theta_3) + 6*cos(theta_2)))/20, -(sin(theta_1 + theta_b)*(5*sin(theta_2 + theta_3) + 6*sin(theta_2)))/20, -(sin(theta_2 + theta_3)*sin(theta_1 + theta_b))/4;...
                                           0, 0,                                                                                                                                                                     0,                                                                        0,                           cos(theta_2 + theta_3)/4 + (3*cos(theta_2))/10,                           cos(theta_2 + theta_3)/4];

wTp = @(x_b,y_b,theta_b)[cos(theta_b) -sin(theta_b) 0 x_b; sin(theta_b) cos(theta_b) 0 y_b; 0 0 1 0; 0 0 0 1];
pTb = [1 0 0 0.25; 0 1 0 0; 0 0 1 0.25; 0 0 0 1];

T_0_1 = @(theta_1)[cos(theta_1) 0 sin(theta_1) 0; sin(theta_1) 0 -cos(theta_1) 0; 0 1 0 0.35; 0 0 0 1];

T_1_2 = @(theta_2)[cos(theta_2) -sin(theta_2) 0 0.3*cos(theta_2); sin(theta_2) cos(theta_2) 0 0.3*sin(theta_2); 0 0 1 0; 0 0 0 1];

T_2_3 = @(theta_3)[cos(theta_3) -sin(theta_3) 0 0.25*cos(theta_3); sin(theta_3) cos(theta_3) 0 0.25*sin(theta_3); 0 0 1 0; 0 0 0 1];

q_plot = [];
qp_plot = [];
x_plot = [];
xd_plot = [];
t_plot = [];
%% Proceso
for i=1:N
    
    bTe = T_0_1(q_arm(1))*T_1_2(q_arm(2))*T_2_3(q_arm(3));
    wTe = wTp(P(1), P(2), P(3))*pTb*bTe; 

    X = wTe(1:3,4); 
    e = td - X;
    
    J = Jacob(P(3), P(4), P(5), P(6));
   
    Jacob_monociclo = [J; sin(P(3)) -cos(P(3)) 0 0 0 0];

    Pp = pinv(Jacob_monociclo)*K*[e; 0];

    P = P+ [1 1 1 1 1 1]'.*Pp*t;

    p_base = P(1:3);
    q_arm = P(4:6);

    cla
    Dibujar_MM(p_base, q_arm)
    plot3(td(1), td(2), td(3), 'm*')
    drawnow

    q_plot = [q_plot P];
    qp_plot = [qp_plot Pp];
    x_plot = [x_plot wTe(1:3,4)];
    xd_plot = [xd_plot td];
    t_plot = [t_plot i*t];
end
%% Graficas
figure 
hold on
grid on
title('Grafica del Vector de Coordenadas Generalizadas')
plot(t_plot, q_plot, '-', 'LineWidth', 2)
legend('x_b', 'y_b', 'theta base', 'theta_1', 'theta_2', 'theta_3')
figure
hold on 
grid on
title('Grafica de Velocidades')
plot(t_plot, qp_plot, '-', 'LineWidth', 2)
legend('x_b', 'y_b', 'theta_b', 'theta_1', 'theta_2', 'theta_3')
figure
hold on
grid on
title('Grafica de Trayectorias vs Trayectorias Deseadas del actuador final')
plot(t_plot, x_plot, 'r-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'b--', 'LineWidth',2)
legend('x', 'y', 'z', 'x deseada', 'y deseada', 'z deseada')