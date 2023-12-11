%Carlos Alejandro Barrera Rodriguez
%Robotica Movil
%08/09/2023
close all;
clear all;
clc
%Variables y parametros
L = 0.25;
l = 0.25;
p = [0 0 0]';
pp = [0.2 0.2 0.8]';
t = 0.05;
s = 5;
N=s/t;
p_plot = zeros(3,N);
pp_plot = zeros(3,(N));
t_plot=t:t:s;

for i=1:(N)

    R=[cos(p(3)) -sin(p(3)) 0;sin(p(3))  cos(p(3)) 0;0 0 1];
    T=[1 -1 -(L+l);1  1  (L+l);1  1 -(L+l);1 -1  (L+l)];
    J = T*(R');

    %Cinematicas
    v = J*pp;
    pp = pinv(J)*v;


    p = p + pp*t;
    p_plot(:, i) = p;
    pp_plot(:, i) = pp;
end

figure()
hold on; 
grid on;
title('Posición')
xlabel('t');
ylabel('x, y, theta');

plot(t_plot, p_plot(1,:), '-', 'LineWidth',2);
plot(t_plot, p_plot(2,:), '-', 'LineWidth',2);
plot(t_plot, p_plot(3,:), '-', 'LineWidth',2);
legend('x', 'y', 'theta')

figure()
hold on;
grid on;
title('Velocidad')
xlabel('t');
ylabel('(x,y)');
plot(t_plot, pp_plot(1,:), '--', 'LineWidth',2);
plot(t_plot, pp_plot(2,:), ':', 'LineWidth',2);
plot(t_plot, pp_plot(3,:), '-.', 'LineWidth',2);
legend('x_pot', 'y_p', 'theta_p')

figure()
hold on;
grid on;
title('Trayectoria')
xlabel('x')
ylabel('y')

Dibujar_Omnidireccional_4(p, L, l);
plot(p_plot(1,:), p_plot(2,:), 'r-', 'LineWidth', 2)





