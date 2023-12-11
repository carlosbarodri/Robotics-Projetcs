clear all
close all
clc

%% Variables
R = 0.100/2;
L = 0.300/2;
xd = -0.5;
yd = 1;
k = diag([1.5 1.51]);
D = 0.05;
p_plot = [];
t_plot = [];
q_plot = [];
x_plot = [];
bot = Bot_Pioneer();

S = 30;
%% Desarrollo
tic;
while toc<=S
    p = bot.Get_Pose();

    xp = p(1) + D*cos(p(3));
    yp = p(2) + D*sin(p(3));

    ex = xd - xp;
    ey = yd - yp;

    T = [cos(p(3)) -D*sin(p(3));
         sin(p(3)) D*cos(p(3))];

    qp = inv(T)*k*[ex ey]';

    wr = (2*qp(1)+qp(2)*L)/(2*R);
    wl = (2*qp(1)-qp(2)*L)/(2*R);
    
    bot.Set_Joint_Velocity([wr wl]');
    disp(p')

    p_plot = [p_plot [xp yp]'];
    t_plot = [t_plot toc];
    q_plot = [q_plot [wr wl]'];
    x_plot = [x_plot [xd yd]'];
end

%% Graficas
figure(1);
title('Trayectoria')
hold on
grid on
plot(p_plot(1,:), p_plot(2,:), 'r', 'LineWidth',2)
Dibujar_Diferencial(p,L)
xlabel('x')
ylabel('y')
figure(2);
title('Posición')
hold on
grid on
plot(t_plot, p_plot(1,:),'r','LineWidth',2)
plot(t_plot, p_plot(2,:),'b','LineWidth',2)
plot(t_plot, x_plot(1,:), 'c-', 'LineWidth',2)
plot(t_plot, x_plot(2,:), 'g-', 'LineWidth',2)
ylabel('Posición (m, rad)')
xlabel('Tiempo (s)')
legend('x_p', 'y_p', 'x_d', 'y_d')
figure(3);
title('Acción de control')
hold on
grid on
plot(t_plot, q_plot, '-', 'LineWidth',2)
ylabel('Velocidad')
xlabel('Tiempo')
legend('wr','wl')

%% Terminar simulacion
bot.Stop_Simulation();