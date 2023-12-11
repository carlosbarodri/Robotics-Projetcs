clear all
close all
clc

%% Variables
R = 0.100/2;
L = 0.300/2;
p_plot = [];
t_plot = [];
q_plot=[];
x_plot=[];
bot = Bot_Pioneer();
S = 37;
xd = 0.5;
yd = 1.0;
kv = 0.2;
kw = 1.5;
tic;

%% Desarrollo
while toc<=S
    p = bot.Get_Pose();

    ev = sqrt((xd-p(1))^2+(yd-p(2))^2);
    th = atan2(yd-p(2),xd-p(1));
    ew = th-p(3);
    ew = atan2(sin(ew),cos(ew));

    v = kv*ev;
    w = kw*ew;

    wr = (2*v+w*L)/(2*R);
    wl = (2*v-w*L)/(2*R);
    
    bot.Set_Joint_Velocity([wr wl]');
    disp(p')

    p_plot = [p_plot p];
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
plot(t_plot, x_plot(1,:), 'c--', 'LineWidth',2)
plot(t_plot, x_plot(2,:), 'g--', 'LineWidth',2)
ylabel('Posición (m, rad)')
xlabel('Tiempo (s)')
legend('x_p', 'y_p', 'x_d', 'y_d')

figure(3);
title('Acción de control')
hold on
grid on
plot(t_plot, q_plot, '-', 'LineWidth',2)
ylabel('Velocidad (m/s, rad/s)')
xlabel('Tiempo (s)')
legend('wr','wl')
figure
title('Coordendas')
hold on
grid on
plot(t_plot,p_plot(1,:),'r','LineWidth',2)
plot(t_plot,p_plot(2,:),'b','LineWidth',2)
plot(t_plot,p_plot(3,:),'c','LineWidth',2)
ylabel('m, rad')
xlabel('t')
legend('x','y','\theta','Location','best')

%% Terminar simulacion
bot.Stop_Simulation();