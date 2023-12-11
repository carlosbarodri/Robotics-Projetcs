clear all
close all
clc
%% Variables y Declaraciones
p_plot = [];
t_plot = [];
v_plot = [];
x_plot = [];
xd_plot = [];
L = 0.471/2;
l = 0.3/2;
td = [1.3 0.9 pi/4]';
k = diag([9 9 9]);
bot = Bot_youBot_Platform();
S = 60;
%% Desarrollo
tic;
while toc<=S
    img = bot.Get_Image();
    p = bot.Get_Pose();
    ex = td(1)-p(1);
    ey = td(2)-p(2);
    et = td(3)-p(3);
    e = [ex;ey;et];
    Cin = [sqrt(2)*sin(p(3)+pi/4) -sqrt(2)*cos(p(3)+pi/4) -(L + l);
           sqrt(2)*cos(p(3)+pi/4) sqrt(2)*sin(p(3)+pi/4) (L + l);
           sqrt(2)*cos(p(3)+pi/4) sqrt(2)*sin(p(3)+pi/4) -(L + l);
           sqrt(2)*sin(p(3)+pi/4) -sqrt(2)*cos(p(3)+pi/4) (L + l)];
    
    v = Cin*k*e;

    bot.Set_Joint_Velocity(v);

    p_plot = [p_plot p];
    t_plot = [t_plot toc];
    v_plot= [v_plot v];
    x_plot = [x_plot p(1:2)];
    xd_plot = [xd_plot td(1:2)];
end

%% Graficas
figure
title('Trayectoria del robot')
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'c-','LineWidth',2)
plot(td(1), td(2), 'm*')
Dibujar_Omnidireccional_4(p,L,l)

figure
title('Velocidades')
grid on
hold on
plot(t_plot,v_plot(1,:),'b-','LineWidth',2)
plot(t_plot,v_plot(2,:),'y-','LineWidth',2)
plot(t_plot,v_plot(3,:),'k-','LineWidth',2)
plot(t_plot,v_plot(4,:),'c-','LineWidth',2)
legend('V_1', 'V_2', 'V_3', 'V_4')
xlabel('Tiempo')
ylabel('Velocidad')

figure
title('Trayectoria Real vs Trayectoria Deseada')
hold on
grid on
plot(t_plot, x_plot, 'c-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'k--', 'LineWidth', 2)
legend('x', 'y', 'x_d', 'y_d')

%%
bot.Stop_Simulation();
