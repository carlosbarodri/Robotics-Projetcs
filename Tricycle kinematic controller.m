close all
clear all
clc
%% Variables
alpha_sup = pi/2;
alpha_inf = -pi/2;
R = 0.05;
t = 0.05;
s = 5;
N = s/t;
d = 0.25;
alpha = 0;
p = [0 0 0]';
pp = [0 0 0]';


kv = 1.0;
k_alpha = 0.2;
td = [1.5 1.5]';
p_plot = zeros(3,N);
pp_plot = zeros(3,N);
t_plot = t:t:s;
x_plot = [];
xd_plot = [];

%% Desarrollo
for i=1:N
    cla
    ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    alpha = atan2(td(2)-p(2),td(1)-p(1))-p(3);

    if alpha>alpha_sup
        alpha = alpha_sup;
    end

    if alpha<alpha_inf
        alpha = alpha_inf;
    end 

    vs = kv*ev;

    pp(1) = vs*cos(alpha)*cos(p(3));
    pp(2) = vs*cos(alpha)*sin(p(3));
    pp(3) = (vs/d)*sin(alpha);

    p = p+pp*t; 

    Dibujar_Triciclo(p,alpha,d)
    drawnow

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;
    x_plot = [x_plot p(1:2)];
    xd_plot = [xd_plot td];
end


%% Graficas
figure  
title('Velocidades')
grid on
hold on
plot(t_plot,pp_plot(1,:),'r-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'k-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'c-','LineWidth',2)
legend('X.punto', 'Y.punto', 'Theta.punto')
xlabel('Tiempo')
ylabel('Velocidades')

figure 
title('Pose')
grid on
hold on
plot(t_plot,p_plot(1,:),'r-','LineWidth',2)
plot(t_plot,p_plot(2,:),'k-','LineWidth',2)
plot(t_plot,p_plot(3,:),'c-','LineWidth',2)
legend('X', 'Y', 'Theta')
xlabel('Tiempo')
ylabel('Pose')
 

figure
title('Trayectoria real vs deseada')
hold on
grid on 
plot(t_plot, x_plot, 'r-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'b--', 'LineWidth', 2)
legend('x', 'y', 'xd', 'yd')

figure
title('Trayectoria del Triciclo')
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'r-','LineWidth',2)
plot(td(1), td(2), 'c*')
Dibujar_Triciclo(p,alpha,d)
