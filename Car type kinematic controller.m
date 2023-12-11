clear all
close all
clc
%% Variables
t = 0.01;
s = 10;
N = s/t;
d = 0.3;
p = [0 0 pi 0]';
pp = [0 0 0 0]';

Kv = 1.1;
K_alpha = 0.4;

td = [-1.5 1.5]';

p_plot = zeros(4,N);
pp_plot = zeros(4,N);
t_plot = t:t:s;
x_plot = [];
xd_plot = [];
%% Desarrollo 
for i=1:N
    %% Traccion Trasera
    cla
    ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    ew = atan2(td(2)-p(2),td(1)-p(1))-(p(3)+p(4));
    ew = atan2(sin(ew),cos(ew));

    vs = Kv*ev;
    ws = K_alpha*ew;

    pp(1) = vs*cos(p(3));
    pp(2) = vs*sin(p(3));
    pp(3) = (vs/d)*tan(p(4));
    pp(4) = ws;

    p = p+pp*t; 

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;
    x_plot = [x_plot p(1:2)];
    xd_plot = [xd_plot td];
    %% Traccion Delantera
    % cla
    % ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    % ew = atan2(td(2)-p(2),td(1)-p(1))-(p(3)+p(4));
    % ew = atan2(sin(ew),cos(ew));
    % 
    % vs = Kv*ev;
    % ws = K_alpha*ew;
    % 
    % pp(1) = vs*cos(p(4))*cos(p(3));
    % pp(2) = vs*cos(p(4))*sin(p(3));
    % pp(3) = (vs/d)*sin(p(4));
    % pp(4) = ws;
    % 
    % p = p+pp*t; 
    % p_plot(:,i) = p;
    % pp_plot(:,i) = pp;
    % x_plot = [x_plot p(1:2)];
    % xd_plot = [xd_plot td];
end


%% Graficas
figure 
title('Velocidades')
grid on
hold on
plot(t_plot,pp_plot(1,:),'r-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'k-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'c-','LineWidth',2)
plot(t_plot,pp_plot(4,:),'y-','LineWidth',2)
legend('X.punto', 'Y.punto', 'Theta.punto', 'Alpha.punto')
xlabel('Tiempo')
ylabel('Velocidades')


figure 
title('Pose')
grid on
hold on
plot(t_plot,p_plot(1,:),'r-','LineWidth',2)
plot(t_plot,p_plot(2,:),'k-','LineWidth',2)
plot(t_plot,p_plot(3,:),'c-','LineWidth',2)
plot(t_plot,p_plot(4,:),'y-','LineWidth',2)
legend('X', 'Y', 'Theta', 'Alpha')
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
title('Velocidades')
grid on
hold on
plot(t_plot,pp_plot(1,:),'r-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'k-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'c-','LineWidth',2)
plot(t_plot,pp_plot(4,:),'y-','LineWidth',2)
legend('X.punto', 'Y.punto', 'Theta.punto', 'Alpha.punto')
xlabel('Tiempo')
ylabel('Velocidades')
