%Carlos Alejandro Barrera Rodriguez
%Actividad #2
%Robotica Movil
clear all
close all
clc
%% Variables
t = 0.05;
S = 5;
N= S/t;
p = [0 0 0]';
wr=[2];
wl=[2];
L=0.15;
R=0.05;
%% Graficación
t_p = t:t:S;
p_plot = zeros(3,N);
pp_plot = zeros(3,N);
j=1;

for i=1:((S/t))

    pp = [0.5*R*cos(p(3)) 0; 0.5*R*sin(p(3)) 0;0 (R/(2*L))]*[(wr+wl);(wr-wl)];
    p = p + pp*t;

    p_plot(:,j) = p;
    pp_plot(:,j) = pp;
%% Ploteo de Graficas
    if mod(i,N) == 0
        figure(1)
        hold on
        grid on
        Dibujar_Diferencial(p,L)
        plot(p_plot(1,:),p_plot(2,:),'-k','LineWidth',1)
        
        figure(2)
        hold on
        grid on
        plot(t_p,p_plot(1,:),'LineWidth',1)
        plot(t_p,p_plot(2,:),'LineWidth',1)
        plot(t_p,p_plot(3,:),'LineWidth',1)
        xlabel('t'), ylabel('Posición')
        legend('x','y','theta')

        figure(3)
        hold on
        grid on
        plot(t_p,pp_plot(1,:),'LineWidth',1)
        plot(t_p,pp_plot(2,:),'LineWidth',1)
        plot(t_p,pp_plot(3,:),'LineWidth',1)
        xlabel('t'), ylabel('Velocidad')
        legend('x','y','theta')
    
        p = [0 0 0]';
        pp = [0 0 0]';
        p_plot = zeros(3,N);
        pp_plot = zeros(3,N);
        j = 0;
    end

    j = j+1;

end
    figure(1)
    sgtitle('Movimiento del Robot')
    figure(2)
    sgtitle('Grafica Posicion')
    figure(3)
    sgtitle('Grafica Velocidad')