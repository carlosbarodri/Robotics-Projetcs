%Carlos Alejandro Barrera Rodriguez
%Actividad #3
%Robotica Movil
clear all
close all
clc
%% Variables
t = 0.05;
S = 10;
N= S/t;
p = [0 0 0 0]';
pp =[0 0 0 0]';
Vs=0.3;
alpha=0;
Ws=-0.4;
d=0.3;
%% Graficación
t_p = t:t:S;
p_plot = zeros(4,N);
pp_plot = zeros(4,N);
j=1;
for i=1:N
%Traccion trasera
    % pp(1) = Vs*cos(p(3));
    % pp(2) = Vs*sin(p(3));
    % pp(3) = (Vs/d)*tan(p(4));
    % pp(4) = Ws;
    %Traccion delantera
    pp(1) = Vs*cos(p(4))*cos(p(3));
    pp(2) = Vs*cos(p(4))*sin(p(3));
    pp(3) = (Vs/d)*sin(p(4));
    pp(4) = Ws;
    % 
    p = p + pp*t;

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;
%% Ploteo de Graficas
    if mod(i,N) == 0
        figure(1)
        hold on
        grid on
        Dibujar_Coche(p,d)
        plot(p_plot(1,:),p_plot(2,:),'-k','LineWidth',1)
        
        figure(2)
        hold on
        grid on
        plot(t_p,p_plot(1,:),'LineWidth',1)
        plot(t_p,p_plot(2,:),'LineWidth',1)
        plot(t_p,p_plot(3,:),'LineWidth',1)
        plot(t_p,p_plot(4,:),'LineWidth',1)
        xlabel('t'), ylabel('Posición')
        legend('x','y','theta','alpha')

        figure(3)
        hold on
        grid on
        plot(t_p,pp_plot(1,:),'LineWidth',1)
        plot(t_p,pp_plot(2,:),'LineWidth',1)
        plot(t_p,pp_plot(3,:),'LineWidth',1)
        plot(t_p,pp_plot(4,:),'LineWidth',1)
        xlabel('t'), ylabel('Velocidad')
        legend('x','y','theta','alpha')
    
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