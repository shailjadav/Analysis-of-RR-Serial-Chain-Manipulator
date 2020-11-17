% ME 639: Introduction to robotics
% Midsem exam : Question 1 (b)
%               3 Oct 2018
%b. Sinusoidal joint torques are applied, i.e., T 1 = T 2 = sin(t) . Plot joint positions and angular
%   velocities with respect to time. Discuss the results.
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%% ODE solver
 
[t,x]=ode45('ode_solver_script_q1_b',[0,10],[pi/4,0,pi/4,0]);  % Time span 0 to 10 IC=[pi/4 0 pi/4 0] Theta1=pi/4 Theta2=pi/4

m1=1; m2=1; l1=1; l2=1; g=9.81;

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4);

%% Display The Results

figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t,th1,'r','LineWidth',1.5)
title('Joint Position $\theta_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t,th2,'r','LineWidth',1.5)
title('Joint Position $\theta_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)
saveas(gcf,'Q1_b_JP.png')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t,dth1,'b','LineWidth',1.5)
title('Joint Velocity $\dot{\theta_1}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t,dth2,'b','LineWidth',1.5)
title('Joint Velocity $\dot{\theta_2}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)
saveas(gcf,'Q1_b_JV.png')
