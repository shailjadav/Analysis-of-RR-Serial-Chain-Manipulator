% ME 639: Introduction to robotics
% Midsem exam : Question 4 (b)
%               3 Oct 2018
%  Repeat above part (a) when a rotational spring, k r = 1 , is added at the joint 2 between
%  link 1 and 2. Plot the results and discuss, how does the value of k r change the results.
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%% ODE solver computed
 
[t,x]=ode45('ode_solver_script_q4_b',[0,10],[0,0,0,0]);  % Time span 0 to 10 IC=[pi/4 0 pi/4 0] Theta1=pi/4 Theta2=pi/4

m1=1; m2=1; l1=1; l2=1; g=9.81;

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4);

%% Trejectory genration plannned
[~,q0,dq0,ddq0]=trecgen(0,1/10,10,0,0,pi/6,0);  %Trejectory generation for theta 1 0 to pi/6
[tq,q1,dq1,ddq1]=trecgen(0,1/10,10,0,0,pi/3,0);  %Trejectory generation for theta 2 0 to pi/3



%% Display The Results

figure('units','normalized','outerposition',[0 0 1 1])
subplot(223)
plot(t,th1,'r','LineWidth',1.5)
title('Joint Position computed $\theta_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(224)
plot(t,th2,'r','LineWidth',1.5)
title('Joint Position computed $\theta_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)


subplot(221)
plot(tq,q0,'r','LineWidth',1.5)
title('Joint Position palnned $\theta_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(222)
plot(tq,q1,'r','LineWidth',1.5)
title('Joint Position palnned $\theta_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)
saveas(gcf,'Q4_b_JP.png')

figure('units','normalized','outerposition',[0 0 1 1])

subplot(223)
plot(t,dth1,'b','LineWidth',1.5)
title('Joint Velocity computed $\dot{\theta_1}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(224)
plot(t,dth2,'b','LineWidth',1.5)
title('Joint Velocity computed $\dot{\theta_2}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)




subplot(221)
plot(tq(1,1:end-1),dq0,'b','LineWidth',1.5)
title('Joint Velocity planned $\dot{\theta_1}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(222)
plot(tq(1,1:end-1),dq1,'b','LineWidth',1.5)
title('Joint Velocity planned $\dot{\theta_2}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
%ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)

saveas(gcf,'Q4_b_JV.png')
