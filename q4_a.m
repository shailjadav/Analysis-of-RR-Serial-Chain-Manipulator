% ME 639: Introduction to robotics
% Midsem exam : Question 4 (a)
%               3 Oct 2018
%  a. For T 1 as computed in Q2(b), solve Eq. (1) using a numerical solver, say ode45, over a
%  span of time, [t i t f ] = [0 10] . Compare the computed joint positions and velocities with
%  the desired trajectory as planned in Q2 (a). Plot the results and discuss.
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%% ODE solver computed
 
[t,x]=ode45('ode_solver_script_q4_a',[0,10],[0,0,0,0]);  
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
saveas(gcf,'Q4_a_JP.png')

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

saveas(gcf,'Q4_a_JV.png')
