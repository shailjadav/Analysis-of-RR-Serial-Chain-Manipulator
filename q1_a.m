% ME 639: Introduction to robotics
% Midsem exam : Question 1 (a)
%               3 Oct 2018
%a.  No joint torques are applied, i.e., T 1 = T 2 = 0 . Plot the kinetic, potential
%    and total energy of the system with respect to time. Discuss the results.
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%% ODE solver
 
[t,x]=ode45('ode_solver_script_q1_a',[0,10],[pi/4,0,pi/4,0]);  % Time span 0 to 10 IC=[pi/4 0 pi/4 0] Theta1=pi/4 Theta2=pi/4

m1=1; m2=1; l1=1; l2=1; g=9.81;

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4); %Joint position and velocities

%% Energy Calculation

for i=1:1:length(th1)

% Equation for kinetic energy
KE(i,1) = (0.5* (((m1/3) + m2)*l1*l1*dth1(i)*dth1(i))) + ((1/6)*m2*l2*l2*(dth1(i)^2 + dth2(i)^2 + 2*dth1(i)*dth2(i))) + (0.5*m2*l1*l2*cos(th2(i))*(dth1(i)^2 + dth1(i) *dth2(i)));

% Equation for potential energy
PE(i,1)=((0.5*m1 + m2)*g*l1*sin(th1(i)))+(0.5*m2*g*l2*sin(th1(i)+th2(i)));
end

%Total energy
TE=KE+PE;

%% Display The Results

%Ploting energies
figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t,KE,'r','LineWidth',1.5)
title("Kinetic Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy (J) ','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t,PE,'b','LineWidth',1.5)
title("Potential Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy  (J) ','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)
saveas(gcf,'Q1_a_KE_PE1.png')

figure('units','normalized','outerposition',[0 0 1 1])
plot(t,TE,'LineWidth',1.5)
title("Total Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy  (J)','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor
saveas(gcf,'Q1_a_TE1.png')