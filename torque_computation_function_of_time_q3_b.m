%% Initialization
clear 
close all
clc
%% Trejectory genration 
[~,q0,dq0,ddq0]=trecgen(0,1/10,10,0,0,pi/6,0);  %Trejectory generation for theta 1 0 to pi/6
[t,q1,dq1,ddq1]=trecgen(0,1/10,10,0,0,pi/3,0);  %Trejectory generation for theta 2 0 to pi/3


x=[q0 dq0 q1 dq1];

%% Initialization of parameters
m2=2; m1=1; l1=1; l2=1; g=9.81;


%% Equation of motion 

for i=1:1:length(ddq0)
M11=((((m1/3) + m2)*l1^2) +((m2/3)*l2^2) + (m2*l1*l2*cos(q1(1,i))));
M12=(m2*(((l2^2)/3) + (0.5*l1*l2*cos(q1(1,i)))));
M21=M12;
M22=((1/3)*m2*l2*l2);


H1 =((-m2*l1*l2*sin(q1(1,i))*dq0(1,i)*dq1(1,i) - (0.5*m2*l1*l2*sin(q1(1,i))*dq1(1,i)*dq1(1,i))));
H2 = (0.5 * m2* l1*l2*sin(q1(1,i))*dq0(1,i)*dq0(1,i));

G1=( ((((0.5*m1) + m2)*l1*cos(q0(1,i))) + (0.5*m2*l2*cos(q0(1,i)+q1(1,i))))*g);
G2=0.5*m2*l2*cos(q0(1,i)+q1(1,i))*g;

Tau1(1,i)= M11*ddq0(1,i) + M12*ddq1(1,i) + H1 +G1;
Tau2(1,i)= M21*ddq0(1,i) + M22*ddq1(1,i) + H2 +G2;
end


%% Curve fitting and equation
f1= fit(t(1,1:end-2)',Tau1','poly6');
cf1=coeffvalues(f1);
f2= fit(t(1,1:end-2)',Tau2','poly6');
cf2=coeffvalues(f2);

figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t(1,1:end-2),Tau1,'ob','LineWidth',1)
hold on
plot(f1)
title('Joint Torque $\tau_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Torque (Nm) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t(1,1:end-2),Tau2,'ob','LineWidth',1)
hold on
plot(f2)
title('Joint Torque $\tau_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Torque (Nm) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor
saveas(gcf,'Q3_b_CT.png')
