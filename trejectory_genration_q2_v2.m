%% Initialization
clear 
close all
clc
%% Trejectory genration 
[t,q0,dq0,ddq0]=trecgen(0,0.001,3,0,0,pi/4,0);  %Trejectory generation for theta 1 0 to pi/6
[t,q1,dq1,ddq1]=trecgen(0,1/10,10,0,0,pi/3,0);  %Trejectory generation for theta 2 0 to pi/3


%% Input parameters
m2=1; m1=1; l1=1; l2=1; g=9.81;

tau1= 0; %Input torque is zero
tau2=tau1; %second torque is the same as torque 1



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

%% Display generated trejectories
figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t,q0,'r','LineWidth',2)
title('Joint Position $\theta_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t,q1,'r','LineWidth',2)
title('Joint Position $\theta_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Position (rad) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor
ylim([0 1.2])
saveas(gcf,'Q2_a_JP.png')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t(1,1:end-1),dq0,'k','LineWidth',2)
title('Joint Velocity $\dot{\theta_1}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
set(gca,'FontSize',18)
ylim([0 0.17])
grid minor

subplot(212)
plot(t(1,1:end-1),dq1,'k','LineWidth',2)
title('Joint Velocity $\dot{\theta_2}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Velocity (rad/sec) ','Interpreter','latex')
set(gca,'FontSize',18)
ylim([0 0.17])
grid minor
saveas(gcf,'Q2_a_JV.png')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t(1,1:end-2),ddq0,'b','LineWidth',2)
title('Joint Acceleration $\ddot{\theta_1}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Acceleration $(rad/sec^2)$ ','Interpreter','latex')
set(gca,'FontSize',18)
ylim([-0.062 0.062])
grid minor

subplot(212)
plot(t(1,1:end-2),ddq1,'b','LineWidth',2)
title('Joint Acceleration $\ddot{\theta_2}$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Acceleration $(rad/sec^2)$ ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor
saveas(gcf,'Q2_a_JA.png')


%% Display computed torque
figure('units','normalized','outerposition',[0 0 1 1])
subplot(211)
plot(t(1,1:end-2),Tau1,'r','LineWidth',2)
title('Joint Torque $\tau_1$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Torque (Nm) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor


subplot(212)
plot(t(1,1:end-2),Tau2,'r','LineWidth',2)
title('Joint Torque $\tau_2$','Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Torque (Nm) ','Interpreter','latex')
set(gca,'FontSize',18)
grid minor
saveas(gcf,'Q2_b_CT.png')

%% Validation through animation
figure('units','normalized','outerposition',[0 0 1 1])
for i=1:1:length(q0)
    
theta1=q0(1,i);
theta2=q1(1,i); 


l1=1; %Input the l length
l2=1; %Input the l length



% Homogeneus transformation matrix
H01 = [cos(theta1) -sin(theta1) 0 l1*cos(theta1);sin(theta1) cos(theta1) 0 l1*sin(theta1);0 0 1 0;0 0 0 1]; %Frame 0 to 1 tranformation
H12 = [cos(theta2) -sin(theta2) 0 l2*cos(theta2);sin(theta2) cos(theta2) 0 l2*sin(theta2);0 0 1 0;0 0 0 1]; %Frame 1 to 2 tranformation


H02=H01*H12;      %Frame 0 to 2 tranformation     


O=[0,0];                   %Joint 1 position
P1=[H01(1,4) H01(2,4)];    %Joint 2 position
P2=[H02(1,4) H02(2,4)];    %Joint 3 position
    

Orn= atan2(H02(2,1),H02(1,1));  %Orientation of end effector
Orn=(Orn)*(180/pi);
%subplot(221)
plot(P1(1),P1(2),'ok','LineWidth',5)
hold on
plot(P2(1),P2(2),'om','LineWidth',5)
plot(0,0,'ok','LineWidth',10)
xlim([-2.5 2.5])
ylim([-2.5 2.5])
grid minor
plot([0 P1(1)], [0 P1(2)],'r','LineWidth',5)
plot([P1(1) P2(1)], [P1(2) P2(2)],'b','LineWidth',5)
hold off
title(strcat('Time = ',num2str(t(1,i))),'Interpreter','latex')
xlabel('X axis (m)','Interpreter','latex')
ylabel('Y axis (m)','Interpreter','latex')
set(gca,'FontSize',18)
pause(0.0000000000000000001);
F(i) = getframe(gcf) ;
end

% create the video writer with 30 fps
  writerObj = VideoWriter('animation_Q2_a.avi');
  writerObj.FrameRate = 30;
  % set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);