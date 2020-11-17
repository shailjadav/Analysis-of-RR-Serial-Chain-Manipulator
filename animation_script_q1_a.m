% ME 639: Introduction to robotics
% Midsem exam : Question 1 D
%               3 Oct 2018
% Animate the 2R manipulator for part (a) and (b).
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%%
%syms theta1 theta2 theta3 l1 l2 l3   

[t,x]=ode45('ode_solver_script_q1_a',[0,10],[pi/4,0,pi/4,0]);

m1=1; m2=1; l1=1; l2=1; g=9.81;

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4);

%%

for i=1:1:length(th1)
    
KE(i,1) = (0.5* (((m1/3) + m2)*l1*l1*dth1(i)*dth1(i))) + ((1/6)*m2*l2*l2*(dth1(i)^2 + dth2(i)^2 + 2*dth1(i)*dth2(i))) + (0.5*m2*l1*l2*cos(th2(i))*(dth1(i)^2 + dth1(i) *dth2(i)));

PE(i,1)=((0.5*m1 + m2)*g*l1*sin(th1(i)))+(0.5*m2*g*l2*sin(th1(i)+th2(i)));
end

TE=KE+PE;


figure('units','normalized','outerposition',[0 0 1 1])
for i=1:1:length(x)
    
theta1=x(i,1);
dtheta1=x(i,2); 
theta2=x(i,3); 
dtheta2=x(i,4);

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
subplot(221)
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
title(strcat('Time = ',num2str(t(i,1))),'Interpreter','latex')
xlabel('X axis (m)','Interpreter','latex')
ylabel('Y axis (m)','Interpreter','latex')
set(gca,'FontSize',18)
pause(0.000000000000000000000000000000000000000001);

subplot(223)
plot(t(i,1),TE(i,1),'ok','LineWidth',1)
hold on
title("Total Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy  (J)','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor


subplot(222)
plot(t(i,1),KE(i,1),'or','LineWidth',1)
hold on
title("Kinetic Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy (J) ','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor

subplot(224)
plot(t(i,1),PE(i,1),'ob','LineWidth',1)
hold on
title("Potential Energy",'Interpreter','latex')
xlabel('Time (s)','Interpreter','latex')
ylabel('Energy  (J) ','Interpreter','latex')
ylim([-40 40])
set(gca,'FontSize',18)
grid minor
set(gca)
F(i) = getframe(gcf) ;
end

% create the video writer with 30 fps
  writerObj = VideoWriter('animation_Q1_a.avi');
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