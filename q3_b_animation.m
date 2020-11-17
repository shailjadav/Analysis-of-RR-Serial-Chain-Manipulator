% ME 639: Introduction to robotics
% Midsem exam : Question 3 (b)
%               3 Oct 2018
%   Repeat above part (a) when a point mass, m e = 1 , is added at the center of mass of the
%   link 2. Plot the results and discuss.
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%% ODE solver computed
 
[t,x]=ode45('ode_solver_script_q3_b',[0,10],[0,0,0,0]);  

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4);

%% Trejectory genration plannned
[~,q0,dq0,ddq0]=trecgen(0,1/10,10,0,0,pi/6,0);  %Trejectory generation for theta 1 0 to pi/6
[tq,q1,dq1,ddq1]=trecgen(0,1/10,10,0,0,pi/3,0);  %Trejectory generation for theta 2 0 to pi/3


%% Computed
figure('units','normalized','outerposition',[0 0 1 1])
c=1;
for i=1:2:length(th1)
    
theta1=th1(i,1);
theta2=th2(i,1); 


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
title(strcat('Time = ',num2str(t(i,1))),'Interpreter','latex')
xlabel('X axis (m)','Interpreter','latex')
ylabel('Y axis (m)','Interpreter','latex')
set(gca,'FontSize',18)
pause(0.0000000000000000001);
F(c) = getframe(gcf) ;
c=c+1;
end

% create the video writer with 30 fps
  writerObj = VideoWriter('Q3_computed_b.avi');
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


