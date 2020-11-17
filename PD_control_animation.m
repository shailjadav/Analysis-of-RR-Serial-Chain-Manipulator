% ME 639: Introduction to robotics
% Midsem exam : PID Control
%               18 Oct 2018
%
% Author: Shail Jadav 18310039
%% Initialization
clear 
close all
clc
%%


[t,x]=ode45('ode_solver_script_pidcontrol',[0,10],[0,0,0,0]);

m1=1; m2=1; l1=1; l2=1; g=9.81;

th1=x(:,1); dth1=x(:,2); th2=x(:,3); dth2=x(:,4);



figure('units','normalized','outerposition',[0 0 1 1])

c=1;
for i=1:10:length(x)
    
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
    
    
    F(c) = getframe(gcf) ;
    c=c+1;
end

% create the video writer with 30 fps
writerObj = VideoWriter('PID.avi');
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
