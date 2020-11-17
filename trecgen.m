function [t,q,dq,ddq] = trecgen(t0,dt,tf,q0,dq0,q1,dq1)
%% Function for trjectory generation using polynomial equation
t=t0:dt:tf;
q= (q0) + (dq0*t) + (((3*(q1 - q0))-(2*dq0 + dq1)*tf)/tf^2)*t.^2 + (((2*(q0 - q1))-(dq0 + dq1)*tf)/tf^3)*t.^3 ; %Position
dq=diff(q)/dt; %Velocity
ddq=diff(dq)/dt; %Acceleration
end