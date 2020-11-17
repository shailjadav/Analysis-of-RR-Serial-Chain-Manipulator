function Out= ode_solver_script_q1_c(t,x)

%% Input parameters
m2=1; m1=1; l1=1; l2=1; g=9.81;

tau1=sin(t); %Input torque is zero
tau2=tau1; %second torque is the same as torque 1


%% Equation of motion 
M11=((((m1/3) + m2)*l1^2) +((m2/3)*l2^2) + (m2*l1*l2*cos(x(3))));
M12=(m2*(((l2^2)/3) + (0.5*l1*l2*cos(x(3)))));
M21=M12;
M22=((1/3)*m2*l2*l2);


H1 =((-m2*l1*l2*sin(x(3))*x(2)*x(4)) - (0.5*m2*l1*l2*sin(x(3))*x(4)*x(4)));
H2 = (0.5 * m2* l1*l2*sin(x(3))*x(2)*x(2));

G1=( ((((0.5*m1) + m2)*l1*cos(x(1))) + (0.5*m2*l2*cos(x(1)+x(3))))*g);
G2=0.5*m2*l2*cos(x(1)+x(3))*g;

T=[tau1; tau2];

M=[M11 M12;M21 M22];

HG = [H1 + G1; H2 + G2];

%% Equation in terms of acceleration

ddth = (inv(M)) * (T  - HG )    ;

OP=zeros(4,1);

%% Output
OP(1)=x(2); %Intergretion of velocity will give the position for theta 1
OP(2)=ddth(1);%Intergretion of acceleration will give the velocity for theta 1
OP(3)=x(4); %Intergretion of velocity will give the position  for theta 2
OP(4)=ddth(2); %Intergretion of acceleration will give the velocity for theta 2


Out=OP; % Output

end