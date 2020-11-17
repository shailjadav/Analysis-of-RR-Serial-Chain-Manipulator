function Out= ode_solver_script_pidcontrol(t,x)

m2=1; m1=1; l1=1; l2=1; g=9.81;

th1f=pi/6;
th2f=pi/3;

tau1=(-3.474e-05)*t.^6 + (0.0008649 )*t.^5 + (-0.004596)*t.^4 + (-0.01598  )*t.^3 + ( 0.03615)*t.^2 + (-0.0571 )*t +19.76; %Input torque is zero
tau2=(-2.676e-05 )*t.^6 + (0.0006413 )*t.^5 + (-0
.003133)*t.^4 + (-0.01392  )*t.^3 + ( 0.03229  )*t.^2 + (-0.03644 )*t +4.956; %second torque is the same as torque 1




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

S=[0 ; 0.001*x(3)];

%PID Controller
P=[853*(th1f - x(1)) - 100*x(2) + 0.5*x(1) ;553*(th2f - x(3)) - 100*x(4)  + 0.5*x(3)];



ddth = (inv(M)) * (T  - HG -S +P)    ;

OP=zeros(4,1);

OP(1)=x(2);
OP(2)=ddth(1);
OP(3)=x(4);
OP(4)=ddth(2);


Out=OP;
end