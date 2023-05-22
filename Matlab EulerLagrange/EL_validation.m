% Validation for EL functions. 
% Select the corespoding section and run the script to get the EOM in
% matrix form. 

clear all; 
syms q1(t) q1t(t) q2(t) q2t(t);
syms I1 I2 m1 m2 g l r1 ;
syms a1 a2 l1 l2;

%%  simple pendulum (pg 12)
T = 1/2*I1*q1t^2;
U = m1*g*l/2*(1-cos(q1));
L = T-U;

[L_qt,L_q] = EL_derivatives(L,[q1;q1t],1);
[B,C,G]    = EL_collect(L_qt,L_q,[q1;q1t],1);

%%  3D - simple pendulum (pg 13)
T = 1/2*m1*l^2*(q1t^2+(1-cos(q1)^2)*q2t^2);
U = -m1*g*l*cos(q1);
L = T-U;

[L_qt,L_q] = EL_derivatives(L,[q1;q2;q1t;q2t],2);
[B,C,G]    = EL_collect(L_qt,L_q,[q1;q2;q1t;q2t],2);

%% extensible arm (pg 22 ->25 for matrix form)   
T  =  1/2*(I1+m1*r1^2)*q1t^2 + 1/2*m2*q2t^2 +1/2*(I2+m2*q2^2)*q1t^2;
U  = m1*r1*(sin(q1)*g) + m2*q2*sin(q1)*g;
L = T-U;


[L_qt,L_q] = EL_derivatives(L,[q1;q2;q1t;q2t],2);
[B,C,G]    = EL_collect(L_qt,L_q,[q1;q2;q1t;q2t],2);

%% 2link (pg 28)
x1 = cos(q1)*l1;
y1 = sin(q1)*l1;
x1t = -l1*q1t*sin(q1); 
y1t =  l1*q1t*cos(q1);

x2 = a1*cos(q1)+l2*cos(q1+q2);
y2 = a1*sin(q1)+l2*sin(q1+q2);
x2t = -l2*q2t*sin(q1+q2) - q1t*( a1*sin(q1)+l2*sin(q1+q2) ); 
y2t =  l2*q2t*cos(q1+q2) + q1t*( a1*cos(q1)+l2*cos(q1+q2) ); 

T = 1/2*m1*(x1t^2+y1t^2) + 1/2*I1*q1t^2+...
     1/2*m2*(x2t^2+y2t^2) + 1/2*I2*(q1t+q2t)^2;
U = m1*g*y1+m2*g*y2;
L=T-U;

[L_qt,L_q] = EL_derivatives(L,[q1;q2;q1t;q2t],2);
[B,C,G]    = EL_collect(L_qt,L_q,[q1;q2;q1t;q2t],2);





