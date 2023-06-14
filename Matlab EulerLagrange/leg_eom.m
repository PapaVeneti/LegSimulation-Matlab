%This script creates the dynamics matrices for the simulation of the leg.
%It calculates the Lagrangian and then cals the EL_derivatives.m and
%EL_collect.m functions to create the matrices. 

syms q1t(t) q2t(t) q3t(t) ;
assume(q1t(t),'real');
assume(q2t(t),'real');
assume(q3t(t),'real');
syms zero;

Tsimplify
%% inertia info

% link 1
m1 = 0.4759;
rc1 = [-0.00083;-0.00061;0.01032];

Ixx1 = 0.000576572 ;
Ixy1 = 1.091449E-05;
Ixz1 =-4.096E-06;
Iyy1 = 0.000825053;	
Iyz1 =-3.018E-06;
Izz1 = 0.000391889;

% link 2
m2 = 0.94987;
rc2 = [0.03158;-0.00042;-0.02813];

Ixx2 = 0.001561735 ;
Ixy2 =-1.2538E-05;
Ixz2 =-0.000843757;
Iyy2 = 0.005174717;	
Iyz2 =-1.72E-05;
Izz2 = 0.004776989;

% link 3
m3 = 0.36814 ;
rc3 = [0.10653;0;0];

Ixx3 = 8.8159E-05;
Ixy3 = 0 ;
Ixz3 = 0;
Iyy3 = 0.003793803;	
Iyz3 = 0;
Izz3 = 0.003804351;



%% transformation data:
RS = [eye(3),zeros(3,1)]; %select Rl
PST = [0 ;0 ;0 ;1]; %select Rl

R1  = RS*T1*RS';
R12 = RS*T2*RS';
R2  = R1*R12;
R23 = RS*T3*RS';
R3  = R2*R23;jacobian_calcs = false;


P01 = RS*T1*PST;
P02 = RS*(T1*T2)*PST;
P03 = RS*T123*PST;
P12 = RS*T2*PST;
P23 = RS*T3*PST;

I1 = [ Ixx1, Ixy1, Ixz1; Ixy1, Iyy1, Iyz1; Ixz1, Iyz1, Izz1]; 
I2 = [ Ixx2, Ixy2, Ixz2; Ixy2, Iyy2, Iyz2; Ixz2, Iyz2, Izz2]; 
I3 = [ Ixx3, Ixy3, Ixz3; Ixy3, Iyy3, Iyz3; Ixz3, Iyz3, Izz3]; 
%% positions
xc1 = P01 + R1*rc1;
xc2 = P02 + R2*rc2;
xc3 = P03 + R3*rc3;

%% velocities 
% velocities of CS-Origin -------------------------------------------------
w1 = [ 0; 0; q1t];              w1 = simplify(w1);
w2 = w1 + R2*[0; 0 ;q2t];       w2 = simplify(w2);  
w3 = w2 + R3*[0; 0 ;q3t];       w3 = simplify(w3); 

%hand written
% w2 = [ sin(q1)*q2t;       -cos(q1)*q2t;       q1t];
% w3 = [ sin(q1)*(q2t+q3t); -cos(q1)*(q2t+q3t); q1t];

u1 = [zero;zero;zero]; u1 = u1*0;  u1 = simplify(u1);
u2 = u1 + exterior(w1)*R1*P12;     u2 = simplify(u2);
u3 = u2 + exterior(w2)*R2*P23;     u3 = simplify(u3);

%hand written
% u2 = q1t*[L12*cos(q1)               ; -L12*sin(q1)            ;0 ];

% velocities of COM -------------------------------------------------------
uc1 = u1 + exterior(w1)*R1*rc1; uc1 = simplify(uc1);
uc2 = u2 + exterior(w2)*R2*rc2; uc2 = simplify(uc2);
uc3 = u3 + exterior(w3)*R3*rc3; uc3 = simplify(uc3);

%hand written
% uc1 = q1t*[-C1y*cos(q1) - C1x*sin(q1);C1y*sin(q1) + C1x*cos(q1);0 ];

%% Kinetic Energy:

TK1 = 1/2*m1*(uc1.') * uc1 + 1/2*(w1.')*R1 *I1 * (R1.') *w1; TK1 = simplify(TK1);
TK2 = 1/2*m2*(uc2.') * uc2 + 1/2*(w2.')*R2 *I2 * (R2.') *w2; TK2 = simplify(TK2);
TK3 = 1/2*m3*(uc3.') * uc3 + 1/2*(w3.')*R3 *I3 * (R3.') *w3; TK3 = simplify(TK3);

TK = TK1+TK2+TK3; TK = simplify(TK);

%% Potential Energy
g = 9.8;

U = [g,0,0]*( m1*xc1 + m2*xc2+ m3*xc3);

L = TK-U;

%% E-L equations
states = [q1;q2;q3;q1t;q2t;q3t];

[L_qt,L_q] = EL_derivatives(L,states,3);
[B,C,G]    = EL_collect(L_qt,L_q,states,3);

%% extract
syms Q1 Q2 Q3 Q1t Q2t Q3t real
Bs = subs(B,[q1 q2 q3 q1t q2t q3t],[Q1 Q2 Q3 Q1t Q2t Q3t]);
Cs = subs(C,[q1 q2 q3 q1t q2t q3t],[Q1 Q2 Q3 Q1t Q2t Q3t]);
Gs = subs(G,[q1 q2 q3 q1t q2t q3t],[Q1 Q2 Q3 Q1t Q2t Q3t]);



