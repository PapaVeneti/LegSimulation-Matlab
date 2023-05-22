%This script calculates the Jacobian matrix and finds the determinant from
%which the singularities are found

Tsimplify

%% transformation data
RS = [eye(3),zeros(3,1)]; %select Rl
PST = [0 ;0 ;0 ;1]; %select Rl

%Rot
RW  = RS*TW*RS';
R1  = RS*T1*RS';
R12 = RS*T2*RS';
R23 = RS*T3*RS';

R2  = R1*R12;
R3  = R2*R23;

%z
z1 = R1*[0;0;1];
z2 = R2*[0;0;1];
z3 = R3*[0;0;1];

%Pos
P01 = RS*T1     *PST;
P02 = RS*(T1*T2)*PST;
P03 = RS*T123   *PST;
P0E = RS*T123E  *PST;

PE1 = P0E - P01;
PE2 = P0E - P02;
PE3 = P0E - P03;

Jv = sym('J',3);
Jv(:,1) =  exterior(z1)*PE1;
Jv(:,2) =  exterior(z2)*PE2;
Jv(:,3) =  exterior(z3)*PE3;

Jv = simplify(Jv);
JvW = RW*Jv;

syms Q1 Q2 Q3
JVWs = subs(JvW,[q1,q2,q3],[Q1,Q2,Q3]);

%% singularities
detJv = simplify( det( Jv ));

%% singularities deep dive
L23 = 0.2531;
L3E = 0.223;

q2v = linspace(-pi,pi,25);
q3a = q2v + asin( L3E/L23 * sin(q2v) );
q3b = q2v +pi -  asin( L3E/L23 * sin(q2v) );

figure("Name",'Singularites deep dive');
plot(q2v,q3a)
hold on
plot(q2v,q3b)
hold off

