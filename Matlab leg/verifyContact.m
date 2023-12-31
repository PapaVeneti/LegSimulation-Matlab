%% Data
Fsim = 50.4603;
Fmat = 51.1818;

tsim = [ -6.22209;7.64137;10.0734] ;
tmat = [ -6.24277;7.71709;10.0734] ;

rq = [    -0.10372970806022
         -1.17351706627888
         -1.29147755466255] ;
% rG = [          1.04261693284394
%          0.163050124413438
%         -0.298362951946575];
% rx = [         0.128403864196956
%        0.00168241947987868
%         0.0173818498782159]; 
% rJV = [0.382618150121784       -0.0078639002312777       -0.0180036492071179
%                          0         0.373017972664917         0.139630137308457
%         0.0728538641969561       -0.0755393543791102        -0.172940143895073];

r.q = rq;

% q = [-0.104322899637673;-1.1706910888524 ;-1.29337213853489 ];
% r.q = q;
r.DK;
r.x;
r.MCG;


q = r.q;

wZpoc =   -0.0225;%   -  0.0174538576850627; %-0.0225;

%% extra  Transformation matrix
Tsimplify

wR3(1,1:3) = [-sin(q(1))*sin(q(2) + q(3)), -sin(q(1))*cos(q(2) + q(3)), cos(q(1))];
wR3(2,1:3) = [            cos(q(2) + q(3)),            -sin(q(2) + q(3)),          0];
wR3(3,1:3) = [ cos(q(1))*sin(q(2) + q(3)),  cos(q(1))*cos(q(2) + q(3)), sin(q(1))];


Zpoc_EE_3 = (wR3')*[0;0;wZpoc];

Tpoc = [
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
% Tpoc(1:3,4) = Zpoc_EE_3;

%% Jacobian
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
P0E = RS*(T123E*Tpoc)*PST;

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






JVf  = matlabFunction(JVWs);

L12 = 0.11208;
L23 = 0.2531;
L3E = 0.223;

J =  JVf(L12,L23,L3E,q(1),q(2),q(3));

inv(J')*(tmat-r.G)

