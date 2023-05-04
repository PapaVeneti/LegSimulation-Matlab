Tsimplify

%from ros
qq = [-0.11362458087418936, -1.182736210249276, -1.3141185459199907];
tt =[-0.7019779265105621, 0.6245049675194781, 1.5792841881873734];
hh = [2.4251060485839844, 1.2156858444213867, 11.37186050415039];
hh = [0.4337301552295685, 0.34810179471969604, 9.080540657043457]; %no gravity
 
syms Q1 Q2 Q3
TWEs = subs(TWE,[q1,q2,q3],[Q1,Q2,Q3]);
FT =matlabFunction(TWEs(100000000000)) 

%transform TE when in contact with the ground:
TEc = FT(qq(1),qq(2),qq(3))
REc = TEc(1:3,1:3)
%position of tip in the end_effector frame
Ptip = (REc')*[0;0;-0.045/2];

%transformation of the tip to end_effector
Ttip = [eye(3),Ptip;0 0 0 1]

%% jacobian

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
P0E = RS*( T123E*Ttip)  *PST;

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
JVtip = subs(JvW,[q1,q2,q3],[Q1,Q2,Q3]);
FJVtip =matlabFunction(JVtip); 
JVtip_q = FJVtip(qq(1),qq(2),qq(3));

t_exp  = (JVtip_q') * (hh')
