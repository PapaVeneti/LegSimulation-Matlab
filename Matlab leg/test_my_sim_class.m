clear all;
close all;
clc;

%% Simulation Parameters:
%sampling time:
Tsampling = 1e-3;

%set K for class implementation:
Kp = [10;10;10];
Kd = [1;1;1];
Ki = 0*[1;1;1];

%HLC and LLC parameters
K = [Kp,Kd,Ki];

%% Initialize simulation handler
%Set initial conditions
% qinitial=[-0.0025;-1.1899;-1.2599]; %for static test
qinitial=[0;0;0];

r= legRobot(tmax=1.007344*[7;10;10],ground=true);
r.q = qinitial';

PC = PosControl(r,Tsampling,K); %Position Controller
TC = TrajControl(r,Tsampling,K); %Trajectory Controller
EC = EffortControl(r,Tsampling); %Effort Controller

% simulation_preparation
s = simulation_preparation(r,PC,TC,EC);
s.setInitialConditions([qinitial;0;0;0],0);

%!! turn on for custom simulation:
s.customSimulationsSwitch(true);

%% Position Controller test events (Uncomment for test 1)
% PC.setTarget([0.1667;0.05;0.08]);
% s.setEvent(0,1,1);
% PC.setTarget([0.1667;0;0.0225]);
% s.setEvent(1,1,1);
% PC.setTarget([0.1667;0;0.022]);
% s.setEvent(2,1,1);


%% Trajectory Controller test events (Uncomment for  test 2)
% testXpos
% TC.generateQ(x,tw,0);
% s.setEvent(0,2,tw(end));

%% Elliptical Trajectory test events (Uncomment for  test 3)
% eclipse_par
% TC.generateEllipse(a,b,DX,Dth,5,30,1,0);
% s.setEvent(0,2,5);



%% Pos - Traj events: (Uncomment for  test 4)
% PC.setTarget([0.1667;0.3;0.5]);
% s.setEvent(0,1,1);
% 
% testXpos
% TC.generateQ(x,tw,1);
% s.setEvent(1,2,tw(end));
% 
% PC.setTarget([0.1667;0.3;0.5]);
% s.setEvent(6,1,1);

%% Effort Test: (Uncomment for  test 5, set right initial conditions above)

% EC.setWrench([0,0,-100]);
% s.setEvent(0,3,1);

%% Combined Effort Test (Uncomment for  test 6):
% PC.setTarget([0.1667;0.05;0.08]);
% s.setEvent(0,1,1);
% 
% PC.setTarget([0.1667;0;0.0225]);
% s.setEvent(1,1,1);
% 
% EC.setWrench([0,0,-100]);
% s.setEvent(2,3,1);


%% Simscape Simulation
[Select_Controller,Qpos,TSqd,TSqtd,TSh] = s.SimScapeSimulation();
out = sim('Leg_Simulation_Framework_test_2.slx',[0 s.t_total]);

%joint 1
q1simulink = out.q1 ; %get ps signal
w1simulink = out.qt1; %get ps signal
u1simulink = out.t1 ;

%joint 2
q2simulink = out.q2; %get ps signal
w2simulink = out.qt2; %get ps signal
u2simulink = out.t2;

%joint 3
q3simulink = out.q3; %get ps signal
w3simulink = out.qt3; %get ps signal
u3simulink = out.t3;

%Contact Force
Fn = out.Fn;
Ff = out.Ff;


%% Custom sim data
[t,y,u,F] = s.getCustomSimData; 

% get states
q1v = y(:,1);
q2v = y(:,2);
q3v = y(:,3);
q1tv = y(:,4);
q2tv = y(:,5);
q3tv = y(:,6);

% tusim = Tsampling:Tsampling:t(end)-Tsampling; %last u command won't be simulated
% u1v = u(2:end,1);
% u2v = u(2:end,2);
% u3v = u(2:end,3);

tusim = 0:Tsampling:t(end); %last u command won't be simulated
u1v = u(:,1);
u2v = u(:,2);
u3v = u(:,3);

%% RosBag import (This is used for comparisons, but is deactivated)
close all;
% rosbagIMPORT;

Simulation_type = 'Default'; %traj for gazebo - matlab reference traj. 
ts_sim = [0 s.t_total];

Leg_plot