clear all;
close all;
clc;

%% 
%sampling time:
Tsampling = 1e-3;

%set K for class implementation:
Kp = [10;10;10];
Kd = [1;1;1];
Ki = 0*[1;1;1];

%HLC and LLC parameters
K = [Kp,Kd,Ki];

%% 
r= legRobot(tmax=1.007344*[7;10;10],ground=true);

PC = PosControl(r,Tsampling,K); %Position Controller
TC = TrajControl(r,Tsampling,K); %Trajectory Controller
EC = EffortControl(r,Tsampling); %Effort Controller

% simulation_preparation
s = simulation_preparation(r,PC,TC,EC);
s.setInitialConditions([0;0;0;0;0;0],0);
s.customSimulationsSwitch(true);

%% Pos test events
%event 1:
% PC.setTarget([0.1667;0.05;0.08]);
% % PC.setAngleTarget([0;0;0])
% s.setEvent(0,1,1);
% 
% % PC.setAngleTarget([1;1;1])
% 
% PC.setTarget([0.1667;0;0.0225]);
% s.setEvent(1,1,1);
% PC.setTarget([0.1667;0;0.022]);
% s.setEvent(2,1,1);
% 
% % PC.setTarget([0.1667;0.2;0.08]);
% PC.setAngleTarget([0;0;1])
% s.setEvent(2,1,1);
% PC.setTarget([0.1667;0;0.08]);
% s.setEvent(3,1,1);
% PC.setTarget([0.1667;0;0.0225]);
% s.setEvent(4,1,1);

%% traj  test events
% testXpos2
% TC.generateQ(x,tw,0);
% s.setEvent(0,2,tw(end));
% % 
% TC.generateQ(x,tw+tw(end),tw(end));
% s.setEvent(tw(end),2,tw(end));

%% Pos - Traj events:
% PC.setTarget([0.1667;0.3;0.5]);
% s.setEvent(0,1,1);
% 
% testXpos
% TC.generateQ(x,tw+1,1);
% s.setEvent(1,2,tw(end));
% 
% PC.setTarget([0.1667;0.3;0.5]);
% s.setEvent(6,1,1);
% 
% testXpos
% TC.generateQ(x,tw+7,7);
% s.setEvent(7,2,tw(end));



% PC.setTarget([0.1667;0;0.08]);
% s.setEvent(2,1,1);
% PC.setAngleTarget([0;0;0.0]);
% s.setEvent(3,1,1);
% 
% 
% testXpos
% TC.generateQ(x,tw+4,4);
% s.setEvent(4,2,tw(end));
% % 
% 

%% Effort Test:
%event 1:
PC.setTarget([0.1667;0.05;0.08]);
s.setEvent(0,1,1);

PC.setTarget([0.1667;0;0.0225]);
s.setEvent(1,1,1);

EC.setWrench([0,0,-100]);
s.setEvent(2,3,1);


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

%% RosBag
rosbagIMPORT;

Simulation_type = 'pos';
ts_sim = [0 t(end)];

Leg_plot