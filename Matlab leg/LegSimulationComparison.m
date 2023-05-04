%% My Sim
%results
% [t,y] = ode15s( @(t,y) Controller.control_sim(t,y),ts_sim,yo,options);
[t,y] = ode45( @(t,y) Controller.control_sim(t,y),ts_sim,yo,options);


% get states
q1v = y(:,1);
q2v = y(:,2);
q3v = y(:,3);
q1tv = y(:,4);
q2tv = y(:,5);
q3tv = y(:,6);

tusim = Tsampling:Tsampling:ts_sim(end);
u1v = Controller.uhistory(1,:)';
u2v = Controller.uhistory(2,:)';
u3v = Controller.uhistory(3,:)';

%% RosBag
rosbagIMPORT;

%% simulink
% out = sim('Leg_Simulation_Framework_test_2.slx',ts_simulink);

%joint 1
q1simulink = out.q1; %get ps signal
w1simulink = out.qt1; %get ps signal
u1simulink = out.t1;

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


