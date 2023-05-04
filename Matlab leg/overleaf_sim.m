%% Simulation paramters: ------------------------ 
Tsampling = 1e-3;

%set K for class implementation:
Kp = [10;10;10];
Kd = [1;1;1];
Ki = 0*[1;1;1];
K = [Kp,Kd,Ki];

%%  Set up simulation: --------------------------
r= legRobot(tmax=1.007344*[7;10;10]);

PC = PosControl(r,Tsampling,K); %Position Controller
TC = TrajControl(r,Tsampling,K); %Trajectory Controller
EC = TrajControl(r,Tsampling); %Trajectory Controller

% simulation_preparation
s = simulation_preparation(r,PC,TC,EC);
s.setInitialConditions([0;0;0;0;0;0],0);
s.customSimulationsSwitch(true); % false -> do not run custom sim

%%  set up tasks: -------------------------------
PC.setTarget([0.1667;0.3;0.5]);
s.setEvent(0,1,1);

%script to set up a 3xN vector x, and a 1xN vector tw. N = #waypoints
testXpos 
TC.generateQ(x,tw+1,1);
s.setEvent(1,2,tw(end));

PC.setTarget([0.1667;0.3;0.5]);
s.setEvent(tw(end)+1,1,1);

%% Run simscape simulation:
[Select_Controller,Qpos,TSqd,TSqtd,TSh] = s.SimScapeSimulation();
out = sim('Leg_Simulation_Framework_test_2.slx',[0 s.t_total]);