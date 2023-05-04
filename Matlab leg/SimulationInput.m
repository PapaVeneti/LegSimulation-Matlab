%% input  1 --------------
Simulation_type = 'traj'; %'pos','traj','force','custom';

%waypoints
% testXpos

%simulation time:
ts_sim = [1e-9,10]; %ts_sim = [1e-10,1]; %1e-10 due to implementatio
ts_simulink = [0,10];

%sampling time:
Tsampling = 1e-3;

%set K for class implementation:
Kp = [10;10;10];
Kd = [1;1;1];
Ki = 0*[1;1;1];

% Kp = [10;10;12];
% Kd = [3.5;3.5;6];
% Ki = [3;3;3];

i_clamp_min = [-5;-5;-5];
i_clamp_max = [5;5;5];
i_clamp = [i_clamp_min,i_clamp_max];

%HLC and LLC parameters
K = [Kp,Kd,Ki];

%% ode options --------
options = odeset('AbsTol',1e-6,'RelTol',1e-8,'MaxStep',0.1e-3);
yo = [0;0;0;0;0;0]; %initial conditions

%% Controller 
%system
r= legRobot(tmax=1.007344*[7;10;10]);


switch Simulation_type
    case 'pos'
        Qd = [0;0;0];
%         Qd = [0.1667;0;0.08]; %second option 

        Controller = PosControl(r,Tsampling,K,i_clamp); %Pos Controller
        Controller.setAngleTarget(Qd(:,end)); %Set Target

%         Controller.setTarget(Qd(:,end));
    case 'traj'
        Controller = TrajControl(r,Tsampling,K); %Pos Controller

        eclipse_par %get ellipse parameters
        Controller.generateEllipse(a,b,[Dxe;Dye],dth,5,30,2)

% if you want to manually set cartesian wp
%         Controller.generateQ(x,tw);
% if you want to manually set angle targets
%         Controller.setWP(Qd,Qdt,tw);

    case 'force'
        %first trajectory
        tw = round(tw,3); %it can accept only in the scale of Tscale (SOS)
        Controller = TrajControl(r,Tsampling,K); %Pos Controller
        Controller.generateQ(x,tw);

    case 'custom'
        Qd = [0;0;0];
        Controller = PosControl(r,Tsampling,K,i_clamp); %Pos Controller
        Controller.setAngleTarget(Qd(:,end)); %Set Target
        

end

%% prepare simscape
switch Simulation_type
    case 'pos'
        %prepare pos;
        Qf = Controller.qd(:,end);
        Qpos  = timeseries(Qf ,0 );

        %trajectory 
        TSqd = timeseries([Qf,Qf], ts_simulink);
        TSqtd = timeseries([0 0;0 0;0 0 ],ts_simulink);

        %prepare force
        h = [0;0;0];
        TSh = timeseries(h ,0  );

        %Set Controller Qeue:
        ControllerQeue = [1];
        ControlerTimeChange = [0];

    case 'traj'
        %prepare pos; /after trajectory, it activates
        Qf = Controller.TG.Q_wp(:,end);
        tw = Controller.TG.t_wp; %ending of trajectory
        Qpos  = timeseries([Qf,Qf] ,[0 tw(end)] );

        %prepare trajectory;
        ttraj = linspace(tw(1),tw(end),length( Controller.TG.qd(1,:) ));
        TSqd  = timeseries(Controller.TG.qd(:,2:end) ,ttraj(2:end) );
        TSqtd = timeseries(Controller.TG.qdt(:,2:end) ,ttraj(2:end) );

        %prepare force
        h = [0;0;0];
        TSh = timeseries([h,h] ,[0 tw(end)] );
        
        %Set Controller Qeue:
        ControllerQeue = [2;1];
        ControlerTimeChange = [0 tw(end)];

        
    case 'force'
        %load torque bus datatype
        load('OutputTorqueBusType.mat');

        %prepare pos; /after trajectory, it activates
        Qf = Controller.TG.Q_wp(:,end);
        Qpos  = timeseries(Qf , 0 );

        %prepare trajectory; First Step
        Controller.TG.qd;
        Controller.TG.qdt;
        tw = Controller.TG.t_wp;
        ttraj = linspace(tw(1),tw(end),length( Controller.TG.qd(1,:) ));
        TSqd  = timeseries(Controller.TG.qd(:,2:end) ,ttraj(2:end) );
        TSqtd = timeseries(Controller.TG.qdt(:,2:end) ,ttraj(2:end) );

        %prepare force
        h1 = [0,0,-100];
        h2 = [0,0,-50];
        
        ForceChange = [h1;h2];
        ForceChangeTimings = [0,5.5];

        TSh = CreateZOHTimeseries(Tsampling,ForceChangeTimings,ForceChange);
        
        %Set Controller Qeue:
        ControllerQeue = [2;3];
        ControlerTimeChange = [0 tw(end)];


end

Select_Controller = CreateZOHTimeseries(Tsampling,ControlerTimeChange,ControllerQeue);


