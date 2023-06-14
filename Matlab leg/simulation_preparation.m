classdef simulation_preparation < handle
    % simulation_preparation : setting up simscape and matlab simulation
    %
    % This class is responsible for setting up the simscape simulation and
    % (along, if the user requests, with the matlab simulation). The robot
    % and the controller are passed as handles (by reference in c++
    % terminology), so the user easily sets new control targets using the
    % controller class methods. 
    %
    %
    %
    %Important:
    % 1: The timeseries that are imported in simscape must have the option
    % "keep final value" selected. It won't work with extrapolation. 

    %To do:
    % 1.sync errors when effort is activated (must be compatible with gazebo)
    % logika mono simscape thelei alagi
    
    properties (SetAccess=private,GetAccess=public)
        R robot = DefaultRobot; % Robot
        PC  PosControl          % Position Controller
        TC  TrajControl         % Trajectory Controller
        EC (1,1) EffortControl;
        Tsampling double = 1e-3; %sampling time (Default: 1e-3)

        ts (1,:) double  =[] %event timestamps
        t_total (1,1) double = 0 %total simulation runtime
        
        Qpos_ts double =[] %position target for the ZOH timeseries creation (dim,:) 
        qd_ts   double =[] %trajectory reference target for the ZOH timeseries creation (dim,:)
        qtd_ts  double =[] %trajectory velocity reference target for the ZOH timeseries creation (dim,:)
        h_ts    double =[] %effort target for the ZOH timeseries creation (dim,:)

        options struct    %struct containing ode solver parameters 
        MaxTimeStep (1,1) double = 1e-4; %max time step for ode (Default: 1e-4)

        y double       %total state history
        t (:,1) double %total simulation time
        u double       %total input history
        F double       %total Interaction force history
        
        DoCustomSimulations logical = false %switch on/off the custom simulations
    end

    properties (Access = private) 
        SC    ; %SelectController (Custom Timeseries for simscape)
        Qpos  ; %Qpos             (Custom Timeseries for simscape)
        TSqd  ; %TSqd             (Custom Timeseries for simscape)
        TSqtd ; %TSqtd            (Custom Timeseries for simscape)
        TSh   ; %TSh              (Custom Timeseries for simscape)
    end

    methods (Access=public) 

        function obj = simulation_preparation(R,PC,TC,EC)
            arguments
                R robot
                PC PosControl
                TC TrajControl
                EC EffortControl
            end
            % simulation_preparation : Contructor for the simulation_preparation class 
            %
            % 
            % It accepts: 
            % -the robot instance Robot,
            % -the position controller PC, 
            % -the trajectory controller TC, 

            obj.R  = R;
            obj.PC = PC;
            obj.TC = TC;
            obj.EC = EC;
            obj.options = odeset('AbsTol',1e-6,'RelTol',1e-8,'MaxStep',obj.MaxTimeStep);
        end
        
        function setEvent(obj,tnew,cnew,tsim)
            arguments
                obj  (1,1) simulation_preparation
                tnew (1,1) double {mustBeNonnegative}
                cnew (1,1) int16 {mustBeInRange(cnew,1,3,"inclusive")}
                tsim (1,1) double {mustBePositive}
            end
            %setEvent : Setup the simulation for a new event.
            %
            %Should be called after setting a new target with the specific
            %controller, to run the matlab simulation and prepare the the 
            %simscape simulation. 
            %
            % It accepts: 
            % -the start time of the new event
            % -the controller that will be active during the new event
            % -the duration of the simulation
            %
            %Notes: The custom simulation will run if: 
            % DoCustomSimulations = true 
            

            %1: check correct timestamp
            if ~isempty(obj.ts)
                if tnew <= obj.ts(end)
                    error("[simulation preparation]: New event (switching controller) must happen after the previous one.")
                elseif tnew < obj.t_total
                    disp ( "[simulation preparation]: Simulation is in a future moment")
                    error(['[simulation preparation]: Lower the duration of the previous simution to: ', ...
                        num2str( tnew - obj.ts(end) ), 's'])
                end
            end
            
            %2: timestamps
            obj.t_total = max(obj.t_total,tnew)+tsim;
            obj.ts      = [obj.ts  ,tnew];

            tspan = [tnew tnew+tsim]; %tspan is increasing by cosntruction
            
            %3: update Simscape Input Timeseries
            obj.updateTimeSeries(cnew,tspan);
            
            %4: run custom simulation
            if obj.DoCustomSimulations
                obj.runCustomSimulations(cnew,tspan);
            end
            
        end
        
        function [Select_Controller,Qpos,TSqd,TSqtd,TSh] = SimScapeSimulation(obj)
        %SimScapeSimulation : Function to prepare and run the simscape
        %simulation. This method publishes the simscape required
        %timeseries in the workspace. 

            

            Select_Controller = [obj.SC ; obj.t_total ,obj.SC(end,2)  ];
            Qpos              = obj.Qpos;
            TSqd              = obj.TSqd;
            TSqtd             = obj.TSqtd;
            TSh               = obj.TSh;

        end
        
        function [t,y,u,F] = getCustomSimData(obj)
            %getCustomSimData : return [t,y] from custom sim
            obj.F = obj.R.Fev_history;

            if obj.DoCustomSimulations
                t = obj.t;
                y = obj.y;
                u = obj.u; 
                F = obj.F; 
            else
                error("Custom Simulation is inactive");
            end
        end

        % Options
        function setInitialConditions(obj,yo,to)
        %setInitialConditions : Function to set initial conditions 
        %
            
            if isempty(obj.y)
                if size(yo) ~= [obj.R.dim,1]
                    error("[simulation_preparation]: Wrong initial conditions dimensions")
                end
                obj.R.q    = yo ;
                obj.y(1,:) = yo';
                obj.t      = to;
            end
            obj.u = zeros(1,obj.R.dim); 
        
        end
      
        function customSimulationsSwitch(obj,flag)
        %customSimulationsSwitch : Function to turn on/off custom simulation in matlab. 
        %
        %Usefull for initializing just the Simscape simulation

            obj.DoCustomSimulations = flag;
        end

    end

    % to do, add free system sim
    methods (Access=private)

        function runCustomSimulations(obj,cnew,tsim)
        arguments
            obj simulation_preparation;
            cnew     (1,1) double ; 
            tsim     (1,2) double ; %increasing by construction
        end
        %runCustomSimulations : run custom simulations
        

            switch cnew
                case 1 %Position Controller
                        obj.simulatePC(tsim)
           
                case 2 %Trajectory Controller
                        obj.simulateTC(tsim);

                case 3 %Effort Controller
                    obj.simulateEC(tsim);
            end
        end

        function updateTimeSeries(obj,cnew,tspan)
        arguments
            obj simulation_preparation;
            cnew     (1,1) double ; 
            tspan    (1,2) double ; %increasing by construction
        end
        %updateTimeSeries : This function updates the timeseries properties
        %of the class that are inputs to simscape simulation. 
        %
        %It creates timeseries packet only for the current event.
        %It appends the  new packets below the previous ones:
        %TSi = [TSi ; TSi_packet]
              
        %1: Create new Timeseries packet

        SCts = obj.singleZOH_TS( tspan ,cnew);
        switch cnew


            case 1 %Position Controller
                PosTS = obj.singleZOH_TS( tspan ,obj.PC.qd );
                QdTS  = obj.singleZOH_TS( tspan ,obj.PC.qd );
                QdtTS = obj.singleZOH_TS( tspan ,[0;0;0]   );

                if isempty(obj.h_ts)
                    hTS   = obj.singleZOH_TS(tspan ,[0;0;0]             ); 
                else
                    hTS   = obj.singleZOH_TS(tspan ,obj.TSh(end,2:end)' );
                end  

            case 2 %Trajectory Controller

                ttraj = linspace(tspan(1),tspan(2), size(obj.TC.TG.qd,2 ))';

                PosTS = obj.singleZOH_TS( tspan , obj.TC.TG.qd(:,end) );
                QdTS  = [ttraj, obj.TC.TG.qd'  ];
                QdtTS = [ttraj, obj.TC.TG.qdt' ];

                if isempty(obj.h_ts)
                    hTS   = obj.singleZOH_TS(tspan ,[0;0;0]             ); 
                else
                    hTS   = obj.singleZOH_TS(tspan ,obj.TSh(end,2:end)' );
                end

            case 3 %Effort Controller
                PosTS = obj.singleZOH_TS( tspan ,obj.R.q );
                QdTS  = obj.singleZOH_TS( tspan ,obj.R.q);
                QdtTS = obj.singleZOH_TS( tspan ,[0;0;0]   );
                hTS   = obj.singleZOH_TS(tspan  ,obj.EC.hd );
               
        end %switch end
        
        %2: Update Timeseries
                obj.SC     = [obj.SC    ; SCts  ];
                obj.Qpos   = [obj.Qpos  ; PosTS ];
                obj.TSqd   = [obj.TSqd  ; QdTS  ];
                obj.TSqtd  = [obj.TSqtd ; QdtTS ];
                obj.TSh    = [obj.TSh   ; hTS   ];

        end
         
        function simulatePC(obj,ts_sim)
        %simulatePC : Position Control Simulation 
        %
        %tsim is the interval of the simulation

            %1: Must Call before new simulation (see controller classes):
            obj.PC.getControllerUpToDate(ts_sim(1));
            
            %2: Run simulation and save results:
            [t_temp,y_temp] = ode45( @(t,y) obj.PC.control_sim(t,y), ts_sim, obj.y(end,:)' ,obj.options);

            obj.t = [obj.t ; t_temp(2:end)   ];
            obj.y = [obj.y ; y_temp(2:end,:) ];
            obj.u(end,:) = []; %last commands from previous control wasn't simulated! 
            obj.u = [obj.u ; obj.PC.uhistory'];

            %3: sync errors:
            obj.TC.syncErrors(obj.PC.error,obj.PC.d_error,obj.PC.i_error)
        end

        function simulateTC(obj,ts_sim)
        %simulatePC : Trajectory Control Simulation
        %
        %tsim is the interval of the simulation

            %1: Must Call before new simulation (see controller classes):
            obj.TC.getControllerUpToDate(ts_sim(1));

            %2: Run simulation and save results:
            [t_temp,y_temp] = ode45( @(t,y) obj.TC.control_sim(t,y), ts_sim, obj.y(end,:)' ,obj.options);

            obj.t = [obj.t ; t_temp(2:end)   ];
            obj.y = [obj.y ; y_temp(2:end,:) ];
            obj.u(end,:) = []; %last commands from previous control wasn't simulated! 
            obj.u = [obj.u ; obj.TC.uhistory'];
            

            %3: sync errors:
            obj.PC.syncErrors(obj.TC.error,obj.TC.d_error,obj.TC.i_error)

        end

        function simulateEC(obj,ts_sim)
        %simulateEC : Effort Control Simulation 
        %
        %tsim is the interval of the simulation. It doesn't sync errors

            %1: Must Call before new simulation (see controller classes):
            obj.EC.getControllerUpToDate(ts_sim(1));
            
            %2: Run simulation and save results:
            [t_temp,y_temp] = ode45( @(t,y) obj.EC.control_sim(t,y), ts_sim, obj.y(end,:)' ,obj.options);

            obj.t = [obj.t ; t_temp(2:end)   ];
            obj.y = [obj.y ; y_temp(2:end,:) ];
            obj.u(end,:) = []; %last commands from previous control wasn't simulated! 
            obj.u = [obj.u ; obj.EC.uhistory'];

            %3: sync errors:
            
%             obj.TC.syncErrors(obj.PC.error,obj.PC.d_error,obj.PC.i_error)
        end

        function [STS]  = singleZOH_TS(obj,Tspan,Quantity)
        arguments
            obj      simulation_preparation;
            Tspan    (1,2) double ;
            Quantity (:,1) double ; 
        end
        %singleZOH_TS : Create simscape compatible timeseries. 
        %
        %This method is appropriate for one event only.
        % It accepts: 
        % -the interval of the event,
        % -the Quantity from which we want to create a timeseries. 
        %  Important: It must be a column vector
        %
        %The output is:
        % [ time ,... ,[Quantity(i) ;Quantity(i)] ,... ], 

                dim = size(Quantity,1);
                STS = zeros(2,dim+1);

                STS(:,1)       = [Tspan(1)   ;Tspan(2)-obj.Tsampling];
                STS(:,2:dim+1) = [Quantity';Quantity'     ]; 
                
        end

        % (Deprecated)
        function [SCTS] = CreateZOHTimeseries(Tsampling,Tchange,Qchange)
            arguments
                Tsampling (1,1)double;
                Tchange (1,:) double;
                Qchange (:,:) double
            end
            % CreateZOHTimeseries -> LegSimulation helper function
            % (Deprecated)
            %
            % Function to create ZOH timeseries, just by specyfing the timestapms that
            % a quantity (or more) changes and the new value of a quantity. It is 
            % assumed that before the changes, the quantity keeps the previous value.
            %
            % Qchange must be in the following form:
            % Qchange = [Quantity1, Quantity2, ...] 
            % where Quantity1 = [Q1_1;Q1_2;Q1_3;...]
            
            Nchanges = size(Tchange,2);
            dim      = size(Qchange,1);
            SCTS = zeros(2*Nchanges-1,dim+1);%the first, and two for everyother change.
            
            for i = 1:Nchanges-1
            SCTS( (2*i-1),2:2+(dim-1) ) = Qchange(:,i)';
            SCTS(  2*i   ,2:2+(dim-1) ) = Qchange(:,i)';
            SCTS( (2*i-1)    ,1 )       = Tchange(i);
            SCTS(  2*i       ,1 )       = Tchange(i+1) - Tsampling;
            end
            SCTS(2*Nchanges-1,1)           = Tchange(Nchanges);
            SCTS(2*Nchanges-1,2:2+(dim-1)) = Qchange(:,Nchanges);

        end
    
    end

end
