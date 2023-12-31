classdef EffortControl <handle
    % EffortControl Effort Controller class
    % This class implements a similar effort controller to the one
    % implemented in ros using ros_control.
    %
    %Important: 
    % 1: Robot object must have non-zero tmax.
    % 2: Call `getControllerUpToDate` before new simulation (important in 
    % consecutive simulations)
    %
    
    % Improvements: Preallocate uhistory to speed up
    
    properties (GetAccess = public, SetAccess = protected)
        tmax (:,1) double; %max torque output
        
        hd double; %desired wrench in world frame 
       
        R robot = DefaultRobot(); %robot
        N double ;                %state dimentions
        Ts double = 1e-3;         %sampling time
        
        %control command specific
        Ntime double = 0; %call timer: 
        
        %this variable is usefull as it let's the controller sim method
        %issue a command only when DT=Ts has passed, simulating a ZOH.

        u        double;  %command
        uhistory double;  %command history
    end

    methods (Access=public)

        % Public functions for user

        function obj = EffortControl(Robot,Ts)
            arguments
                Robot (1,1)   robot  = DefaultRobot; 
                Ts (1,1)      double  {mustBeNumeric,mustBePositive} = 1e-3;       
            end
            % PosControl : Contructor for the class PosControl. 
            % 
            % It accepts: 
            % -the robot instance Robot,
            % -the sampling rate Ts, 

            obj.R  = Robot;
            obj.N  = Robot.dim;
            obj.Ts = Ts;
            obj.tmax = Robot.tmax;

            %initial target -> zero wrench
            obj.hd = zeros(obj.R.dim,1);
            
            obj.reset();
            
        end
         
        function  setWrench(obj,target)
            arguments
                obj (1,1) EffortControl
                target (:,1) double {mustBeNumeric}
            end
            %setWrench : Set desired end effector wrench in the world frame
            obj.hd = target;

        end
        
        % Public functions for simulation class    

        function  reset(obj)
            arguments
                obj (1,1) EffortControl
            end
            %reset : Reset discrete simulation parameters and errors
                obj.Ntime  = 0;
        end

        function  getControllerUpToDate(obj,tnow)
            arguments
                obj  (1,1) EffortControl
                tnow (1,1) double {mustBeNonnegative}
            end
            %getControllerUpToDate : Important to call before new simulation. 
            % 
            % It accepts: 
            % Tnow = the starting time of the new simulation
                obj.Ntime  = floor(tnow/obj.Ts);
                obj.uhistory = []; %delete controller history.


            %We call a new command when t> obj.Ntime* obj.Ts, where t is
            %the ode time. So, we synchronize the controller Ntime with the
            %discrete time interval of the simulation
        end

        function [yt] = control_sim(obj,t,y)
            %control_sim : Simulate the controller in matlab ode solvers
            %
            %argument validation is avoided to increase performance
            %`obj.R.sim(t,y)` MUST be called first, as it updated the robot
            %state and relevant matrices.
            
            %this sets the state (q,qt,x,xt) , and calculates the new M,Jv
            ydyna =  obj.R.sim(t,y); 

            M = obj.R.M;

            %remember to call getControllerUpToDate before new sims
            if t>= obj.Ntime* obj.Ts 
                obj.Ntime = obj.Ntime+1;
                obj.command()
            end

            %output
            yt = ydyna +[zeros(obj.N,1); M\(obj.u)]; 

        end
    end

    methods (Access=private)

        function  command(obj)
            %command : Calculate position controller command. It
            %also updates the input history

            % the robot has updated Jv,G

            obj.u = (obj.R.Jv')* obj.hd + obj.R.G ; 
            

            
            %% saturate controller:
            Is = obj.u > obj.tmax;
            obj.u(Is) = obj.tmax(Is);
            
            %for debugging:
            if sum(Is) > 0
                disp("max torque")
            end

            Is = obj.u < (-obj.tmax);
            obj.u(Is) = -obj.tmax(Is);
            
            %for debugging:
            if sum(Is) > 0
                disp("max negative torque")
            end

            %% save input history
            obj.uhistory(:,end+1) = obj.u;

        end

    end
end
