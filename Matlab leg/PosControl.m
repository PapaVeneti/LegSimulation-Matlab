classdef PosControl <handle
    % PosControl Position Controller class
    % This class implements a similar position controller to the one
    % implemented in ros using ros_control.
    %
    %Important: 
    % 1: Robot object must have non-zero tmax.
    % 2: Call `getControllerUpToDate` before new simulation (important in 
    % consecutive simulations)
    %

    
    % Improvements: Preallocate uhistory to speed up
    
    properties (GetAccess = public, SetAccess = protected)
        Kp (:,1) double ; %Proportinal Gains (in a column vector)
        Kd (:,1) double ; %Derivative Gains (in a column vector)
        Ki (:,1) double ; %Integral Gains (in a column vector)

        i_clamp_min double;  %min accumulated integral error
        i_clamp_max double ; %max accumulated integral error

        tmax (:,1) double; %max torque output

        xd double; %goal in cartesian coordinates
        qd double; %goal in joint angles

        error double ;   %position error
        d_error double ; %derivative error
        i_error double ; %integral error

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

        function obj = PosControl(Robot,Ts,K,i_clamp)
            arguments
                Robot (1,1)   robot  = DefaultRobot; 
                Ts (1,1)      double  {mustBeNumeric,mustBePositive} = 1e-3;
                K             double  {mustBeNumeric} = 0;
                i_clamp (:,2) double  {mustBeNumeric} = [-inf,inf];
        
            end
            % PosControl : Contructor for the class PosControl. 
            % 
            % It accepts: 
            % -the robot instance Robot,
            % -the sampling rate Ts, 
            % -the gain matrix K in the form:
            %       K=[Kp,Kd,Ki], where Kp = [Kp1;Kp2;Kp3;...]
            % 
            % **Optional Arguments**
            % - the intergration saturation limits as optional arguments in the form:
            %       i_clamp = [min,max], where [min= min1;min2;...]


            obj.R  = Robot;
            obj.N  = Robot.dim;
            obj.Ts = Ts;
            obj.tmax = Robot.tmax;

            obj.Kp = K(:,1);
            obj.Kd = K(:,2);
            obj.Ki = K(:,3);

            %set i clamp
            if nargin==4
                obj.i_clamp_min = i_clamp(:,1);
                obj.i_clamp_max = i_clamp(:,2);
            else
                obj.i_clamp_min = -1e9*zeros(obj.N,1);
                obj.i_clamp_max = 1e9*zeros(obj.N,2);
            end

            %initial errors =0:
            obj.reset();

            %initial target -> Current position of robot
            obj.qd = obj.R.q;
            obj.xd = obj.R.x;
            
        end
        
        function  setTarget(obj,target)
            arguments
                obj (1,1) PosControl
                target (:,1) double {mustBeNumeric}
            end
            %setTarget : Set cartesian target for the controller
            obj.xd = target;

            %Inverse kinematics
            obj.R.IK(obj.xd);

            obj.qd(1:obj.N,1) = obj.findBestSol();

            %for debugging:
            disp('[Position Controller] Angle Target:')
            disp(obj.qd);
        end
            
        function  setAngleTarget(obj,target)
            arguments
                obj (1,1) PosControl
                target (:,1) double {mustBeNumeric}
            end
            %setAngleTarget : Set joint angle target for the controller
            obj.qd = target;
            obj.xd = obj.R.DK_querry(obj.qd);
        end
        
        % Public functions for simulation class    

        function  reset(obj)
            arguments
                obj (1,1) PosControl
            end
            %reset : Reset discrete simulation parameters and errors
                obj.Ntime  = 0;
                obj.error  = zeros(obj.N,1);
                obj.d_error= zeros(obj.N,1);
                obj.i_error= zeros(obj.N,1);
                obj.u      = zeros(obj.N,1);
        end

        function  syncErrors(obj,error,d_error,i_error)
            arguments
                obj PosControl  
                error double   {mustBeReal}
                d_error double {mustBeReal}
                i_error double {mustBeReal}
            end
            %syncErrors : Synchronize errors, as in ros there is one pid

            obj.error    = error   ;
            obj.d_error  = d_error ;
            obj.i_error  = i_error ;

        end

        function  getControllerUpToDate(obj,tnow)
            arguments
                obj  (1,1) PosControl
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
            
            %this sets the state (q,qt,x,xt) , and calculates the new M
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

        function  BestSol = findBestSol(obj)
        %findBestSol : This function finds the best solution from the set
        %of IK solutions. It assumes that the robot starts from the current
        %position and ends in the desired position in a standstill
        %
        % **The current implementation is not optimal:**
        % Best solution is the solution with the smallest euclidean
        % distance in the joint angle space.
        
        %find distance
        D = obj.R.Dist( obj.R.sol);

        %calculate norm (diagonal elements of (D^T * D) ):
        Qdist = diag( (D') * D ) ;     

        %get solution with minumum norm
        [~,best_sol] = min(Qdist) ;

        BestSol = obj.R.sol(1:obj.N,best_sol);
       
        end
        
        function  command(obj)
            %command : Calculate position controller command. It
            %also updates the input history
            
            %% PID 
            error_last  = obj.error; 

            %errors:
            obj.error   = obj.R.Dist(obj.qd); %correct distance function
            obj.d_error = (obj.error - error_last)/obj.Ts;
            obj.i_error = obj.i_error + obj.Ts*obj.error;

            %% saturate intergation (Not implemented) 
%             i_term = (obj.Ki) .* obj.i_error;
            
%             Iclamp = i_term > obj.i_clamp_max; %clamp index
%             obj.i_error ( Iclamp ) = obj.i_clamp_max (Iclamp)./obj.Ki(Iclamp); 
% 
%             Iclamp = i_term < obj.i_clamp_min;
%             obj.i_error ( Iclamp ) = obj.i_clamp_min (Iclamp)./obj.Ki(Iclamp); 

            %Discrete
            obj.u = (obj.Kp) .* obj.error + (obj.Kd) .* obj.d_error + (obj.Ki) .* obj.i_error ;
                
            %continuous
%             obj.u = (obj.Kp) .* obj.error + (obj.Kd) .* (-obj.R.qt) ;

            
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
