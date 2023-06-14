classdef    TrajControl < handle
    % TrajControl Trajectory Controller class
    % This class implements a similar trajectory controller to the one
    % implemented in ros using ros_control.
    %
    % Important: 
    % 1: Robot object must have non-zero tmax.
    % 2: Call `getControllerUpToDate` before new simulation (important in 
    % consecutive simulations)

    %all the trajectories start with t=0s
    
    properties (GetAccess=public,SetAccess=protected)
        %state
        qr double  % Position reference
        qtr double % Velocity reference

        %Wp
        Qw  double % Position waypoint (must include current point)?
        Qtw double % Velocity waypoint (must include current velocity)
        Tw  double % timestamp (must include current time)
        
        %PID gains
        Kp (:,1) double ; %Proportinal Gains (in a column vector)
        Kd (:,1) double ; %Derivative Gains (in a column vector)
        Ki (:,1) double ; %Integral Gains (in a column vector)

        tmax (:,1) double; %max torque output

        %PID erros
        error double ;   %position error
        d_error double ; %derivative error
        i_error double ; %integral error

        %other controller parameters
        R robot = DefaultRobot(); %robot
        N double ;                %state dimentions
        Ts double = 1e-3;         %sampling time

        % Trajectory Gen Specifiers:
        TG  trajectoryGen;       %Trajectory Generation object
        trajMethod = 'cubic';    %Trajectory method ('linear','cubic')
   

        %control command specific
        Ntime      (1,1)  = 0; %call timer: 
        NtimeStart (1,1)  = 0; %trajectory index  

        %this variable is usefull as it let's the controller sim method
        %issue a command only when DT=Ts has passed, simulating a ZOH.

        u        double;  %command
        uhistory double;  %command history

    end

    methods (Access=public)

        % Public functions for user

        function obj = TrajControl(Robot,Ts,K)
            arguments
                Robot (1,1)   robot  = DefaultRobot; 
                Ts (1,1)      double  {mustBeNumeric,mustBePositive} = 1e-3;
                K             double  {mustBeNumeric} = 0;        
            end
            % TrajControl : Contructor for the class TrajControl. 
            % 
            % It accepts: 
            % -the robot instance Robot,
            % -the sampling rate Ts, 
            % -the gain matrix K in the form:
            %       K=[Kp,Kd,Ki], where Kp = [Kp1;Kp2;Kp3;...]

            obj.R    = Robot;
            obj.N    = Robot.dim;
            obj.Ts   = Ts;
            obj.tmax = Robot.tmax;

            obj.Kp = K(:,1);
            obj.Kd = K(:,2);
            obj.Ki = K(:,3);
            
            %initial errors =0:
            obj.reset();

            %create trajGen object
            obj.TG = trajectoryGen(obj.R.q,obj.R.qt,0,1);

            %initial target -> Current position of robot
            obj.qr  = obj.R.q;
            obj.qtr = obj.R.qt;
            
        end
      
        function generateEllipse(obj,a,b,DX,Dth,T,Npoints,Tstart)
            arguments
                obj TrajControl
                a  (1,1) double
                b  (1,1) double
                DX (2,1) double 
                Dth (1,1) double
                T   (1,1) double {mustBePositive}
                Npoints  (1,1) double {mustBePositive}
                Tstart (1,1) double {mustBeNonnegative}
    
            end
            %generateEllipse : Generate ellipse.
            %
            %The current implementation generates an ellipse based on the
            %polar form of the ellipse:
            % https://en.wikipedia.org/wiki/Ellipse#Polar_form_relative_to_center
            %
            % 
            %
            % It accepts: 
            % - Parameters a,b
            % - Center of ellipse DX = [DY,DZ]
            % - Angular rotation of ellipse dth
            % - Period of trajectory
            % - Number of points to discretize the ellipse
            % - Number of repetitions of the trajectory
            % - Starting time for the trajectory

            %the output starts from t=dt+tnow
            [X,tw] = obj.generateVerticalEllipse(a,b,DX,Dth,T,Npoints);
            
            obj.generateQ(X,tw,Tstart);

        end
        
        function generateQ(obj,x,twp,Tstart)
            arguments
                obj TrajControl
                x  double
                twp(1,:) double
                Tstart (1,1) double {mustBeNonnegative}
            end
            %generateQ : Create joint-angle waypoints given cartesian wp.
            %The current implementation, sets the desired velocity as the
            %mean of the mean velocities of two consecutives wp only when
            %they are pointing in the same direction. 
            %
            %Also, current implementation includes starting point
            %
            % It accepts: 
            % - The cartesian positions of the waypoints
            % - The timestampts of the waypoints 
            %   (Must be strictly increasing and diff(twp) >= Tsampling)

            Npoints = length(twp);
            %% 1. Set timestamps
            twp = [Tstart twp+Tstart];                    %include current time
            %% 2. Position WP:
            qw = zeros(obj.R.dim, Npoints+1 ); %include current time          
            qw(:,1) = obj.R.q; %initial point of trajectory is the current position
            
            %if back to back trajectories
            if obj.TG.t_wp(end) == Tstart
                qw(:,1) = obj.TG.Q_wp(:,end);
            end

            for i=1:Npoints
                obj.R.IK(x(:,i)); %find possible solutions

                if isempty( obj.R.sol) 
                    error('NO IK')
                end

                qw(:,i+1)  = obj.findBestSol( qw(:,i) ); 
             
            end
            
            %% 3. Set velocities 
            % Mean velocity only when velocities are pointing in the same
            % direction            

            % find mean velocity (along rows)  % size(qw) = [dim,Npoints+1]
            Ul =  diff(qw,1,2) ./ diff(twp) ;  % size(Ul) = [dim,Npoints-1] 
            Ur =  Ul(:,2:end)  ;              
            Ul =  Ul(:,1:end-1);

            meanVel               = (Ur+Ul)/2; % size(meanVel) = [dim,Npoints-1]

            %loggical array
            VelActivation         = sign(Ul.*Ur)>0 ; %same direction
            VelActivationExt      = [ ones(obj.R.dim,1) ,VelActivation ,ones(obj.R.dim,1) ];
            
            % qtw
            qtw                   = [obj.R.qt,  meanVel , zeros(obj.R.dim,1) ];
            qtw                   = qtw .* VelActivationExt; 

            %% 4. Create Trajectories    
            obj.setWP(qw,qtw,twp);

        end

        function setTrajGenMethod(obj,method)
            arguments
                obj (1,1) TrajControl
                method (1,:) char 
            end
            %setTrajGenMethod : Set trajectory generation method. 
            %Choose between 'cubic' and 'linear' trajectories.
            %Default trajectories are cubic.
            
            if ( ~strcmp(method,'linear') ) && ( ~strcmp(method,'cubic') ) 
                error("[Trajectory Controller]: trajectory methods include: {'linear','cubic'}")
            end

            switch method
                case 'linear'
                obj.trajMethod = 'linear';
                case 'cubic'
                obj.trajMethod = 'cubic';
                otherwise 
                obj.trajMethod = 'cubic';
            end
        end
        
        % Public functions for simulation class
        function  syncErrors(obj,error,d_error,i_error)
            arguments
                obj TrajControl  
                error double   {mustBeReal}
                d_error double {mustBeReal}
                i_error double {mustBeReal}
            end
            %syncErrors : Synchronize errors, as in ros there is one pid

            obj.error    = error   ;
            obj.d_error  = d_error ;
            obj.i_error  = i_error ;

        end

        function reset(obj)
            arguments
                obj (1,1) TrajControl
            end
            %reset : Reset discrete simulation parameters and errors
                obj.Ntime = 0;
                obj.error =  zeros(obj.N,1);
                obj.d_error= zeros(obj.N,1);
                obj.i_error= zeros(obj.N,1);
                obj.u      = zeros(obj.N,1);
        end
        
        function getControllerUpToDate(obj,tnow)
            arguments
                obj  (1,1) TrajControl
                tnow (1,1) double {mustBeNonnegative}
            end
            %getControllerUpToDate : Important to call before new simulation. 
            % 
            % It accepts: 
            % Tnow = the starting time of the new simulation
                obj.Ntime      = floor(tnow/obj.Ts);
                obj.NtimeStart = obj.Ntime; 
                
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

            %this sets the state, and calculates the new M
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

    methods (Access=protected) %to get documentation
       
        function command(obj)
            %command : Calculate position controller command. It
            %also updates the input history        

            %% PID 
            traj_index = obj.Ntime - obj.NtimeStart;
            qd  = obj.TG.qd (:,traj_index);
            qtd = obj.TG.qdt(:,traj_index);

            %errors
            obj.error = obj.R.Dist(qd); %correct distance function
            obj.d_error = qtd - obj.R.qt;
            obj.i_error = obj.i_error + obj.Ts*obj.error;

            %discrete
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
        
        function setWP(obj,qwp,qtwp,twp)
        %setWP :  Generates Trajectories based on the waypoints   
        %This class generates trajectories in the closed interval [ts,tf]. 
        %
        % It accepts: 
        % - The joint angle positions of the waypoints
        % - The joint angle velocities of the waypoints
        % - The timestampts of the waypoints 
        %   (Must be strictly increasing and diff(twp) >= Tsampling)

            obj.Qw  = qwp;
            obj.Qtw = qtwp;
            obj.Tw  = twp;
            
            %create trajectory gen object:
            obj.TG = trajectoryGen(qwp,qtwp,twp,obj.Ts);

            %generate trajectory:
            switch obj.trajMethod
                case 'cubic'
                    obj.TG.cubicTrajGen();
                case 'linear'
                    obj.TG.linearTrajGen();
            end
            
        end
        
        function BestSol = findBestSol(obj,q_start)
        %findBestSol : This function finds the best solution from the set
        %of IK solutions. It assumes that the robot starts from the
        %previous trajectory waypoint and ends in the desired position
        %
        % **The current implementation is not optimal:**
        % Best solution is the solution with the smallest euclidean
        % distance in the joint angle space.
   
        %find distance
        D = obj.R.gDist( obj.R.sol ,q_start ) ;

        %calculate norm (diagonal elements of (D^T * D) ):
        Qdist = diag( (D') * D ) ;     

        %get solution with minumum norm
        [~,best_sol] = min(Qdist) ;

        BestSol = obj.R.sol(1:obj.N,best_sol);
       
        end

        %Extra 
        function [X,tw] = generateVerticalEllipse(obj,a,b,DX,Dth,T,Npoints)
            % This function generates a set of points that describe an
            % ellipse in the YZ plane with the center at 
            % P_w = [LB0+L12,DY,DZ] in the polar form of the ellipse:
            % https://en.wikipedia.org/wiki/Ellipse#Polar_form_relative_to_center
            %
            %Important:
            %1: Output is:
            % t = [dt:dt:T] , where dt = T/(NumberOfPoints-1)
            % X = [X(dth):...] 
            % The first point is not for t=0,th=0!
            %
            %2: The robot workspace for the ellipse is:
            %  z > 0.045/2;
            %  x = 0.16763;
            %  R = 0.4761
            %  r = 0.182839

            X_000 = obj.R.DK_querry(zeros(obj.R.dim,1));

            NumberOfPoints = Npoints;
            tw   = linspace(T/(NumberOfPoints-1),T,NumberOfPoints);
            th   = linspace(2*pi/(NumberOfPoints-1),2*pi,NumberOfPoints);
            r_th = a*b./sqrt(b^2*cos(th-Dth).^2 + a^2*sin(th-Dth).^2  );
            X    = zeros(3,NumberOfPoints);

            %Populate Cartesian setPoints
            X(1,1:NumberOfPoints) = X_000(1);
            X(2,1:NumberOfPoints) = r_th .* cos(th)+DX(1);
            X(3,1:NumberOfPoints) = r_th .* sin(th)+DX(2);

        end
    end


end