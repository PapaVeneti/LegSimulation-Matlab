classdef trajectoryGen < handle
    %trajectoryGen Trajectory generation class
    %This class creates linear and cubic trajectories given timestamps,
    %position and velocities at each waypoint. The generated trajectories
    %are simillar to the ones created by ros_control. 
    %

properties (GetAccess=public,SetAccess=private)
    %waypoints
    Q_wp  double % Position waypoint (must include current point)
    Qt_wp double % Velocity waypoint (must include current velocity)
    t_wp  double % timestamp (must include current time)
    N_wp  double % number of waypoints

    %sampling rate
    Ts (1,1)        double; % Sampling rate
    time_scale (1,1) int16; % Time scale of the sampling rate (# decimals)

    %trajectory
    qd   double % Position reference
    qdt  double % Velocity reference
    qdtt double % Acceleration reference

end

methods (Access = public)
    function obj = trajectoryGen(q_wp,qt_wp,t_wp,Ts)
        arguments
            q_wp double
            qt_wp double
            t_wp double {mustBeNonnegative}
            Ts (1,1) double {mustBePositive}
        end
        %trajectoryGen : Contructor for the class TrajGen. 
        % 
        %The constructor checks that timestamps are strictly increasing.   
        %This class generates trajectories in the closed interval [ts,tf]. 
        %
        % It accepts: 
        % - The joint angle positions of the waypoints
        % - The joint angle velocities of the waypoints
        % - The timestampts of the waypoints
        %
        % **Optional Arguments**
        % -sampling/update rate of the controller. Default value is 1e-3
        % 

        
        %check t_wp is strictly increasing and DT > Ts:
        if      ( ~ diff(t_wp) > 0 )
            error("[Trajectory Generation]: Trajectory timestamps must be increasing.")
        elseif  (  diff(t_wp) < obj.Ts )
            error("[Trajectory Generation]: Timestamps difference must be greater than sampling rate")
        end

        obj.Q_wp   = q_wp;
        obj.Qt_wp  = qt_wp;
        obj.t_wp   = t_wp;
        
        obj.N_wp      = size(q_wp,2); 
        obj.Ts        = 1e-3;
        if ~isempty(Ts)
            obj.Ts   = Ts;
        end
        
        %get time_scale (important for trajectory generation). It defines
        %how many decimals to round.
        obj.time_scale = ceil( -log10(obj.Ts) ); 

        %vector length of trajectories
        trajL = (t_wp(end)-t_wp(1))/obj.Ts;

        %round first to avoid 3.0001 becoming 4. But if we are 
        trajL = ceil( round(trajL,obj.time_scale) );        
        trajL = trajL+1; %because we include t=0 and t=t(end);

        %initialize vectors for speed
        dim = size(q_wp,1);

        obj.qd   = zeros(dim,trajL);
        obj.qdt  = zeros(dim,trajL);
        obj.qdtt = zeros(dim,trajL);
        
    end

    function linearTrajGen(obj)
        %linearTrajGen : Generate Linear Trajectories. 
        % It sends to the helper function `linearTraj` two consecutives wp 
        % to create a linear trajectory between them. 

        for i=1:obj.N_wp-1
            position_waypoints = [obj.Q_wp(:,i),obj.Q_wp(:,i+1)];
            time_waypoints     = [obj.t_wp(i)  ,obj.t_wp(i+1)  ];
            obj.linearTraj(position_waypoints ,time_waypoints );
        end

    end

    function cubicTrajGen(obj)
        %cubicTrajGen : Generate Cubic Trajectories. 
        % It sends to the helper function `cubicTraj` two consecutives wp 
        % to create a cubic trajectory between them. 

        for i=1:obj.N_wp-1
            position_waypoints = [obj.Q_wp(:,i)  ,obj.Q_wp(:,i+1) ];
            velocity_waypoints = [obj.Qt_wp(:,i) ,obj.Qt_wp(:,i+1)];
            time_waypoints     = [obj.t_wp(i)    ,obj.t_wp(i+1)   ];
            obj.cubicTraj(position_waypoints, velocity_waypoints, time_waypoints );
        end

    end

end

methods(Access=private)

    function linearTraj(obj,x,t)
        arguments
            obj trajectoryGen
            x (:,2) double
            t (1,2) double
        end
        %linearTraj : Linear trajectory between 2 waypoints.
        %no accelerations in ros -> would result in huge accelerations

        %get timestamps exectly on sampling moments. 
        % (From the constructor all DT>=Tsampling)
        t = t - mod(t,obj.Ts);
        
        %time interval for these wp
        tf = t(2)-t(1);                  
        dt = round(tf,obj.time_scale+2); %round in order to make sense (2 order of magnitutes less)
        
        %time vector
        tvector = t(1)+obj.Ts: obj.Ts: t(2); 
        
        %from time vector, get the index for each moment in the trajectory.
        % (As in matlab we index from 1, we add +1, so that:  index(1)=1)
        index =  ( tvector - obj.t_wp(1) )/obj.Ts ;
        index = index+1;
        index = int16(index);

        velocity     = ( x(:,2) - x(:,1) )/dt;
      
        obj.qd(:,index)    = x(:,1) + (obj.Ts:obj.Ts:dt).*velocity;
        obj.qdt(:,index)   = velocity.*ones(1,length(index) );

    end

    function cubicTraj(obj,x,v,t)
        arguments
            obj trajectoryGen
            x (:,2) double
            v (:,2) double
            t (1,2) double 
        end
        %cubicTraj : Cubic trajectory between 2 waypoints. 

        %get timestamps exectly on sampling moments. 
        % (From the constructor all DT>=Tsampling)
        t = t - mod(t,obj.Ts);
        
        %time interval for these wp
        tf = t(2)-t(1);                  
        dt = round(tf,obj.time_scale+2); %round in order to make sense (2 order of magnitutes less)
        
        %time vector
        tvector = t(1)+obj.Ts: obj.Ts: t(2); 
        
        %from time vector, get the index for each moment in the trajectory.
        % (As in matlab we index from 1, we add +1, so that:  index(1)=1)
        index =  ( tvector - obj.t_wp(1) )/obj.Ts ;
        index = index+1;
        index = int16(index);

        %coefficients for cubic trajectories. [column vectors]
        ao = x(:,1);
        a1 = v(:,1);
        a2 = 3/(tf^2)  * ( x(:,2) - x(:,1) ) - (2*v(:,1) + v(:,2) )/tf;
        a3 = -2/(tf^3) * ( x(:,2) - x(:,1) ) + (  v(:,2) + v(:,1) )/(tf^2);

        %trajectory time
        tj = (obj.Ts:obj.Ts:dt);

        obj.qd(:,index)    =   ao+   a1.*tj +   a2.*tj.^2+ a3.*tj.^3;
        obj.qdt(:,index)   =   a1+ 2*a2.*tj + 3*a3.*tj.^2 ;
        obj.qdtt(:,index)  = 2*a2+ 6*a3.*tj;


    end


end

end
