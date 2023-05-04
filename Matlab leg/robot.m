classdef robot < handle
    % robot Summary of robot
    % Parent Class for all robots. This documentations is "inherited" by
    % children classes.

    %How to save contact force better

%public properties for easy testing:
properties (Access = public)
        % joint space
        q (:,1) double ; %joint positions
        qt(:,1) double ; %joint velocities

        % end_effector cartesian space
        x(:,1) double   ; %end effector cartesian position
        xt(:,1) double;   %end effector cartesian velocity

        simulate_ground logical ; %model ground reaction flag
end

properties (SetAccess = protected,GetAccess=public)
        tmax (:,1) double;   %maximum torque at each joint(as seen by link)

        %nDofs
        dim double; %number of states of the robot

        %solutions  
        sol double  %array containing IK solutions

        %Dynamics
        M double %mass matrix
        C double %centrifugal forces matrix 
        G double %gravity term
        
        Fev_history double = []; %Interaction Force History

end

methods (Abstract,Access = public)
     
     %get Direct Kinematics for input Q 
     [Qd] =  DK_querry(obj,Q)

     %Inverse Kinematics 
     IK(obj,XV)

     %Inverse Kinematics 
     calculateJv(obj)

     %General Distance Function to q2 from q1 
     [dist] = gDist(obj,q2,q1);
     
     %Get Dynamics Matrices 
     MCG(obj)
     
     %Ground reaction
     [Fr] = GroundReaction(obj)
end

methods (Access = public)
     function [yt] = sim(obj,t,y)
     % function for the simulation of the robot in ode solvers. It also
     % saves the contact force. 
     % Important: Must be called first in the sim
     %
     % All the controllers have q,qt,x,xt,M,C,G and Jv available.
     %The implementations is as follows:
     %1. Get state and save it to the robot states (q,qt)
     %2. Updates Matrices (M,C,G)
     %3. Updates Jv
     %4. Updates cartesian states
     %5. Gets Reaction Forces and the rest of the dynamics.

            % Get State
            obj.q  = y(1:obj.dim,1);
            obj.qt = y(obj.dim+1:2*obj.dim,1);
       
            %Update Matrices
            obj.MCG();

            %Update Matrices
            obj.calculateJv();

            %Cartesian states
            obj.DK();
            obj.xt = obj.Jv*obj.qt;

            %Ground Reaction Forces
            if obj.simulate_ground
                h = obj.GroundReaction(); %updates Jv too
                tr = (obj.Jv') * h;
            else
                h = zeros(obj.dim,1) ; 
                tr = zeros(obj.dim,1); 
            end
            obj.Fev_history = [obj.Fev_history,[t;-h] ];

            %output
            yt(1:obj.dim,1) = obj.qt;
            yt(obj.dim+1:2*obj.dim,1) = obj.M\( - obj.C*obj.qt -obj.G -tr);
        
    end
    
     function [dist] = Dist(obj,qd)
        % get distance of the robot from the current state to the desired
        % state
        dist = obj.gDist(qd,obj.q);
    end
        
     function DK(obj)
        % get end effector postion from the current state 
           obj.x     =  obj.DK_querry(obj.q);
     end 
end

methods (Static,Access = public)
   
    function [d] = angularDist(qf,qi)
        %calculate angular distance in the interval [-pi,pi]
        arguments 
            qf(1,:) double
            qi(1,1) double
        end
            AAD = ( qf(1,:)-qi ) +pi; %Angular Distance
            d = mod(AAD,2*pi)-pi;
            
    end
    
end

end
