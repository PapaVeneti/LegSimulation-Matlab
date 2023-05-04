classdef legRobot <robot
    % legRobot Summary of LegRobot:
    % Child of robot class. In contains the geometric properties of the
    % leg. Also, it contains the implementations of the basic kinematic
    % functions of the parent class. 

    properties(Constant,Access=protected)
       LB0 = 0.05555; %LB0
       L01 = 0.07763; %L01
       L12 = 0.11208; %L12
       L23 = 0.2531;  %L23
       L3E = 0.223;   %L3E
       footD = 0.045; %footD
       H = 0.4;       %Height of robot

       %contact
       Kp = 1e4;
       Kd = 1e2;
    end
    
    %public for easy testing:
    properties (GetAccess=public,SetAccess=protected)
        nSols = 1; %number of solutions from the IK
               Jv; %Geometric Jacobian

    end

    methods (Access=public) 

        function obj =legRobot(input)
            arguments
                input.q (:,1) double {mustBeNumeric}
                input.qt (:,1) double {mustBeNumeric}
                input.tmax (:,1) double {mustBeNumeric,mustBePositive}
                input.ground (1,1) logical 
            end
            % Contructor for the class legRobot. 
            % 
            % 
            % It accepts: 
            %
            % **Optional Arguments**
            % -the initial conditions for the joint positions,
            % -the initial conditions for the joint velocities
            % -the maximum torque for each joint as seen by their respective link
            % -a flag to select wheter to simulate ground reaction forces
            %
            %  The initialiation is done in the following form: 
            %
            % r = legRobot(tmax=[10,10,20],q=[1;3;3],qt=[3,2,1],ground=true)
            %
            % Default values for q=0,qt=0,tmax=1e9, ground=false

            obj.dim=3;
            obj.tmax = 1e9*ones(obj.dim,1); %unlimited torque
            obj.q    = zeros(obj.dim,1);
            obj.qt   = zeros(obj.dim,1);
            obj.simulate_ground   = false;

            if exist('input','var') 
                if isfield(input,'tmax')
                    obj.tmax(1:obj.dim,1) = input.tmax(1:obj.dim,1);
                end
                if isfield(input,'q')
                    obj.q(1:obj.dim,1)    = input.q(1:obj.dim,1);
                end
                if  isfield(input,'qt')
                    obj.qt(1:obj.dim,1)   = input.qt(1:obj.dim,1);
                end
                if  isfield(input,'ground')
                    obj.simulate_ground   = input.ground ;
                end
            end
       
            obj.sol = zeros(obj.dim,4); %at most 4 solutions 

            obj.Jv = zeros(obj.dim);
            obj.DK();
            obj.MCG();
        end
        
        % --- Kinematics ---:
        % Direct Kinematics:
        function [Qd] =  DK_querry(obj,Q)
           Qd(1,1) = obj.LB0 + obj.L12*cos(Q(1)) - obj.L23*sin(Q(1))*sin(Q(2)) - obj.L3E*sin(Q(1))*sin(Q(2) + Q(3)); 
           Qd(2,1) = obj.L01 + obj.L3E*cos(Q(2) + Q(3)) + obj.L23*cos(Q(2)); 
           Qd(3,1) = obj.H + obj.L12*sin(Q(1)) + obj.L23*cos(Q(1))*sin(Q(2)) + obj.L3E*cos(Q(1))*sin(Q(2) + Q(3));
        end

        % Inverse Kinematics:
        function IK(obj,Xw)
            obj.sol = zeros(obj.dim,4); %reset solutions

            % reset number of solutions
            obj.nSols = 0;
            nTh1 = 0;
            
            % set positions as seen in the 0th frame
            x0 = Xw(3) -obj.H;
            y0 = obj.LB0-Xw(1);
            z0 = -Xw(2);
            
            
            %% Get th1 ---------------------------:
            
            % Check if solution exists:
              if (x0*x0 + y0*y0 < obj.L12^2 ) 
            error( "Solution to IK doesn't exist: x0^2 + y0^2 < L12^2 "  );
              end
            
            % discriminant
            D = 4*(obj.L12^2)*(x0^2) - 4*( (obj.L12^2)-(y0*y0) )*( (x0*x0)+(y0*y0) ); 
            
            th1 = asin( ( 2*obj.L12*x0+sqrt(D) ) / ( 2*x0*x0+2*y0*y0) ) ;%+sqrt(D)  
            
            %because it may pi-th1 (which is out of bounds)
            if ( abs(obj.L12-x0*sin(th1)+y0*cos(th1) ) < 1e-8)  
                nTh1 = nTh1+1;
                obj.sol(1,nTh1) = th1; %SOLS[0][nTh1] = th1; 
            end
            
            if (abs(D)>1e-6) % else identical solutions
                th1 = asin( ( 2*obj.L12*x0-sqrt(D) ) / ( 2*x0*x0+2*y0*y0) ); %-sqrt(D)
                if ( abs(obj.L12-x0*sin(th1)+y0*cos(th1) ) < 1e-8)   
                    %i want to group the solutions with the same th1 -> [th1 th1 th2 th2;...;...]
                    nTh1 = nTh1+1;
                    obj.sol(1,2*(nTh1-1)+1) = th1; % SOLS[0][2*(nTh1)] = th1; (it accepts index=0)
                end
            
            end            
            
            %% Get th2 and th3 -------------------:
            if nTh1 == 1
                obj.sol(:,3:4) = [];
            end

            for i_th=1:nTh1 %(int i = 0; i<nTh1 ;i++)
                th1 = obj.sol(1,2*(i_th-1)+1 ); %SOLS[0][2*i];    
            
                K1 = x0*cos(th1)+y0*sin(th1);
                K2 = -z0 - obj.L01;
            
            %     // Check if solution exists
                if (K1*K1 + K2*K2 > (obj.L23 + obj.L3E)^2 )
                    continue                %check the second solution
                end
                
                if (K1*K1 + K2*K2 < obj.L23^2 + obj.L3E^2 + 2*obj.L23*obj.L3E*cos(2.229)  )
                    continue
                end
            
                % system definitions
                th3 = acos( (K1*K1 + K2*K2 - obj.L23^2 - obj.L3E^2) / (2*obj.L23*obj.L3E)  );
            
                det = -( obj.L23^2+obj.L3E^2 + 2*obj.L23^2*cos(th3)) ;
            
                A = obj.L3E*sin(th3);
                B = obj.L23+obj.L3E*cos(th3) ; 
            
                % 1st solution
                c2 = -(  A*K1 + B*K2 ) / det ;
                s2 =  ( -B*K1 + A*K2 ) / det ;
            
                th2 = atan2(s2,c2);
            
                obj.nSols = obj.nSols+1;

                obj.sol(1,obj.nSols ) = th1;
                obj.sol(2,obj.nSols ) = th2;
                obj.sol(3,obj.nSols ) = th3;
            
            
                % 2nd solution
                c2 = -( -A*K1 + B*K2 ) / det ;
                s2 =  ( -B*K1 - A*K2 ) / det ;
            
                th2 = atan2(s2,c2);
            
                obj.nSols = obj.nSols+1;

                obj.sol(1,obj.nSols ) = th1;
                obj.sol(2,obj.nSols ) = th2;
                obj.sol(3,obj.nSols ) = -th3;
            
                
            end

            if obj.nSols == 0
                obj.sol = [];
            end


        end

        %Differential Kinematics Jv=Jr
        function calculateJv(obj)
            
            Q1 = obj.q(1);
            Q2 = obj.q(2);
            Q3 = obj.q(3);

            obj.Jv(1,:) = [- obj.L12*sin(Q1) - obj.L23*cos(Q1)*sin(Q2) - obj.L3E*sin(Q2 + Q3)*cos(Q1), -sin(Q1)*(obj.L3E*cos(Q2 + Q3) + obj.L23*cos(Q2)), -obj.L3E*cos(Q2 + Q3)*sin(Q1)];
            obj.Jv(2,:) = [                                                                         0,          - obj.L3E*sin(Q2 + Q3) - obj.L23*sin(Q2),         -obj.L3E*sin(Q2 + Q3)];
            obj.Jv(3,:) = [  obj.L12*cos(Q1) - obj.L23*sin(Q1)*sin(Q2) - obj.L3E*sin(Q2 + Q3)*sin(Q1),  cos(Q1)*(obj.L3E*cos(Q2 + Q3) + obj.L23*cos(Q2)),  obj.L3E*cos(Q2 + Q3)*cos(Q1)];
        end
        
        % --- Dynamics ---:
        function MCG(obj)
        
            Q1 = obj.q(1);  Q2 = obj.q(2) ; Q3 = obj.q(3) ;    
            Q1t= obj.qt(1); Q2t= obj.qt(2) ; Q3t= obj.qt(3) ;
    
            %Mass matrix
            obj.M(1,1) =(496303210401*cos(Q3))/50000000000000 - (17752137377054734447317641523*cos(2*Q2 + 2*Q3))/4503599627370496000000000000000 - (496303210401*cos(2*Q2 + Q3))/50000000000000 - (102811325585251813697606066584270844200265338678741985009^(1/2)*cos(2*Q2 - atan(43735884162909205953625/10139591983079939796779111872)))/720575940379279360000000000000 + 141298574669242556727740555789/4503599627370496000000000000000;
            obj.M(1,2) =(274721769171*cos(Q2 + Q3))/62500000000000 + (618478810978159917158793317358601653194747934816534425129^(1/2)*cos(Q2 - atan(29348096677242789894221875/24869216909009848040054913248)))/1801439850948198400000000000000;
            obj.M(1,3) = (274721769171*cos(Q2 + Q3))/62500000000000;
            
            obj.M(2,1) = (274721769171*cos(Q2 + Q3))/62500000000000 + (618478810978159917158793317358601653194747934816534425129^(1/2)*cos(Q2 - atan(29348096677242789894221875/24869216909009848040054913248)))/1801439850948198400000000000000;
            obj.M(2,2) = (496303210401*cos(Q3))/25000000000000 + 656005633056180630430383741/17592186044416000000000000000;
            obj.M(2,3) = (496303210401*cos(Q3))/50000000000000 + 280850090332252565083362807/35184372088832000000000000000;
            
            obj.M(3,1) = (274721769171*cos(Q2 + Q3))/62500000000000;
            obj.M(3,2) = (496303210401*cos(Q3))/50000000000000 + 280850090332252565083362807/35184372088832000000000000000;
            obj.M(3,3) = 280850090332252565083362807/35184372088832000000000000000;
                        
            %C matrix
            obj.C(1,1) = (496303210401*Q2t*sin(2*Q2 + Q3))/25000000000000 - (496303210401*Q3t*sin(Q3))/50000000000000 + (496303210401*Q3t*sin(2*Q2 + Q3))/50000000000000 - (349887073303273647629*Q2t*cos(2*Q2))/2882303761517117440000000000 + (158431124735624059324673623*Q2t*sin(2*Q2))/5629499534213120000000000000 + (17752137377054734447317641523*Q2t*sin(2*Q2 + 2*Q3))/2251799813685248000000000000000 + (17752137377054734447317641523*Q3t*sin(2*Q2 + 2*Q3))/2251799813685248000000000000000;
            obj.C(1,2) = - (274721769171*Q3t*sin(Q2 + Q3))/31250000000000 - (Q2t*(39591579432610528505848922112*sin(Q2 + Q3) - 5*618478810978159917158793317358601653194747934816534425129^(1/2)*cos(Q2 + atan(24869216909009848040054913248/29348096677242789894221875))))/9007199254740992000000000000000;
            obj.C(1,3) = -(274721769171*Q3t*sin(Q2 + Q3))/62500000000000;
            
            obj.C(2,1) = -Q1t*((17752137377054734447317641523*sin(2*Q2 + 2*Q3))/4503599627370496000000000000000 + (496303210401*sin(2*Q2 + Q3))/50000000000000 - (102811325585251813697606066584270844200265338678741985009^(1/2)*cos(2*Q2 + atan(10139591983079939796779111872/43735884162909205953625)))/720575940379279360000000000000);
            obj.C(2,2) = -(496303210401*Q3t*sin(Q3))/25000000000000;
            obj.C(2,3) = -(496303210401*Q3t*sin(Q3))/50000000000000;
            
            obj.C(3,1) = -Q1t*((17752137377054734447317641523*sin(2*Q2 + 2*Q3))/4503599627370496000000000000000 - (496303210401*sin(Q3))/100000000000000 + (496303210401*sin(2*Q2 + Q3))/100000000000000);
            obj.C(3,2) = (496303210401*Q2t*sin(Q3))/50000000000000;
            obj.C(3,3) = 0;


            % G matrix
            obj.G(1,1) = (1189884493827*cos(Q1))/1000000000000 + (387492057*sin(Q1))/100000000000 + (1956827187*cos(Q2)*sin(Q1))/500000000000 - (604164195783*sin(Q1)*sin(Q2))/500000000000 - (192364065351*cos(Q2)*sin(Q1)*sin(Q3))/500000000000 - (192364065351*cos(Q3)*sin(Q1)*sin(Q2))/500000000000;
            obj.G(2,1) = (981*cos(Q1)*(196089771*cos(Q2 + Q3) + 379294469163607978^(1/2)*cos(Q2 - atan(1994727/615865643))))/500000000000;
            obj.G(3,1) = (192364065351*cos(Q2 + Q3)*cos(Q1))/500000000000;
        end
        
        % --- General Distance ---:
        function [dist] = gDist(obj,qd,qo)
            %general distance function for this robot
            %
            % It accepts: 
            % -the desired joint angles,
            % -the starting joint angles, 

            %how many differences i have to find
            Cols = size(qd,2); 
            Rows = size(qd,1);
            dist = zeros(Rows,Cols);

            %q1,q3 have limits:
            
            dist(1,:) = qd(1,:)-qo(1);
%           dist(2,:) = qd(2,:)-qo(2);
            dist(3,:) = qd(3,:)-qo(3);

            %joint 2 doesn't have limits
            dist(2,:) = obj.angularDist( qd(2,:), qo(2) );

        end
        
        % --- Ground reaction ---:
        function [h] = GroundReaction(obj)
            %Get the ground reaction wrench. 
            %The direction is: from EE to environment
            % 
            
            %the sim function calculates these
            %1: direct kinematics to find end effector:
%             obj.DK(); 

            %2: calculate Jacobian
%             obj.calculateJv();

            %3: get velocity in y direction
            obj.xt = obj.Jv * obj.qt; 
    
            %4: check if there is contact and calculate wrench:
            d = obj.x(3,1) - (obj.footD/2);
            
            if d>0 %distance from the wall to EE
                h = zeros(obj.dim,1);
            else
                Fn = -obj.Kp*d ; %from environment to EE (expressed in World Coordinate)

                if obj.xt(3) < 0 %penetretion is increasing -> simscape applies damping
                    Fn = Fn  -obj.Kd* obj.xt(3) ;
                end

                Ft = my_stick_slip(obj.xt(1:2)) .*abs(Fn);
               

                h = -[Ft;Fn]; %wedge from EE to environment
                
            end


        end

    end
end

