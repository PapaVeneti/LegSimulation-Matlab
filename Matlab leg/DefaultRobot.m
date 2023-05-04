classdef DefaultRobot < robot
    % DefaultRobot Summary of DefaultRobot
    % Default child of robot class. This class is for the initialization of
    % controllers that have an object of the abstract type `robot` as an
    % attribute. Matlab needs a default object to initialize such classes.

    methods(Access = public)
     %Kinematics

     %Implementation of DK_querry function
     function[Qd] =  DK_querry(obj,Q)
      Qd = [];
     end
     
     %Implementation of IK function
     function IK(obj,XV)
     end

     %Implementation of calculateJv function
     function calculateJv(obj)
     end
     
     %Implementation of gDist function
     function [dist] = gDist(q2,q1);
     end 
    
     %Implementation of MCG function
     function MCG(obj)
     end
        
     %Implementation of GroundReaction function
     function [Fr] = GroundReaction(obj)
     end
     
    end

end
