function mu = my_stick_slip(v)
arguments
    v (:,1) double ;
end
%my_stick_slip  : This functions implements the simulink stick slip model
%with hardcoded friction parameters.
%
% It accepts: 
%-the velocity as a column vector
%
% Output: 
%-The friction force direction along with the friction coefficient, that
%is: [ mu_x, mu_y, mu_y]. 
% The direction is oposite from the velocity, so NO minus sign is needed.


%contact model parameters:
vth = 0.001;
ms = 0.7;
mk = 0.5;

vn  = norm(v) ; %norm
dir = v/vn    ; %direction


if vn < vth
    mu = vn*ms/vth;
elseif  vn <= 1.5*vth
    mu = ms - (vn-vth)*(ms-mk) / (0.5*vth);
else
    mu  = mk;
end

mu =  - dir* mu; %resists the velocity;
end