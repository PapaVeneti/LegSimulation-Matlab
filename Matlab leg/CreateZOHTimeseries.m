function [SCTS] = CreateZOHTimeseries(Tsampling,Tchange,Qchange)
arguments
    Tsampling (1,1)double;
    Tchange (1,:) double;
    Qchange (:,:) double
end
% CreateZOHTimeseries -> LegSimulation helper function
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