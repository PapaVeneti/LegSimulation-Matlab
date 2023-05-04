%% initialization
%tunable parameters:
Tsampling = 1e-3;
K = [10,0.1,0;10,0.1,0;10,0.1,0;];% K(i,j) link i, state j(q,qt);
Select_Controller = 2; %1 Pos; 2 Traj; 3Force

%trajectory 
TSqd = timeseries([0;0;0],0);
TSqtd = timeseries([0;0;0],0);

%position
Qd = [0;0;0];

