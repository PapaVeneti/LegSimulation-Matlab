clear tbag t1bag t2bag t3bag u1bag u2bag u3bag 

JointStates = true;
PosControl = false;
TrajControl = false;
contact = true;

if JointStates 
%%  joint_states
% DATA1 = readtable('rosBags/legKp10Kd1Ki1_qd000JS.txt');
% t_offset =-0.0035; %kp10kd1ki1

DATA1 = readtable('rosBags/ForceNewJS.txt');
% DATA1 = readtable('rosBags/legKp10Kd1_qd000_JS_new.txt');
t_offset = 1.33155;%+0.01945; %ForceJS
t_offset = 2.488+0.221; %ForceNewJS
% t_offset = -5e-3;
% t_offset = +0.004; %simpleEffortTest_JS

tbag = DATA1.x_time; tbag = (tbag-tbag(1))*1e-9; %ns to s
tbag = tbag - t_offset; % offset time
t1bag= tbag;t2bag= tbag;t3bag= tbag;
tu1bag= tbag;tu2bag= tbag;tu3bag= tbag;

q1bag = DATA1.field_position0;
w1bag = DATA1.field_velocity0;
u1bag = DATA1.field_effort0;

q2bag = DATA1.field_position1;
w2bag = DATA1.field_velocity1;
u2bag = DATA1.field_effort1;

q3bag = DATA1.field_position2;
w3bag = DATA1.field_velocity2;
u3bag = DATA1.field_effort2;

elseif  PosControl
%% controller/state
%offsets
% t1_offset =-0.01; %legPosKd10Kp1


DATA1 = readtable('rosBags/legKp10Kd1qd000_j1.txt');
DATA2 = readtable('rosBags/legKp10Kd1qd000_j2.txt');
DATA3 = readtable('rosBags/legKp10Kd1qd000_j3.txt');


t1_offset =-0.01; %legPosKp10Kd0_1
t2_offset =-0.01; %legPosKp10Kd0_1
t3_offset =-0.01; %legPosKp10Kd0_1

%1
t1bag = DATA1.x_time; t1bag = (t1bag-t1bag(1))*1e-9; %ns to s
t1bag = t1bag - t1_offset; % offset time

q1bag = DATA1.field_process_value;
u1bag = DATA1.field_command;
w1bag = DATA1.field_process_value_dot;

%2
t2bag = DATA2.x_time; t2bag = (t2bag-t2bag(1))*1e-9; %ns to s
t2bag = t2bag - t2_offset; % offset time

q2bag = DATA2.field_process_value;
u2bag = DATA2.field_command;
w2bag = DATA2.field_process_value_dot;


%3
t3bag = DATA3.x_time; t3bag = (t3bag-t3bag(1))*1e-9; %ns to s
t3bag = t3bag - t3_offset; % offset time

q3bag = DATA3.field_process_value;
u3bag = DATA3.field_command;
w3bag = DATA3.field_process_value_dot;
elseif TrajControl
%% traj control
DATA1 = readtable('rosBags/legTrajKp10Kd1.txt');
DATA2 = readtable('rosBags/legTrajKp10Kd1JS.txt');

t_offset =-0.; %kp10kd1ki1
tu_offset =0.0065;


tbag = DATA1.x_time; tbag = (tbag-tbag(1))*1e-9; %ns to s
tbag = tbag - t_offset; % offset time
t1bag= tbag;t2bag= tbag;t3bag= tbag;

tubag = DATA2.x_time; tubag = (tubag-tubag(1))*1e-9; %ns to s
tubag = tubag - tu_offset; % offset time
tu1bag= tubag;tu2bag= tubag;tu3bag= tubag;

q1bag = DATA1.field_feedback_actual_positions0;
w1bag = DATA1.field_feedback_actual_velocities0;
u1bag = DATA2.field_effort0;

q2bag = DATA1.field_feedback_actual_positions1;
w2bag = DATA1.field_feedback_actual_velocities1;
u2bag = DATA2.field_effort1;


q3bag = DATA1.field_feedback_actual_positions2;
w3bag = DATA1.field_feedback_actual_velocities2;
u3bag = DATA2.field_effort2;

%desired
q1dbag = DATA1.field_feedback_desired_positions0;
w1dbag = DATA1.field_feedback_desired_velocities0;

q2dbag = DATA1.field_feedback_desired_positions1;
w2dbag = DATA1.field_feedback_desired_velocities1;

q3dbag = DATA1.field_feedback_desired_positions2;
w3dbag = DATA1.field_feedback_desired_velocities2;



end
%% contact 
if contact

DATA2 = readtable('rosBags/ForceNewCF.txt');
tc_offset =1.33155;%+0.01945; %kp10kd1ki1
tc_offset =1.33155;%+0.46845; %kp10kd1ki1 %ForceJS
tc_offset = 1.7800+0.02; %ForceNewJS
tc_offset = +0.004; %simpleEffortTest_CF
tc_offset =3.159 - 0.013 - 0.011 ; %3.159; %ForceNewCF


tcbag = DATA2.x_time; tcbag = (tcbag-tcbag(1))*1e-9; %ns to s
tcbag = tcbag - tc_offset; % offset time

Fxbag = DATA2.field_data0;
Fybag = DATA2.field_data1;
Fnbag = DATA2.field_data2;
end