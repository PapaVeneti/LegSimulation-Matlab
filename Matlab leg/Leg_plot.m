%% Description:
%Script for plotting simulation data. For basic simulations, the user
%changes the first section. 

plot_gazebo = true;

%% Set Plot Limits:
% set xlim: ------------------------------------------------------
tplot = ts_sim;
% tplot = [0.8,2.5];
% ts = tplot;

% Default title: -------------------------------------------------
header1 = ['K_P = ',num2str(Kp(1)),', K_D = ',num2str(Kd(1)),', K_I = ',num2str(Ki(1))] ;
header2 = ['K_P = ',num2str(Kp(2)),', K_D = ',num2str(Kd(2)),', K_I = ',num2str(Ki(2))] ;
header3 = ['K_P = ',num2str(Kp(3)),', K_D = ',num2str(Kd(3)),', K_I = ',num2str(Ki(3))] ;
 
suffix = ' $';
% suffix = [newline,'Qd = [',num2str(Qd(1)) ,',',num2str(Qd(2)) ,',',num2str(Qd(3)) ,  '], Kp =', num2str(K(1)), ', Kd = ', num2str(K(2))];

%plot extra options: ---------------------------------------------
collision = false;
limit     = false; 
reference = false; %add a trajectory reference figure:
% tlim=0;

% Reached limit: -------------------------------------------------
% tlim = 0.139734; %step
% tplot = [0,tlim+0.1];
% limit = false;

% % $Gazebo \  Collision$: -------------------------------------------------
% tcollision = 0.193521;
% tplot = [0,max(tlim,tcollision)+0.3];

%% referenced trajectories
if strcmp('traj',Simulation_type)
figure('Name','Referenced Trajectories')
subplot(3,1,1)
title_txt = ['$ Q_{d,1}(t) \ | \ ',header1,suffix]; 
p1 = plot(t1bag,q1dbag,'b',DisplayName="$Ros\ Control\ Reference \ Trajectory$",LineWidth=2.5,LineStyle="-.");
hold on
p2 = plot(TSqd(:,1),TSqd(:,2),'r',DisplayName="$Matlab \ Reference \ Trajectory$",LineWidth=2,LineStyle="--");
hold on
legendVector = [p1,p2];
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{d,1} \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=15)
xlim(tplot)
grid on

subplot(3,1,2)
title_txt = ['$ Q_{d,2}(t) \ | \ ',header1,suffix]; 
p1 = plot(t1bag,q2dbag,'b',DisplayName="$Ros\ Control\ Reference \ Trajectory$",LineWidth=2.5,LineStyle="-.");
hold on
p2 = plot( TSqd(:,1),TSqd(:,3),'r',DisplayName="$Matlab \ Reference \ Trajectory$",LineWidth=2,LineStyle="--");
hold on
legendVector = [p1,p2];
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{d,2} \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=15)
xlim(tplot)
grid on

subplot(3,1,3)
title_txt = ['$ Q_{d,3}(t) \ | \ ',header1,suffix]; 
% p1 = plot(t1bag,q3dbag,'b',DisplayName="$Ros\ Control\ Reference \ Trajectory$",LineWidth=2.5,LineStyle="-.");
hold on
p2 = plot( TSqd(:,1),TSqd(:,4),'r',DisplayName="$Matlab \ Reference \ Trajectory$",LineWidth=2,LineStyle="--");
hold on
legendVector = [p1,p2];
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{d,3} \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=15)
xlim(tplot)
grid on
end

%% compare q
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultAxesFontSize',13);

legendVector = [];
figure('Name','Joint Positions')
subplot(3,1,1)
title_txt = ['$ q_1(t) \ | \ ',header1,suffix]; 
if plot_gazebo 
p1 = plot(t1bag,q1bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end
p2 = plot(t,q1v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(q1simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
hold on
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot(TSqd(:,1),TSqd(:,2),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
% hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_1 \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tplot)
grid on

legendVector = [];
subplot(3,1,2)
title_txt = ['$ q_2(t) \ | \ ',header2,suffix]; 
if plot_gazebo 
p1 = plot(t2bag,q2bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end
p2 = plot(t,q2v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(q2simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
hold on
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot(TSqd(:,1),TSqd(:,3),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_2 \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tplot)
grid on

legendVector = [];
subplot(3,1,3)
title_txt = ['$ q_3(t) \ | \ ',header3,suffix]; 
if plot_gazebo 
p1 = plot(t3bag,q3bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end
p2 = plot(t,q3v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(q3simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
hold on
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot(TSqd(:,1),TSqd(:,4),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_3 \ [rad]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tplot)
grid on

%% compare w
twplot = tplot;% [0.2 1];

legendVector = [];
figure('Name','Joint Velocities')
subplot(3,1,1)
title_txt = ['$ q_{t,1} (t) \ | \  ',header1,suffix]; 
if plot_gazebo 
p1 = plot(t1bag,w1bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(t,q1tv,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(w1simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot((TSqtd.Time'),TSqd.Data(1,:),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{t,1} \ [rad/s]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(twplot)
grid on

legendVector = [];
subplot(3,1,2)
title_txt = ['$ q_{t,2} (t) \ | \  ',header2,suffix]; 
if plot_gazebo 
p1 = plot(t2bag,w2bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(t,q2tv,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(w2simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot((TSqtd.Time'),TSqd.Data(2,:),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{t,2} \ [rad/s]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(twplot)
grid on

legendVector = [];
subplot(3,1,3)
title_txt = ['$ q_{t,3}(t) \ | \  ',header3,suffix]; 
if plot_gazebo 
p1 = plot(t3bag,w3bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

 p2 = plot(t,q3tv,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(w3simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
if limit
p4 = xline(tlim,'r',DisplayName="$Reached \ Joint \ Limit$",LineWidth=1,LineStyle="--");
hold on
legendVector = [legendVector,p4];
end
if collision 
p5 = xline(tcollision,'b',DisplayName="$Ground \  Collision$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p5];
hold on
end
if reference 
p6 = plot((TSqtd.Time'),TSqd.Data(3,:),'k',DisplayName="$Reference$",LineWidth=1,LineStyle="--");
legendVector = [legendVector,p6];
hold on
end
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$q_{t,3} \ [rad/s]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(twplot)
grid on


%% compare F
tfplot = tplot;
% tfplot = [0 0.07];

legendVector = [];
figure('Name','Contact Forces')
subplot(3,1,1)
title_txt = ['$ Contact\ Forces\ for \ h_{desired}=100N',suffix]; 
if plot_gazebo 
p1 = plot(tcbag,Fnbag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(F(1,:),F(end,:),'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="-");
hold on
p3 = plot(out.Fn.Time,out.Fn.Data,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle="--");
hold on
legendVector = [legendVector,p2,p3];
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$F_n\ [N]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tfplot)
grid on

legendVector = [];
subplot(3,1,2)
title_txt = ['$ F_n(t)',suffix];
if plot_gazebo 
p1 = plot(tcbag,Fxbag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(F(1,:),F(2,:) ,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="-");
hold on
p3 = plot(out.Ff.Time,-out.Ff.Data(:,1),'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle="--");
hold on
legendVector = [legendVector,p2,p3];
hold off
% title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$F_{t,x}\ [N]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tfplot)
grid on

legendVector = [];
subplot(3,1,3)
if plot_gazebo 
p1 = plot(tcbag,Fybag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(F(1,:),F(3,:) ,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="-");
hold on
p3 = plot(out.Ff.Time,-out.Ff.Data(:,2),'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle="--");
hold on
legendVector = [legendVector,p2,p3];
hold off
% title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \ [s]$',FontSize=20,Interpreter="latex")
ylabel('$F_{t,y}\ [N]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tfplot)
grid on

%% compare u
% tuplot = [0 0.05];
tuplot = tplot;
figure('Name','Joint Torques')

legendVector = [];
subplot(3,1,1)
title_txt = ['$ \tau_1(t) \ | \ ',header1,suffix]; 
if plot_gazebo 
p1 = plot(tu1bag,u1bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(tusim,u1v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(u1simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
% xline(2.013,'k--',LineWidth=1.5)
% hold on
% xline(2.02,'k--',LineWidth=1.5)
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$\tau_1 \ [Nm]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tuplot)
grid on

%2
legendVector = [];
subplot(3,1,2)
title_txt = ['$ \tau_2(t) \ | \ ',header1,suffix]; 
if plot_gazebo 
p1 = plot(tu2bag,u2bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(tusim,u2v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(u2simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
% xline(2.013,'k--',LineWidth=1.5)
% hold on
% xline(2.02,'k--',LineWidth=1.5)
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$\tau_2 \ [Nm]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tuplot)
grid on

%3
legendVector = [];
subplot(3,1,3)
title_txt = ['$ \tau_3(t) \ | \ ',header1,suffix]; 
if plot_gazebo 
p1 = plot(tu3bag,u3bag,'b',DisplayName="$Bag \  Data$",LineWidth=2.5,LineStyle="-.");
hold on
legendVector = [legendVector,p1];
end

p2 = plot(tusim,u3v,'g',DisplayName="$Sim \  Data$",LineWidth=2,LineStyle="--");
hold on
p3 = plot(u3simulink,'r',DisplayName="$Simscape \  Data$",LineWidth=1.5,LineStyle=":");
legendVector = [legendVector,p2,p3];
% xline(2.013,'k--',LineWidth=1.5)
% hold on
% xline(2.02,'k--',LineWidth=1.5)
hold off
title(title_txt,FontSize=20,Interpreter="latex");
xlabel('$t \  [s]$',FontSize=20,Interpreter="latex")
ylabel('$\tau_3 \ [Nm]$',FontSize=20,Interpreter="latex")
legend(legendVector,Interpreter="latex",FontSize=20)
xlim(tuplot)
grid on

