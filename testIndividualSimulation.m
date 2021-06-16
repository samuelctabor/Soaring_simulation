close all
clear


% SETUP SIMULATION HERE
displayPlots = true;
testcase = 3;                             % Testcase Definition. Note that testcase=100 will use random waypoints.

waypoints = [-0,100,0; 0,0,1];                                               % testcase=1: Pass through thermal core (bottom -> top)

x_real_glob = [2, 40, 0, 0];               % Define the "ground truth" or real thermal values here!

% Application init - You don't have to modify this section
set(0,'defaultTextInterpreter','tex');
scrsz = get(groot,'ScreenSize');

if displayPlots
    figure('Name','Sim','Position',[200 scrsz(4)/4 scrsz(3)/3 2*scrsz(4)/3]);
end

execution_frequency = 20;
nr_iterations = 100*execution_frequency;

variables.SaveReducedHistory= false;
variables.bSimulateSilently=  ~displayPlots;
variables.kf_x_init=          [1.5 80 0];
variables.measurement_noise=  0.4;
variables.measurement_noise_z2=100;
variables.actual_noise     = 0.4;
variables.actual_noise_z2     = 0.0;
variables.pf_K= 0.05;
variables.ukf_alpha=0.1;
variables.ceiling=1200;
variables.thermalling_radius= 40;
variables.min_cruise_time = 10;
variables.min_cruise_time = 10;
variables.min_thermal_latch_time = 20;
variables.KFtype = 1;
variables.acWPs = waypoints;


variables.kf_P_init = diag([100^2 100^2 140^2 140^2]);
variables.process_noise_q1 = 0.01;
variables.process_noise_q2 = 0.25;
variables.process_noise_q3 = 0.25; %opt. for R(2,2)=inf

variables.acInitState = [-0,-100,200,9.3,  deg2rad(90)];

% Set up simulation.
sim = Simulation(gca,displayPlots,0,0,3,120,[],variables);
sim.execution_frequency = execution_frequency;
sim.TheAircraft.updraftsensor.covariance = 0.16;
sim.TheAircraft.controller.ThermalTrackingActive = true;
                
%Innermost loop - actual simulation run.
for i = 1:nr_iterations
    bSilent=true;
    if(mod(i,1)==0)
        drawnow;
        bSilent = false;
    end
    sim.Update(1.0/sim.execution_frequency, bSilent);
end

data = sim.TheAircraft.History;
ax=zeros(0,0);

%3D positions and thermal estimates?
figure('Name','3D Pos');
plot3(data.p(:,1),data.p(:,2),data.p(:,3));
hold all
plot3(data.kf.x_xy_glob(:,1), data.kf.x_xy_glob(:,2), zeros(size(data.kf.x_xy_glob(:,2))));
legend('Airplane pos','Estimated Thermal Pos');
plot3(x_real_glob(3),x_real_glob(4),0,'o','MarkerSize',20);
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;

%States and measurements
figure('Name','States');
ax(1) = subplot(4,1,1);
yyaxis left ; plot(data.t,[data.kf.x(:,1) x_real_glob(1)*ones(size(data.kf.x(:,1)))]);
yyaxis right ; plot(data.t,[data.kf.x(:,2) x_real_glob(2)*ones(size(data.kf.x(:,1)))]);
legend('W_{est}','W_{real}','R_{est}','R_{real}');
ax(end+1) = subplot(4,1,2);
plot(data.t,data.kf.x(:,3));
hold on; plot(data.t,data.kf.x(:,4));
legend('x_{est,local}','y_{est,local}');
ax(end+1) = subplot(4,1,3);
plot(data.t,[data.kf.x_xy_glob(:,1) x_real_glob(3)*ones(size(data.kf.x_xy_glob(:,1)))]);
hold on; plot(data.t,[data.kf.x_xy_glob(:,2) x_real_glob(4)*ones(size(data.kf.x_xy_glob(:,2)))]);
legend('x_{est,glob}','x_{real,glob}','y_{est,glob}','y_{real,glob}');
ax(end+1) = subplot(4,1,4);
yyaxis left ; plot(data.t',[data.kf.z_exp(:,1) data.z(:,1)]);
yyaxis right ; plot(data.t',[data.kf.z_exp(:,2) data.z(:,2)]);
legend('z1_{exp}','z1','z2_{exp}','z2');

%Residuals: 1) Measurements and 2) Estimation (& covariances?)
figure('Name','ResVar');
ax(end+1) = subplot(3,1,1);
plot(data.t',[data.z(:,1)-data.kf.z_exp(:,1) data.z(:,2)-data.kf.z_exp(:,2)]);
title(['WPCase:' num2str(testcase)]);
legend('res_{z1}','res_{z2}');
ax(end+1) = subplot(3,1,2);
plot(data.t',[x_real_glob(1)-data.kf.x(:,1) x_real_glob(2)-data.kf.x(:,2) x_real_glob(3)-data.kf.x_xy_glob(:,1) x_real_glob(4)-data.kf.x_xy_glob(:,2)]);

legend('res_{W}','res_{R}','res_{x}','res_{y}');
ax(end+1) = subplot(3,1,3);
semilogy(data.t',[data.kf.P(:,1) data.kf.P(:,2) data.kf.P(:,3) data.kf.P(:,4)]);

legend('P_{W}','P_{R}','P_{x}','P_{y}');
linkaxes(ax,'x');
