close all
clear all

% SETUP SIMULATION HERE
useParallelSimulation = true;               % Note: you have to manually enable either the for- or the parfor-loop below!
testcase = 100;                             % Testcase Definition. Note that testcase=100 will use random waypoints.
Waypoints{1} = [-0,100,0; 0,0,1];                                               % testcase=1: Pass through thermal core (bottom -> top)
Waypoints{2} = [-40,100,0; 0,0,1];                                              % testcase=2: Tangential pass of thermal (bottom -> top)
Waypoints{3} = [-40,100,0; 0,0,1];                                              % testcase=3: Tangential pass of thermal (bottom -> top), but initialized on wrong side
Waypoints{4} = [-100,100,0; -100,-100,0];                                       % testcase=4: Traverse
Waypoints{5} = [-50,100,0; 50,-100,0 ; 150,100,0;-50,100,0; 50,-100,0 ;];       % testcase=5: Zick-Zack
Waypoints{6} = [-100,100,0; -0,-50,1];                                          % testcase=6: Straight line up then open loop loiter near center
rndsamples = 30;                            % Amount of random samples to use        
x_real_glob = [3, 120, 0, 0];               % Define the "ground truth" or real thermal values here!

var1 = 1;%0.000:0.005:0.015;                %Define the variables (var1,2,3) to loop/optimze over here (scalar->no loop)
var2 = 1;%0.1:0.1:0.4;
var3 = 1;%0.1:0.1:0.4;

% Application init - You don't have to modify this section
set(0,'defaultTextInterpreter','tex');
scrsz = get(groot,'ScreenSize');
if(~useParallelSimulation) figure('Name','Sim','Position',[200 scrsz(4)/4 scrsz(3)/3 2*scrsz(4)/3]); end;
simOrg = Simulation(gca,~useParallelSimulation,0,0,3,120);
bSilent=true;
nr_iterations = 100*simOrg.execution_frequency;
results = cell(numel(var3), numel(var2), numel(var1));
performance_val = zeros(numel(var3), numel(var2), numel(var1));
if(useParallelSimulation)
    gcp;
end

for l=1:numel(var3)   
    for k=1:numel(var2)
        for j=1:numel(var1)
            results{l,k,j}.performance.avrgRes = 0;
            avrgRes = 0;
            GlobalResiduals = zeros(nr_iterations,4);
            GlobalResiduals_count = zeros(nr_iterations,1);
            residuals = zeros(rndsamples,nr_iterations,4);
            residuals_accum = zeros(rndsamples,4);
            
            %for s=1:rndsamples % Inner loop: Random analysis. NOTE: USE THIS FOR NON-PARALELL SIMULATION
            parfor s=1:rndsamples % Inner loop: Random analysis. NOTE: USE THIS FOR PARALELL SIMULATION
           
                %Configure the simulation
                sim(s) = simOrg;
                
                if(testcase == 1)
                    sim(s).TheAircraft.reset(-0,-100,200,9.3,  deg2rad(90),sim(s).execution_frequency, Waypoints{testcase}, nr_iterations);
                elseif(testcase == 2)
                    sim(s).TheAircraft.reset(-40,-100,200,9.3, deg2rad(90),sim(s).execution_frequency, Waypoints{testcase}, nr_iterations);
                elseif(testcase == 3)
                    sim(s).TheAircraft.reset(-40,-100,200,9.3, deg2rad(90),sim(s).execution_frequency, Waypoints{testcase}, nr_iterations);
                elseif(testcase < 100)
                    sim(s).TheAircraft.reset(-100,-100,200,9.3,0,          sim(s).execution_frequency, Waypoints{testcase}, nr_iterations);
                else
                    rnd_f = (rand(3,1)-0.5)*2.0;%rnd_f_s;%
                    rnd_i = randi(numel(Waypoints)); %rnd_i_s;%
                    sim(s).TheAircraft.reset(rnd_f(1)*sim(s).environment.xlim(2),rnd_f(2)*sim(s).environment.ylim(2),200,9.3,rnd_f(3)*pi(),sim(s).execution_frequency, Waypoints{rnd_i},nr_iterations);
                end

                % Configure the filters - load optimized Pinit and process
                % noise values (Note: These are only optimal for a  specific xinit, 
                % process and measurement noise and and overall problem setup!)
                sim(s).TheAircraft.controller.KFtype = 1;
                if(sim(s).TheAircraft.controller.KFtype == 1) %EKF
                    sim(s).TheAircraft.controller.update_variable('kf_P_init',diag([100^2 100^2 140^2 140^2]));
                    q1 = 0.01; q2 = 0.25; q3 = 0.25; %opt. for R(2,2)=inf
                elseif(sim(s).TheAircraft.controller.KFtype == 2) %UKF
                    sim(s).TheAircraft.controller.update_variable('kf_P_init',diag([0.5^2 20^2 30^2 30^2]));
                    q1 = 0.01; q2 = 0.25; q3 = 0.4;
                elseif(sim(s).TheAircraft.controller.KFtype == 3) %PF
                    sim(s).TheAircraft.controller.update_variable('kf_P_init',diag([2^2 80^2 140^2 140^2]));
                    q1 = 0.008; q2 = 0.15; q3 = 0.25;
                end
                
                sim(s).TheAircraft.controller.ThermalTrackingActive = false;
                sim(s).TheAircraft.controller.update_variable('SaveReducedHistory',false);
                sim(s).TheAircraft.controller.update_variable('bSimulateSilently',true);
                sim(s).TheAircraft.controller.update_variable('kf_x_init',[1.5 80 30]);
                sim(s).TheAircraft.controller.update_variable('measurement_noise',00000.4);
                sim(s).TheAircraft.controller.update_variable('measurement_noise_z2',000000.5);
                %sim(s).TheAircraft.controller.update_variable('process_noise_q1',var1(j));
                %sim(s).TheAircraft.controller.update_variable('process_noise_q2',var2(k));
                %sim(s).TheAircraft.controller.update_variable('process_noise_q3',var3(l));
                sim(s).TheAircraft.controller.update_variable('process_noise_q1',q1);%0.01);
                sim(s).TheAircraft.controller.update_variable('process_noise_q2',q2);%0.25);
                sim(s).TheAircraft.controller.update_variable('process_noise_q3',q3);%0.4);
                sim(s).TheAircraft.controller.update_variable('pf_K',0.05);
                sim(s).TheAircraft.controller.update_variable('ukf_alpha',0.1);
                sim(s).TheAircraft.controller.update_variable('ceiling',1200);
                sim(s).TheAircraft.controller.update_variable('thermalling_radius',40);
                sim(s).TheAircraft.controller.update_variable('P_init',diag([2^2,80^2,100^2,100^2]));
                sim(s).TheAircraft.controller.update_variable('measurement_noise_z2',0.5);

                sim(s).TheAircraft.controller.SetupKalmanFilter(sim(s).execution_frequency);
                sim(s).currenttime=0;

                %Innermost loop - actual simulation run.
                for i = 1:nr_iterations
                    bSilent=true;
                    if(mod(i,1)==0)
                        drawnow;
                        bSilent = false;
                    end
                    sim(s).Update(1.0/sim(s).execution_frequency, bSilent);
                end

                %Calculate performance metric(s)
                residuals(s,:,:) = [sim(s).TheAircraft.History.kf.x(:,1)- x_real_glob(1),...
                    sim(s).TheAircraft.History.kf.x(:,2)- x_real_glob(2),...
                    sim(s).TheAircraft.History.kf.x_xy_glob(:,1)- x_real_glob(3),...
                    sim(s).TheAircraft.History.kf.x_xy_glob(:,2)- x_real_glob(4)];
                residuals_accum(s,:) = squeeze(sum(abs(residuals(s,:,:)),2)) ./ [x_real_glob(1), x_real_glob(2), x_real_glob(2), x_real_glob(2)]';
                avrgRes = avrgRes+sum(residuals_accum(s,:));
                
                idx_thermaldetected = find(abs(sim(s).TheAircraft.History.kf.x(:,3))>0,1,'first');
                temp = squeeze(abs(residuals(s,:,:))); %Just needed because of parfor
                GlobalResiduals = GlobalResiduals + [temp(idx_thermaldetected:nr_iterations,:) ; zeros(idx_thermaldetected-1,4)];
                GlobalResiduals_count = GlobalResiduals_count + [ones(nr_iterations-idx_thermaldetected+1,1) ; zeros(idx_thermaldetected-1,1)];
                
                str = sprintf('#%d/%d(l=%d,k=%d,j=%d)-%d | KF:%d ATT:%d | q:%.3f/%.3f/%.3f r:%.3f/%.3f | Res-i.:%.3f',(l-1)*numel(var2)*numel(var1)+(k-1)*numel(var1)+j,numel(var1)*numel(var2)*numel(var3),l,k,j,s,sim(s).TheAircraft.controller.KFtype,sim(s).TheAircraft.controller.ThermalTrackingActive,...
                    sim(s).TheAircraft.controller.variables.process_noise_q1,sim(s).TheAircraft.controller.variables.process_noise_q2,sim(s).TheAircraft.controller.variables.process_noise_q3,...
                    sim(s).TheAircraft.controller.variables.measurement_noise,sim(s).TheAircraft.controller.variables.measurement_noise_z2,...
                    sum(residuals_accum(s,:)));
                fprintf('%s\n',str);
                
                %% Plot & compare: Single simulation run
                plotsingleresult=false;
                if(plotsingleresult && ~useParallelSimulation)
                    data = sim(s).TheAircraft.History;
                    ax=zeros(0,0);
                    h_margin = 0.1;
                    v_margin = 0.05;

                    %3D positions and thermal estimates?
%                     figure('Name','3D Pos');
%                     plot3(data.p(:,1),data.p(:,2),data.p(:,3));
%                     hold all
%                     plot3(data.kf.x_xy_glob(:,1), data.kf.x_xy_glob(:,2), zeros(size(data.kf.x_xy_glob(:,2))));
%                     legend('Airplane pos','Estimated Thermal Pos');
%                     plot3(x_real_glob(3),x_real_glob(4),0,'o','MarkerSize',20);
%                     %contour(handles.simulation.environment.x,handles.simulation.environment.y,handles.simulation.environment.z);
%                     xlabel('x');
%                     ylabel('y');
%                     zlabel('z');

                    %States and measurements
                    figure('Name','States');
                    ax(1) = subplot_tight(4,1,1,[v_margin h_margin]);
                    yyaxis left ; plot(data.t,[data.kf.x(:,1) x_real_glob(1)*ones(size(data.kf.x(:,1)))]);
                    yyaxis right ; plot(data.t,[data.kf.x(:,2) x_real_glob(2)*ones(size(data.kf.x(:,1)))]);
                    legend('W_{est}','W_{real}','R_{est}','R_{real}');
                    ax(end+1) = subplot_tight(4,1,2,[v_margin h_margin]);
                    plot(data.t,data.kf.x(:,3));
                    hold on; plot(data.t,data.kf.x(:,4));
                    legend('x_{est,local}','y_{est,local}');
                    ax(end+1) = subplot_tight(4,1,3,[v_margin h_margin]);
                    plot(data.t,[data.kf.x_xy_glob(:,1) x_real_glob(3)*ones(size(data.kf.x_xy_glob(:,1)))]);
                    hold on; plot(data.t,[data.kf.x_xy_glob(:,2) x_real_glob(4)*ones(size(data.kf.x_xy_glob(:,2)))]);
                    legend('x_{est,glob}','x_{real,glob}','y_{est,glob}','y_{real,glob}');
                    ax(end+1) = subplot_tight(4,1,4,[v_margin h_margin]);
                    yyaxis left ; plot(data.t',[data.kf.z_exp(:,1) data.z(:,1)]);
                    yyaxis right ; plot(data.t',[data.kf.z_exp(:,2) data.z(:,2)]);
                    legend('z1_{exp}','z1','z2_{exp}','z2');

                    %Residuals: 1) Measurements and 2) Estimation (& covariances?)
                    figure('Name','ResVar');
                    ax(end+1) = subplot_tight(3,1,1,[v_margin h_margin]);
                    plot(data.t',[data.z(:,1)-data.kf.z_exp(:,1) data.z(:,2)-data.kf.z_exp(:,2)]);
                    title(['WPCase:' num2str(rnd_i) 'posx,y,pathangle:' num2str(rnd_f(1)) num2str(rnd_f(2)) num2str(rnd_f(3))]);
                    legend('res_{z1}','res_{z2}');
                    ax(end+1) = subplot_tight(3,1,2,[v_margin h_margin]);
                    plot(data.t',[x_real_glob(1)-data.kf.x(:,1) x_real_glob(2)-data.kf.x(:,2) x_real_glob(3)-data.kf.x_xy_glob(:,1) x_real_glob(4)-data.kf.x_xy_glob(:,2)]);
                    %yyaxis right ; plot(data.t',data.z(:,2)-data.kf.z_exp(:,2));
                    legend('res_{W}','res_{R}','res_{x}','res_{y}');
                    ax(end+1) = subplot_tight(3,1,3,[v_margin h_margin]);
                    semilogy(data.t',[data.kf.P(:,1) data.kf.P(:,2) data.kf.P(:,3) data.kf.P(:,4)]);
                    %yyaxis right ; plot(data.t',data.z(:,2)-data.kf.z_exp(:,2));
                    legend('P_{W}','P_{R}','P_{x}','P_{y}');
                    
                    linkaxes(ax,'x');
                end
            end %loop over (s)
            
            %Store in result arrays
            results{l,k,j}.performance.avrgRes = avrgRes / rndsamples;%TODO check
            results{l,k,j}.History = sim(end).TheAircraft.History;
            results{l,k,j}.settings.KFtype = sim(end).TheAircraft.controller.KFtype;
            results{l,k,j}.settings.q = [sim(end).TheAircraft.controller.variables.process_noise_q1, sim(end).TheAircraft.controller.variables.process_noise_q2, sim(end).TheAircraft.controller.variables.process_noise_q3];
            performance_val(l,k,j) = results{l,k,j}.performance.avrgRes;
            fprintf('***** Average (over all %u random samples) accumulated residual: %f *****\n',rndsamples, results{l,k,j}.performance.avrgRes);
        end %loop over (j)
    end %loop over (k)
end %loop over (l)

% %% Plot: Global (over all samples) Residuuals vs. Time
figure('Name','Average residuuals');
%subplot_tight(3,1,2,[v_margin h_margin]);
idx = find(GlobalResiduals_count == rndsamples);
GlobalResiduals = GlobalResiduals / rndsamples;
yyaxis left; 
time = (1:nr_iterations)/simOrg.execution_frequency;
plot(time(idx)',GlobalResiduals(idx,1)); ylim([0 x_real_glob(1)]);
yyaxis right; plot(time(idx)',[GlobalResiduals(idx,2) GlobalResiduals(idx,3) GlobalResiduals(idx,4)]); ylim([0 max(x_real_glob(2),x_real_glob(3))]);
%yyaxis right ; plot(data.t',data.z(:,2)-data.kf.z_exp(:,2));
xlabel('Time [s]');
legend('res_{W}','res_{R}','res_{x}','res_{y}');

%% Plot & compare: Residuals of all simulation runs combined in 3D Plot
if(1 && ndims(performance_val)==3)
    [X,Y,Z] = meshgrid(var3,var2,var1);
    figure('Name','PerformanceComparison');
%     for l = 1:numel(var3)
%         for k = 1:numel(var2)
%             for j = 1:numel(var1)
%                 %param(j) = results{j}.settings.q(1);
%                  = results{l,k,j}.performance.avrgRes;
%             end
%         end
%     end
    
    xslice = 0.2; 
    yslice = [0.1 0.3]; 
    zslice = 0.01;
    test = permute(performance_val, [2 1 3]);
    slice(X,Y,Z,test,xslice,yslice,zslice);
    colormap('jet');
    %plot(param, performance_val);
    %slice(x, y, z, temp, xslice, yslice, zslice) 
    xlabel('var3 - x,y');
    ylabel('var2 - R');
    zlabel('var1 - W');
    legend('Accum. residuuals');
    colorbar
    
    %figure('Name','PerformanceComparison2');
    %slice(x, y, z, temp, xslice, yslice, zslice)
end

