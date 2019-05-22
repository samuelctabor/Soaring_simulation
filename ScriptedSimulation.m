close all
clear

% SETUP SIMULATION HERE
useParallelSimulation = false;               % Note: you have to manually enable either the for- or the parfor-loop below!
displayPlots = false && ~useParallelSimulation;
testcase = 3;                             % Testcase Definition. Note that testcase=100 will use random waypoints.
Waypoints{1} = [-0,100,0; 0,0,1];                                               % testcase=1: Pass through thermal core (bottom -> top)
Waypoints{2} = [-40,100,0; 0,0,1];                                              % testcase=2: Tangential pass of thermal (bottom -> top)
Waypoints{3} = [-40,100,0; 0,0,1];                                              % testcase=3: Tangential pass of thermal (bottom -> top), but initialized on wrong side
Waypoints{4} = [-100,100,0; -100,-100,0];                                       % testcase=4: Traverse
Waypoints{5} = [-50,100,0; 50,-100,0 ; 150,100,0;-50,100,0; 50,-100,0 ;];       % testcase=5: Zick-Zack
Waypoints{6} = [-100,100,0; -0,-50,1];                                          % testcase=6: Straight line up then open loop loiter near center
rndsamples = 50;                            % Amount of random samples to use        
x_real_glob = [2, 40, 0, 0];               % Define the "ground truth" or real thermal values here!

var1 = [1, 2];%0.000:0.005:0.015;                %Define the variables (var1,2,3) to loop/optimze over here (scalar->no loop)
var2 = 1;%0.1:0.1:0.4;
var3 = 1;%0.1:0.1:0.4;

% Application init - You don't have to modify this section
set(0,'defaultTextInterpreter','tex');
scrsz = get(groot,'ScreenSize');

if displayPlots
    figure('Name','Sim','Position',[200 scrsz(4)/4 scrsz(3)/3 2*scrsz(4)/3]);
end
execution_frequency = 20;
nr_iterations = 100*execution_frequency;
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
            
            for s=1:rndsamples % Inner loop: Random analysis. NOTE: USE THIS FOR NON-PARALELL SIMULATION
%             parfor s=1:rndsamples % Inner loop: Random analysis. NOTE: USE THIS FOR PARALELL SIMULATION
           
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
                variables.KFtype = var1(j);
                variables.acWPs = Waypoints{testcase};
                
                % Configure the filters - load optimized Pinit and process
                % noise values (Note: These are only optimal for a  specific xinit, 
                % process and measurement noise and and overall problem setup!)
                switch variables.KFtype
                    case 1 %EKF
                        variables.kf_P_init = diag([100^2 100^2 140^2 140^2]);
                        variables.process_noise_q1 = 0.01;
                        variables.process_noise_q2 = 0.25;
                        variables.process_noise_q3 = 0.25; %opt. for R(2,2)=inf
                    case 2 %UKF
                        variables.kf_P_init = diag([0.5^2 20^2 30^2 30^2]);
                        variables.process_noise_q1 = 0.01;
                        variables.process_noise_q2 = 0.25;
                        variables.process_noise_q3 = 0.4;
                    case 3 %PF
                        variables.kf_P_init = diag([2^2 80^2 140^2 140^2]);
                        variables.process_noise_q1 = 0.008;
                        variables.process_noise_q2 = 0.15;
                        variables.process_noise_q3 = 0.25;
                end

                if(testcase == 1)
                    variables.acInitState = [-0,-100,200,9.3,  deg2rad(90)];
                elseif(testcase == 2)
                    variables.acInitState = [-40,-100,200,9.3, deg2rad(90)];
                elseif(testcase == 3)
                    variables.acInitState = [-40,-100,200,9.3, deg2rad(90)];
                elseif(testcase < 100)
                    variables.acInitState = [-100,-100,200,9.3,0];
                else
                    rnd_f = (rand(3,1)-0.5)*2.0;%rnd_f_s;%
                    rnd_i = randi(numel(Waypoints)); %rnd_i_s;%
                    variables.acInitState = [rnd_f(1)*sim(s).environment.xlim(2),rnd_f(2)*sim(s).environment.ylim(2),200,9.3,rnd_f(3)*pi()];
                end
                
                % Set up simulation.
                sim(s) = Simulation(gca,displayPlots,0,0,3,120,[],variables);
                sim(s).execution_frequency = execution_frequency;
                sim(s).TheAircraft.updraftsensor.covariance = 0.16;
                sim(s).TheAircraft.controller.ThermalTrackingActive = true;
                
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
                residuals(s,:,:) = [sim(s).TheAircraft.History.kf.x(:,1)        - x_real_glob(1),...
                                    sim(s).TheAircraft.History.kf.x(:,2)        - x_real_glob(2),...
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
                    figure('Name','3D Pos');
                    plot3(data.p(:,1),data.p(:,2),data.p(:,3));
                    hold all
                    plot3(data.kf.x_xy_glob(:,1), data.kf.x_xy_glob(:,2), zeros(size(data.kf.x_xy_glob(:,2))));
                    legend('Airplane pos','Estimated Thermal Pos');
                    plot3(x_real_glob(3),x_real_glob(4),0,'o','MarkerSize',20);
                    %contour(handles.simulation.environment.x,handles.simulation.environment.y,handles.simulation.environment.z);
                    xlabel('x');
                    ylabel('y');
                    zlabel('z');
                    axis equal;

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
                    title(['WPCase:' num2str(testcase)]);
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
            
            % Calculate centring times.% Converged when the residual falls below 50% radius for the last time.
             positionEstimateResidual = squeeze(sqrt(residuals(:,:,3).^2 + residuals(:,:,4).^2))';
            [~, convergeIdx] = max(flipud(positionEstimateResidual)>x_real_glob(2)/2, [], 1);
            convergeIdx = size(positionEstimateResidual,1) - convergeIdx + 1;
            convergeTime = sim(1).TheAircraft.History.t(convergeIdx);

            %Store in result arrays
            results{l,k,j}.performance.avrgRes = avrgRes / rndsamples;%TODO check
            results{l,k,j}.History = sim(end).TheAircraft.History;
            results{l,k,j}.settings.KFtype = sim(end).TheAircraft.controller.KFtype;
            results{l,k,j}.settings.q = [sim(end).TheAircraft.controller.variables.process_noise_q1, sim(end).TheAircraft.controller.variables.process_noise_q2, sim(end).TheAircraft.controller.variables.process_noise_q3];
            results{l,k,j}.centringTimes = convergeTime;
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
time = (1:nr_iterations)/execution_frequency;
plot(time(idx)',GlobalResiduals(idx,1)); ylim([0 x_real_glob(1)]);
yyaxis right; plot(time(idx)',[GlobalResiduals(idx,2) GlobalResiduals(idx,3) GlobalResiduals(idx,4)]); ylim([0 max(x_real_glob(2),x_real_glob(3))]);
%yyaxis right ; plot(data.t',data.z(:,2)-data.kf.z_exp(:,2));
xlabel('Time [s]');
legend('res_{W}','res_{R}','res_{x}','res_{y}');

%% Plot & compare: Residuals of all simulation runs combined in 3D Plot
if(0 && ndims(performance_val)==3)
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

if (1) % Histogram of centring times.
    figure,hist([results{1}.centringTimes; results{2}.centringTimes]',10)
    legend('EKF','UKF');
end
