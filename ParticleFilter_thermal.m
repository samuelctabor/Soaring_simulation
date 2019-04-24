classdef ParticleFilter_thermal < handle
    %ParticleFilter to estimate system state from noisy measurements in a 
    % fully non-linear way.
    %   NOTE: This implementation uses and thus requires the Particle Filter
    %   Implementation of Matlab 2016a's robotics toolbox!
    %
    %   Class maintains state estimation, covariance of measurements and
    %   state covariance estimation. Update method takes sensor readings
    %   and performs the necessary calcs. Note it also takes a function
    %   handle to the state update function and measurement estimation
    %   function.
    
    properties (SetAccess=protected)
        P=zeros(4,4);
        x=zeros(4,1);
        Q=zeros(4,4);
        R=zeros(2,2);
        z_exp=zeros(2,1);
        residual=0;
        K=0;
        PF;
        Execute=true;
        NumParticles=5000;
    end
    
    methods
        function obj=ParticleFilter_thermal(Pinit,xinit,Q,R,K)
            
            if(verLessThan('matlab','9.1'))
                DISP('ERROR: Your matlab version is too old to support the Matlab System-ID toolbox ParticleFilter class. The filter will not be executed!');
                obj.Execute=false;
            elseif(verLessThan('robotics','1.2'))
                DISP('ERROR: Your matlab robotics-toolbox version is too old to support the ParticleFilter class. The filter will not be executed!');
                obj.Execute=false;
            end
            
            obj.P=Pinit;
            obj.x=xinit;
            obj.Q=Q;
            obj.R=R;
            obj.K=K;
            
            obj.PF = robotics.ParticleFilter;
            obj.PF.StateTransitionFcn = @StateTransitionFcn;
            obj.PF.MeasurementLikelihoodFcn = @MeasurementLikelihoodFcn;
            initialize(obj.PF, obj.NumParticles, xinit, Pinit);
        end
        function update(obj,z,Px,Py,Vxdt,Vydt,yaw,rollparam)
            if(obj.Execute==false)
                return
            end
            
            if nargin <=4
                yaw=0;
                rollparam=1;
            end
            
            % Particle filtering steps
            [obj.x,obj.P] = predict(obj.PF,Vxdt,Vydt,obj.Q);
            [obj.x,obj.P] = correct(obj.PF,z,Px,Py,yaw,rollparam,obj.R);
            obj.x = getStateEstimate(obj.PF);
            
            % Additional particle filtering step: Roughening
            obj.PF.Particles = roughenParticles(obj.PF, obj.PF.Particles, obj.K);

            %Just calculate the following variables as debug output. 
            % Note: %The z_exp calculate from the estimated state x is NOT the average z_exp 
            % of all particles because the measurement function is nonlinear!
            obj.z_exp = MeasurementFcn(obj.x, Px, Py, yaw, rollparam);
            obj.residual = z - obj.z_exp;
        end
        function reset(obj,xinit,Pinit)
            %Reset particles to initial state and covariance.
            initialize(obj.PF, obj.NumParticles, xinit, Pinit);
        end
    end
    methods(Static)
    end
    
    
end

function new_particles = StateTransitionFcn(PF, particles, Vxdt, Vydt, Q)
    %Computes new state
    new_particles(:,1) = particles(:,1) + sqrt(Q(1,1)) * randn(size(particles,1),1);
    new_particles(:,2) = particles(:,2) + sqrt(Q(2,2)) * randn(size(particles,1),1);
    new_particles(:,3) = particles(:,3)-Vxdt + sqrt(Q(3,3)) * randn(size(particles,1),1);
    new_particles(:,4) = particles(:,4)-Vydt + sqrt(Q(4,4)) * randn(size(particles,1),1);
end

function z = MeasurementFcn(x, Px, Py, yaw, rollparam)
    expon = exp(-((x(3)-Px)^2+(x(4)-Py)^2)/x(2)^2);
    r = sqrt((x(3)-Px)^2+(x(4)-Py)^2);
    yaw_corr = -(yaw-deg2rad(90));
    sinAngle = (cos(yaw_corr)*(x(3)-Px) - sin(yaw_corr)*(x(4)-Py)) / r;
    cosroll = 1.0; %Assume roll angle zero

    %Expected measurement
    z(1) = x(1)*expon;% + noise (1);
    z(2) = -2.0 * rollparam * r * z(1) / x(2)^2 * sinAngle;% + noise(2);
end

function likelihood = MeasurementLikelihoodFcn(PF, particles, z, yaw, rollparam,R)
    %Calculate expected measurements
    z_exp = MeasurementFcn(particles,yaw,rollparam);
    
    %Calculate likelihood of each individual expected measurement given our actual
    %measurement P(z_e|z). Note: We assume the two measurements are
    %uncorrelated here, i.e. we will calculate P(z_e|z) =
    %P(z_e_1|z_1)*P(z_e_2|z_2).
    P_ze1_z1 = 1/sqrt(2*R(1,1)*pi())*exp(-(z_exp(:,1)-z(1)).^2/(2*R(1,1)));
    P_ze2_z2 = 1/sqrt(2*R(2,2)*pi())*exp(-(z_exp(:,2)-z(2)).^2/(2*R(2,2)));
    likelihood = P_ze1_z1.*P_ze2_z2;
    
    %Enforce state bounds
    likelihood(find(particles(:,1)<0)) = 0.0; % Enforce W>0
    likelihood(find(particles(:,2)<0)) = 0.0; % Enforce R>0
end

function roughened_particles = roughenParticles(PF, particles, K)
    % Simple method of particle roughening from "Novel approach to
    % nonlinear/non-Gaussian Bayesian state estimatoin", N.J. Gordon et
    % al., IEE Proceedings F - Radar and Signal Processing (1993).
    
    % Calculate individual standard deviations
    E(:) = max(particles,[],1) - min(particles,[],1);
    sigma_i = K * E * PF.NumParticles^(-1/PF.NumStateVariables);
    
    % Draw from this Gaussian (N[0,sigma_i]) distribution
    temp = randn(PF.NumParticles,PF.NumStateVariables) .* sigma_i;
    roughened_particles = particles + temp;
end