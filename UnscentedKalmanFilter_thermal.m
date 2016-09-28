classdef UnscentedKalmanFilter_thermal < handle
    %UnscentedKalmanFilter to estimate non-linear system state from
    %noisy measurements.
    %   NOTE: This implementation uses and thus requires the Unscented
    %   Kalman Implementation of Matlab 2016b's System Identification
    %   toolbox!
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
        UKF;
        sigma_points;
        Execute=true;
    end
    
    methods
        function obj=UnscentedKalmanFilter_thermal(Pinit,xinit,Q,R)
            
            if(verLessThan('matlab','9.1'))
                DISP('ERROR: Your matlab version is too old to support the Matlab System-ID toolbox unscentendKalmanFilter command. The filter will not be executed!');
                obj.Execute=false;
            elseif(verLessThan('ident','9.5'))
                DISP('ERROR: Your matlab System-ID toolbox version is too old to support the unscentendKalmanFilter command. The filter will not be executed!');
                obj.Execute=false;
            end
            
            obj.P=Pinit;%?
            obj.x=xinit;%?

            obj.Q=Q;%?
            obj.R=R;%?
            
            obj.UKF = unscentedKalmanFilter(@StateTransitionFcn,@MeasurementFcn,xinit,'HasAdditiveProcessNoise',true,'HasAdditiveMeasurementNoise',true);
            %obj.UKF.State = xinit;
            obj.UKF.StateCovariance = Pinit;
            obj.UKF.MeasurementNoise = R;
            obj.UKF.ProcessNoise = Q;
            obj.UKF.Alpha = 0.05;
            obj.UKF.Kappa = 0.0;
        end
        function update(obj,z,Vxdt,Vydt,yaw,rollparam)
            if(obj.Execute==false)
                return
            end
            
            if nargin <=4
                yaw=0;
                rollparam=1;
            end
            
            %Prediction step
            [obj.x, obj.P] = predict(obj.UKF,Vxdt,Vydt); %TODO ? Noise already added or have to pass as arugment?
            
            
            % Calculate unscented transformation parameters
            [c, Wmean, Wcov, OOM] = matlabshared.tracking.internal.calcUTParameters(obj.UKF.Alpha,obj.UKF.Beta,obj.UKF.Kappa,numel(obj.x));
            [X1,obj.sigma_points] = matlabshared.tracking.internal.calcSigmaPoints(obj.P, obj.x, c);
            
            %Correction Step
            obj.z_exp = MeasurementFcn(obj.x, yaw, rollparam);
            obj.residual = z - obj.z_exp;
            [obj.x, obj.P] = correct(obj.UKF,z,yaw,rollparam); 
            
            
            %test = UKFCorrectorAdditive
        end
        function reset(obj,x,P)
            %Reset covariance and state.
            obj.x=x;
            obj.P=P;
            obj.UKF.State = x;
            obj.UKF.StateCovariance = P;
        end
    end
    methods(Static)
    end
    
    
end

function x = StateTransitionFcn(x,Vxdt,Vydt)
    %Computes new state
    x = [x(1);...%+noise(1);...
         x(2);...%+noise(2);...
         x(3)-Vxdt;...%+noise(3);...
         x(4)-Vydt];%+noise(4)];
end
        
function z = MeasurementFcn(x, yaw, rollparam)             
    expon = exp(-(x(3)^2+x(4)^2)/x(2)^2);
    r = sqrt(x(3)^2+x(4)^2);
    yaw_corr = -(yaw-deg2rad(90));
    sinAngle = (cos(yaw_corr)*x(3) - sin(yaw_corr)*x(4)) / r;
    cosroll = 1.0; %Assume roll angle zero

    %Expected measurement
    z(1) = x(1)*expon;% + noise (1);
    z(2) = -2.0 * rollparam * r * z(1) / x(2)^2 * sinAngle;% + noise(2); 
end