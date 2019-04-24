classdef ExtendedKalmanFilter_thermal < handle
    %ExtendedKalmanFilter Filter to estimate non-linear system state from
    %noisy measurements.
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
    end
    
    methods
        function obj=ExtendedKalmanFilter_thermal(Pinit,xinit,Q,R)
            obj.P=Pinit;
            obj.x=xinit;

            obj.Q=Q;
            obj.R=R;
        end
        function update(ekf,z,Px,Py,Vxdt,Vydt,yaw,rollparam)
            if nargin <=4
                yaw=0;
                rollparam=1;
            end
            
            %Estimate new state from old. Also obtain Jacobian matrix for
            %later.
            [x1,A]=ekf.jacobian_f(ekf.x,Vxdt,Vydt); 
            
            %Update the covariance matrix 
            %P=A*ekf.P*A'+ekf.Q;                 %partial update
            %We know A is identity so
            P=ekf.P+ekf.Q;
            
            %What measurement do we expect to receive in the estimated
            %state
            [z_exp,H]=ekf.jacobian_h(x1,Px,Py,yaw,rollparam);
            if numel(z)==1
               H = H(1,:);
               ekf.z_exp = z_exp(1);
               R = ekf.R(1,1);
            else
               ekf.z_exp = z_exp;
               R = ekf.R;
            end
            
            %Calculate the KALMAN GAIN
            P12=P*H';                                   %cross covariance
            K=P12*inv(H*P12+R);                     %Kalman filter gain
            
            %Correct the state estimate using the measurement residual.
            ekf.x=x1+K*(z-ekf.z_exp)';
            
            ekf.residual=(z-ekf.z_exp);
            %fprintf('%f %f \n',mov(3),mov(4));
            
            %Correct the covariance too.
            ekf.P=P-K*P12';        % Displays counting problem - due to low Jacobian for thermal radius when thermal is large and ac is close             
            
            % Ensure P matrix is symmetric
            ekf.P=ekf.make_symmetric(ekf.P);                      
        end
        function reset(ekf,x,P)
            %Reset covariance and state.
            ekf.x=x;
            ekf.P=P;
        end
    end
    methods(Static)
        function [w,A]=jacobian_h(x,Px,Py,yaw,rollparam)
            %This function computes the jacobian using equations from
            %analytical derivation of Gaussian updraft distribution
            %This expression gets used lots
            expon = exp(-((x(3)-Px)^2+(x(4)-Py)^2)/x(2)^2);
            r = sqrt((x(3)-Px)^2+(x(4)-Py)^2);
            yaw_corr = -(yaw-deg2rad(90));
            sinAngle = (cos(yaw_corr)*(x(3)-Px) - sin(yaw_corr)*(x(4)-Py)) / r;
            cosroll = 1.0; %Assume roll angle zero
            
            %Expected measurement
            w(1) = x(1)*expon;
            w(2) = -2.0 * rollparam * r * w(1) / x(2)^2 * sinAngle; 
                        
            %Elements of the Jacobian
            A(1,1)=expon;
            A(1,2)=2*x(1)*((x(3)-Px)^2+(x(4)-Py)^2)/x(2)^3*expon;
            A(1,3)=-2*x(1)*(x(3)-Px)/x(2)^2*expon;
            A(1,4)=-2*x(1)*(x(4)-Py)/x(2)^2*expon;
            
            A(2,1) = w(2) / x(1);
            A(2,2) = 4.0 * rollparam * x(1) * expon * sinAngle * r * (x(2)^2-(x(3)-Px)^2-(x(4)-Py)^2) / x(2)^5.0 * cosroll;
            A(2,3) = 2.0 * rollparam * x(1) * expon / x(2)^2 * (-cos(yaw_corr) + 2.0 / x(2)^2 * (x(3)-Px) * sinAngle * r) * cosroll; %NOTE: Slightly different than in Pixhawk because x/y inverted and directions changed
            A(2,4) = 2.0 * rollparam * x(1) * expon / x(2)^2 * ( sin(yaw_corr) + 2.0 / x(2)^2 * (x(4)-Py) * sinAngle * r) * cosroll; %NOTE: Slightly different than in Pixhawk because x/y inverted and directions changed
        end
        function [xn,A]=jacobian_f(x,Vxdt,Vydt)
            %Computes new state and jacobian
            xn = [x(1);x(2);x(3)-Vxdt;x(4)-Vydt];
            %Jacobian is identity
            A=eye(4);
        end
        function B=make_symmetric(A)
            B=A;
            for i=2:size(A,1)
                for j=1:i-1
                    B(i,j)=(B(i,j)+B(j,i))/2;
                    B(j,i)=B(i,j);
                end
            end
        end
                    
    end
    
    
end


