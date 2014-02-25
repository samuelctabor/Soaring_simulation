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
        R=zeros(1,1);
        residual=0;
    end
    
    methods
        function obj=ExtendedKalmanFilter_thermal(Pinit,xinit,Q,R)
            obj.P=Pinit;
            obj.x=xinit;

            obj.Q=Q;
            obj.R=R;
        end
        function update(ekf,z,Vxdt,Vydt)
            %Estimate new state from old. Also obtain Jacobian matrix for
            %later.
            [x1,A]=ekf.jacobian_f(ekf.x,Vxdt,Vydt); 
            
            %Update the covariance matrix 
            %P=A*ekf.P*A'+ekf.Q;                 %partial update
            %We know A is identity so
            P=ekf.P+ekf.Q;
            
            %What measurement do we expect to receive in the estimated
            %state
            [z1,H]=ekf.jacobian_h(x1);    
            
            %Calculate the KALMAN GAIN
            P12=P*H';                                   %cross covariance
            K=P12*inv(H*P12+ekf.R);                     %Kalman filter gain
            
            %Correct the state estimate using the measurement residual.
            ekf.x=x1+K*(z-z1);
            
            ekf.residual=(z-z1);
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
        function [w,A]=jacobian_h(x)
            %This function computes the jacobian using equations from
            %analytical derivation of Gaussian updraft distribution
            %This expression gets used lots
            expon = exp(-(x(3)^2+x(4)^2)/x(2)^2);
            %Expected measurement
            w = x(1)*expon;
            
            %Elements of the Jacobian
            A(1)=expon;
            A(2)=2*x(1)*(x(3)^2+x(4)^2)/x(2)^3*expon;
            A(3)=-2*x(1)*x(3)/x(2)^2*expon;
            A(4)=-2*x(1)*x(4)/x(2)^2*expon;
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


