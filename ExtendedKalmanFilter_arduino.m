classdef ExtendedKalmanFilter_arduino < handle
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
        s1=serial('/dev/tty.usbserial-A400fQ5x');
        %s1=serial('/dev/tty.usbserial-A1024PXR');
    end
    
    methods
        function obj=ExtendedKalmanFilter_arduino(Pinit,xinit,Q,R)
            obj.P=Pinit;
            obj.x=xinit;

            obj.Q=Q;
            obj.R=R;
            
            obj.s1.BaudRate=115200; %9600;
            fopen(obj.s1);   
        end
        function delete(obj)
            fclose(obj.s1);
        end
        function update(ekf,z,Vxdt,Vydt)
            %Estimate new state from old. Also obtain Jacobian matrix for
            %later.
            if (max([norm(z),norm(Vxdt),norm(Vydt)])>(2^15)/1000)
                fprintf('Warning: overflow likely!\n');
            end
            fprintf(ekf.s1,'U ');
            %fprintf(1,'U ');
            fprintf(ekf.s1,sprintf('%i %i %i\n',int16(z*1000),int16(Vxdt*1000),int16(Vydt*1000)));
            %fprintf(1,sprintf('%i %i %i\n',int16(z*1000),int16(Vxdt*1000),int16(Vydt*1000)));
            %fprintf('Sent\n');
            %arr=fscanf(ekf.s1,'%f\n')
            try
                ekf.x(1)=fscanf(ekf.s1,'%f\n');
                ekf.x(2)=fscanf(ekf.s1,'%f\n');
                ekf.x(3)=fscanf(ekf.s1,'%f\n');
                ekf.x(4)=fscanf(ekf.s1,'%f\n');
                %fscanf(ekf.s1,'Done.\n');
                %fprintf('Received\n');
                %ekf.x
            catch
                fprintf('Warning: read failed!\n')
            end
        end
        function reset(ekf,x,P)
            %Reset covariance and state.
            %ekf.x=x;
            %ekf.P=P;
            fprintf(ekf.s1,'R ');
            fprintf(ekf.s1,sprintf('%i %i %i %i\n',int16(x(1)*1000),int16(x(2)*1000),int16(x(3)*1000),int16(x(4)*1000)));
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
    end
    
    
end


