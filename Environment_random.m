classdef Environment_random < handle
    %Environment Class to keep track of thermals and so on.
    %   Environment contains a number of thermals. On initialisation, a
    %   mass conservation correction is computed such that the average
    %   vertical velocity is zero. The environment provides (exact) measurements to
    %   sensors. The measurement is just the addition of the contributions
    %   from each thermal minus the mass conservation correction.
    
    properties
        Thermals;
        xlim=[];
        ylim=[];
        mass_cons_correction=0;
        x;
        y;
        z;
        field;
    end
    
    methods
        function obj=Environment_random(xlim,ylim,~,~,plot_axis,varargin)
            
            [xf,yf]=meshgrid([xlim(1):20:xlim(2)],[ylim(1):20:ylim(2)]);
            field = 5*randn(size(xf));
           
            [obj.x,obj.y]=meshgrid([xlim(1):1:xlim(2)],[ylim(1):1:ylim(2)]);
            obj.z=interp2(xf,yf,field,obj.x,obj.y,'spline'); 
            %
            obj.xlim=xlim;
            obj.ylim=ylim;
            
            obj.mass_cons_correction = -1*mean(mean(obj.z));
            obj.Display(plot_axis);
        end
        function [w,grad]=ExactMeasurement(env,x,y)
           w=0;
           grad=[0;0];
           w = interp2(env.x,env.y,env.z,x,y,'nearest');
        end
        function handles=Display(env,displayaxis)
            [C,handles]=contour(displayaxis,env.x,env.y,env.z);
            clabel(C,handles);
        end
        function print(env)
            fprintf('%10s %10s %10s %10s %10s\n','Thermal','Strength','Radius','X','Y');
            for i=1:length(env.Thermals)
                fprintf('%10d %10f %10f %10f %10f\n',i,env.Thermals{i}.strength,env.Thermals{i}.radius,env.Thermals{i}.thermalx,env.Thermals{i}.thermaly);
            end
            fprintf('Mass cons correction: %f\n', env.mass_cons_correction);
        end
        
    end
    
end

