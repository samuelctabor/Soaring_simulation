classdef Environment < handle
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
        use_mass_cons_correction = false;
        mass_cons_correction=0;
        x;
        y;
        z;
    end
    
    methods
        function obj=Environment(xlim,ylim,number_thermals,thermal_type,plot_axis,varargin)
            thermal_types = {@GaussianThermal,@FlightGearThermal};
            %thermal_type = thermal_types{ceil(rand(1)*2)};
            
            
            obj.xlim=xlim;
            obj.ylim=ylim;
            if nargin>7
                xloc = varargin{1};
                yloc = varargin{2};
                W    = varargin{3};
                R    = varargin{4};
            else
                xloc=xlim(1) + (xlim(2)-xlim(1))*rand(1,number_thermals);
                yloc=xlim(1) + (ylim(2)-ylim(1))*rand(1,number_thermals);
                W = 3*ones(size(xloc));
                R = 20*ones(size(xloc));
            end
            
            for i=1:number_thermals    
                h=thermal_type(xloc(i),yloc(i),W(i),R(i));
                thermals{i}=h;
            end
            obj.Thermals=thermals;
            
            obj.Grid(3);
            obj.mass_cons_correction = -1*mean(mean(obj.z));
            obj.Display(plot_axis);
        end
        function [w,grad]=ExactMeasurement(env,x,y,yaw)
           w=0;
           grad=0;
           for i=1:length(env.Thermals)
               %x;
               %y;
               [wi,gradi]=env.Thermals{i}.ExactMeasurement(x,y,yaw);
               w=w+wi;
               grad=grad+gradi;
           end
           if(env.use_mass_cons_correction) w=w+env.mass_cons_correction; end;
        end
        function handles=Display(env,displayaxis)
            if numel(env.x)==0
                %Grid hasnt been built
                step=3;
                env.Grid(step);
            end
            [C,handles]=contour(displayaxis,env.x,env.y,env.z,10);
            %clabel(C,handles);
        end
        function Grid(env,step)
            %Form a grid of vertical velocities
            [x,y]=meshgrid(env.xlim(1):step:env.xlim(2),env.ylim(1):step:env.ylim(2));
            z=zeros(size(x));
            fprintf('Building grid')
            for i=1:size(x,1)
                for j=1:size(x,2)
                    z(i,j)=env.ExactMeasurement(x(i,j),y(i,j),0);
                end
                fprintf('.');
            end
            fprintf('\n');
            env.x=x;
            env.y=y;
            env.z=z;
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

