classdef Environment_conc < handle
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
    end
    
    methods
        function obj=Environment_conc(xlim,ylim,mean_length,~,plot_axis,varargin)
            thermal_types = {@GaussianThermal,@FlightGearThermal};
            %thermal_type = thermal_types{ceil(rand(1)*2)};
            
            
            obj.xlim=xlim;
            obj.ylim=ylim;

            
            
            % How many thermals should there be in these limits?
            % l = 1/(n*sigma);
            % sigma = diameter = 40;
            % n = concentration = 1/(l*sigma);
            n = 1/(mean_length * 40);
            % Now number N = area*n
            Area = (xlim(2)-xlim(1))*(ylim(2)-ylim(1));
            number_thermals = floor(Area*n);
            
            
            xloc=xlim(1) + (xlim(2)-xlim(1))*rand(1,number_thermals);
            yloc=xlim(1) + (ylim(2)-ylim(1))*rand(1,number_thermals);
            
            for i=1:number_thermals    
                %h=thermal_type(xloc(i),yloc(i),5,20);
                h=thermal_types{ceil(rand(1)*2)}(xloc(i),yloc(i),3,20);
                thermals{i}=h;
            end
            obj.Thermals=thermals;
            
            step = sqrt(Area)/100;
            obj.Grid(step);
            obj.mass_cons_correction = -1*mean(mean(obj.z));
            obj.Display(plot_axis);
        end
        function [w,grad]=ExactMeasurement(env,x,y)
           w=0;
           grad=[0;0];
           for i=1:length(env.Thermals)
               %x;
               %y;
               [wi,gradi]=env.Thermals{i}.ExactMeasurement(x,y);
               w=w+wi;
               grad=grad+gradi;
           end
           w=w+env.mass_cons_correction;
        end
        function handles=Display(env,displayaxis)
            if numel(env.x)==0
                %Grid hasnt been built
                step=3;
                env.Grid(step);
            end
            [C,handles]=contour(displayaxis,env.x,env.y,env.z);
            %clabel(C,handles);
        end
        function Grid(env,step)
            %Form a grid of vertical velocities
            [x,y]=meshgrid(env.xlim(1):step:env.xlim(2),env.ylim(1):step:env.ylim(2));
            z=zeros(size(x));
            fprintf('Building grid')
            for i=1:size(x,1)
                for j=1:size(x,2)
                    z(i,j)=env.ExactMeasurement(x(i,j),y(i,j));
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

