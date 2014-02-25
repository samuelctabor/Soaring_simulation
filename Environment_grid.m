classdef Environment_grid < handle
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
        grid;
        dx;
        dy;
    end
    
    methods
        function obj=Environment_grid(xlim,ylim,mean_length,~,plot_axis,varargin)
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
            
           
            
            
            %identify grid square
            %use ~100mx100m squares
            nx = floor((xlim(2)-xlim(1))/100);
            obj.dx = (xlim(2)-xlim(1))/nx;
            ny = floor((ylim(2)-ylim(1))/100);
            obj.dy = (ylim(2)-ylim(1))/ny;
            
% 
%             for ii=[1,nx+2];
%                 for jj=[1:ny+2]
%                     obj.grid(ii,jj).thermals=[];
%                 end
%             end
            obj.grid=zeros(nx+2,ny+2,10);
            idxs=zeros(nx+2,ny+2);    

            for i=1:number_thermals

                xi = ceil((thermals{i}.thermalx - xlim(1))/obj.dx)+1; %Plus one to leave a border
                yi = ceil((thermals{i}.thermaly - ylim(1))/obj.dy)+1;
                
                for ii=(xi-1:xi+1)
                    for jj=(yi-1:yi+1)
                        try
                            idxs(ii,jj)=idxs(ii,jj)+1;
                            obj.grid(ii,jj,idxs(ii,jj))=i;
                        catch err
                            fprintf('Error');
                        end
                        
                    end
                end
            end
            
            step = sqrt(Area)/100;
            obj.Grid(step);
            obj.mass_cons_correction = -1*mean(mean(obj.z));
            
            
            obj.Display(plot_axis);
        end
        function [w,grad]=ExactMeasurement(env,x,y)
            w=0;
            grad=[0;0];

            xi = floor((x - env.xlim(1))/env.dx)+1; %Plus one to leave a border
            yi = floor((y - env.ylim(1))/env.dy)+1;
            
            close_thermals = env.grid(xi,yi,:);
            close_thermals=close_thermals(close_thermals~=0);
            
            for i=1:length(close_thermals)
                %x;
                %y;
                thermal_id = close_thermals(i);
                [wi,gradi]=env.Thermals{thermal_id}.ExactMeasurement(x,y);
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

