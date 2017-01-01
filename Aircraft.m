classdef Aircraft < handle
    %UNTITLED10 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vz=0;
        turnrate;
        pathangle;
        V;
        roll_param;
        controller;
        gradientsensor;
        updraftsensor;
        environment;
        name;
    end
    properties (SetAccess=private)
        posx=0
        posy=0
        posz=0
        vx
        vy
        previous_time=0;
        sinkrate
        History
        h_label;
        h_objective;
        h_thermal;
        h_patch;
        h_map;
        h_sigmaPoints;
        landed=0;
    end
    methods
        function obj=Aircraft(posx,posy,posz,V,pathangle,variables,sinkrate,environment,name,execution_frequency, Waypoints)
            obj.name=name;
            obj.controller=FlightController(variables,sinkrate,posx,posy,posz,V,pathangle,@obj.print,execution_frequency, Waypoints);
            obj.updraftsensor = Variometer(variables.actual_noise);
            obj.gradientsensor = RollMomentSensor(variables.actual_noise_z2);
            
            obj.posx=posx;
            obj.posy=posy;
            obj.posz=posz;
            obj.V=V;
            obj.pathangle=pathangle;
            obj.environment=environment;
            obj.sinkrate=sinkrate;
            obj.vx = V*cos(pathangle);
            obj.vy = V*sin(pathangle);
            obj.vz=obj.sinkrate;
            
            obj.roll_param = variables.roll_param;
            
            obj.History.t=0.0;
            obj.History.p=[posx,posy,posz];
            obj.History.v=[obj.vx,obj.vy,obj.vz];
            obj.History.z=[0,0];
            obj.History.kf.z_exp=zeros(1,2); %TODO Check for shift in time by 1 step
            obj.History.kf.x = zeros(1,4);
            obj.History.kf.x_xy_glob = zeros(1,2);
            obj.History.kf.P = zeros(1,4);
        end
        function update(obj,time)
            if obj.posz<0
                % it's landed
                if ~obj.landed
                    obj.print(sprintf('Landed at %4.1f seconds',obj.controller.current_time));
                    obj.landed=1;
                end
                
                return;
            end
            
            %fprintf('deltaT:\n');
            deltaT=time-obj.previous_time;
            %Compute measurements
            [z_R,z_L]=obj.environment.ExactMeasurement(obj.posx,obj.posy,obj.pathangle); %TODO Pathangle for now, but this is wrong, should be yaw -> Add
            roll = 0; %Assume zero roll angle for now
            z_L = z_L * cos(roll) * obj.roll_param; %Adapt exact measurement with aircraft specific parameters
            obj.updraftsensor.update(z_R);
            obj.gradientsensor.update(z_L);
            
            measurements=[obj.updraftsensor.estimated_updraft obj.gradientsensor.estimated_roll_moment];
            
            obj.controller.update(measurements,obj.posx,obj.posy,obj.posz,obj.pathangle,obj.V,time);

            %Update history
            obj.History.t(end+1) = time;
            obj.History.p(end+1,:) = [obj.posx,obj.posy,obj.posz];
            obj.History.v(end+1,:) = [obj.vx,obj.vy,obj.vx];
            obj.History.z(end+1,:) = measurements;
            obj.History.kf.z_exp(end+1,:) = obj.controller.kf.z_exp';
            obj.History.kf.x(end+1,:) = obj.controller.kf.x';
            obj.History.kf.x_xy_glob(end+1,:) = obj.controller.est_thermal_pos;
            obj.History.kf.P(end+1,:) = [obj.controller.kf.P(1,1) obj.controller.kf.P(2,2) obj.controller.kf.P(3,3) obj.controller.kf.P(4,4)];

            %Update state
            obj.posx = obj.posx + deltaT*obj.vx;
            obj.posy = obj.posy + deltaT*obj.vy;
            obj.posz = obj.posz + deltaT*obj.vz;
            
            obj.vx = cos(obj.pathangle)*obj.V;
            obj.vy = sin(obj.pathangle)*obj.V;
            obj.vz = z_R - obj.sinkrate;
            
            obj.pathangle = obj.pathangle + deltaT*obj.controller.turnrate;
            if obj.pathangle > pi
                obj.pathangle=obj.pathangle - 2*pi;
            elseif obj.pathangle < -pi
                obj.pathangle=obj.pathangle + 2*pi;
            end
            
            obj.previous_time = time;
        end
        function Display(obj,axis)
            obj.Clear();
            colours = (obj.vz(1) + 10)/(10--10);
            colours = max(min(colours,1.0),0.0);
            C=colormap(axis,'jet');
            colours=ceil(colours*(length(C)-1)+1);
            obj.h_patch=Aircraft.display_ac_patch(axis,obj.posx,obj.posy,obj.posz,C(colours,:),obj.pathangle);
            switch obj.controller.sm.state
                case StateMachine.thermalling
                    [obj.h_objective(1),obj.h_objective(2)]=Aircraft.display_objective(axis,obj.controller.est_thermal_pos(1),obj.controller.est_thermal_pos(2),obj.posx,obj.posy,obj.posz,'r:^','r-.');
                    if(obj.controller.ThermalTrackingActive==false) [obj.h_objective(3),obj.h_objective(4)]=Aircraft.display_objective(axis,obj.controller.Waypoints(obj.controller.currentWaypoint,1),obj.controller.Waypoints(obj.controller.currentWaypoint,2),obj.posx,obj.posy,obj.posz,'b:^','b-.'); end;
                    obj.h_thermal = Aircraft.display_thermal_cov(axis,obj.controller.est_thermal_pos(1),obj.controller.est_thermal_pos(2),obj.posz,obj.controller.kf.P(3,3),obj.controller.kf.P(4,4));

                    %If the UKF is active, plot the (valid) sigma points
                    if(obj.controller.KFtype==2 && numel(obj.controller.kf.sigma_points)>0)
                        obj.h_sigmaPoints = plot(axis,obj.controller.sigma_points_glob(3,1),obj.controller.sigma_points_glob(4,1),'o','MarkerSize',obj.controller.sigma_points_glob(2,1)/max(obj.controller.sigma_points_glob(2,:))*5.0);
                        for i = 2:2*numel(obj.controller.kf.x)+1
                            obj.h_sigmaPoints(end+1) = plot(axis,obj.controller.sigma_points_glob(3,i),obj.controller.sigma_points_glob(4,i),'o','MarkerSize',obj.controller.sigma_points_glob(2,i)/max(obj.controller.sigma_points_glob(2,:))*5.0);
                        end
                    end
                case StateMachine.searching
                    [obj.h_objective(1),obj.h_objective(2)]=Aircraft.display_objective(axis,obj.controller.search_centre(1),obj.controller.search_centre(2),obj.posx,obj.posy,obj.posz,'g:^','g-.');
                case StateMachine.cruising
                    [obj.h_objective(1),obj.h_objective(2)]=Aircraft.display_objective(axis,obj.controller.Waypoints(obj.controller.currentWaypoint,1),obj.controller.Waypoints(obj.controller.currentWaypoint,2),obj.posx,obj.posy,obj.posz,'b:^','b-.');
                otherwise
                    fprintf('Error: state %d\n',obj.controller.sm.state);
            end
            
            obj.h_map=Aircraft.display_map(axis,obj.controller.map);

            obj.h_label = text(axis, obj.posx+30,obj.posy,obj.posz,sprintf('%s \nx,y:%3.0f/%3.0f m \nHeight %3.1f m \nVertical Velocity: %2.2f \nPA_cor: %3.1f deg\nz1/z1e/r: %2.2f/%2.2f/%2.2f m/s \nz2/z2e/r: %1.3f/%1.3f/%1.3f Nm \n W/R/xg/yg %2.2f/%2.2f/%2.2f/%2.2f \nP:%3.2f/%3.2f/%3.2f/%3.2f',...
                obj.name, obj.posx, obj.posy,obj.posz, obj.vz, -rad2deg(obj.pathangle-deg2rad(90)), ...
                obj.updraftsensor.estimated_updraft, obj.controller.kf.z_exp(1),obj.updraftsensor.estimated_updraft-obj.controller.kf.z_exp(1), ...
                obj.gradientsensor.estimated_roll_moment, obj.controller.kf.z_exp(2),obj.gradientsensor.estimated_roll_moment-obj.controller.kf.z_exp(2),...
                obj.controller.kf.x(1),obj.controller.kf.x(2),obj.controller.est_thermal_pos(1),obj.controller.est_thermal_pos(2),...
                obj.controller.kf.P(1,1),obj.controller.kf.P(2,2),obj.controller.kf.P(3,3),obj.controller.kf.P(4,4)));
        end
        function print(obj, message)
            fprintf('%s: %s\n',obj.name,message);
        end
        function Clear(obj)
            try
                delete(obj.h_label);
                delete(obj.h_objective);
                obj.h_objective = [];
                delete(obj.h_patch);
                delete(obj.h_map);
                delete(obj.h_thermal);
                delete(obj.h_sigmaPoints);
            catch
            end
        end
        
    end
    methods(Static)
        function [handle1,handle2]=display_objective(axis,x,y,objx,objy,objz,str1,str2)
            handle1=plot3(axis,[x,x],...
                [y,y],...
                [0,objz],str1);
            handle2=plot3(axis,  [objx,x],...
                [objy,y],...
                [objz,objz],str2);
        end
        
        function h = display_ac_patch(axis,xp,yp,zp,c,pathangle)
            ywing=15;
            ystab=3.5;
            yfuse=0.6;
            ytail=0.5;
            xnose=-1.0;
            xwingf=4;
            xwingro=5;
            xwingri=5.5;
            xtailf=10.0;
            xtailr=11.0;
            x=[xnose,xwingf,xwingf,xwingro,xwingri,xtailf,xtailf,xtailr];
            
            y=[0,yfuse,ywing,ywing,yfuse,ytail,ystab,ystab];
            
            x=[x,fliplr(x)];
            y=[y,-fliplr(y)];
            
            x=-x;
            
            f=3/max(y);
            y=y*f;
            x=x*f;
            
            xn = x*cos(pathangle) - y*sin(pathangle);
            yn = y*cos(pathangle) + x*sin(pathangle);
            x=xn;
            y=yn;
            
            h=patch(axis,x+xp,y+yp,ones(size(x))*zp,c);
            
        end
        
        function h=display_thermal_cov(axis,x,y,z,px,py)
            th=0:0.1:2*pi;
            xi = sqrt(px) * cos(th);
            yi = sqrt(py) * sin(th);
            xi = xi + x;
            yi = yi + y;
            zi = z*ones(size(xi));
            h = plot3(axis,xi,yi,zi,'r-');
            
        end
        
        function h=display_map(axis,map)
            idx = map.datapoints(:,3)~=0;
            h=plot3(axis,map.datapoints(idx,1),map.datapoints(idx,2),0*map.datapoints(idx,1),'k^');
        end
         
    end
end

