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
            obj.environment=environment;
            obj.sinkrate=sinkrate;
            obj.roll_param = variables.roll_param;
            
            reset_state(obj, posx, posy, posz, V, pathangle);
            reset_history(obj);
        end
        
        function reset(this, posx, posy, posz, V, pathangle, execution_frequency, Waypoints, nr_iterations)
            this.controller=FlightController(this.controller.variables, this.sinkrate,posx,posy,posz,V,pathangle,@this.print,execution_frequency,Waypoints);
            reset_state(this, posx, posy, posz, V, pathangle);
            reset_history(this, nr_iterations);
        end
        function reset_state(this, posx, posy, posz, V, pathangle)
            this.posx=posx;
            this.posy=posy;
            this.posz=posz;
            this.V=V;
            this.pathangle=pathangle;
            this.vx = V*cos(pathangle);
            this.vy = V*sin(pathangle);
            this.vz=this.sinkrate;
            this.previous_time=0;
            this.landed=0;
        end
        function reset_history(this, nr_iterations)
            if (nargin<2)
                nr_iterations = 0; %Do not pre-initialize history arrays then...
            end
            
            if(this.controller.variables.SaveReducedHistory) nr_iterations = nr_iterations/4; end
            this.History.idx = 1;
            this.History.t=[0.0, zeros(1,nr_iterations)];
            this.History.p=[this.posx,this.posy,this.posz ; zeros(nr_iterations,3)];
            this.History.v=[this.vx,this.vy,this.vz ; zeros(nr_iterations,3)];
            this.History.z=[0,0 ; zeros(nr_iterations,2)];
            this.History.kf.z_exp=[zeros(1,2) ; zeros(nr_iterations,2)];
            this.History.kf.x = [zeros(1,4) ; zeros(nr_iterations,4)];
            this.History.kf.x_xy_glob = [zeros(1,2) ; zeros(nr_iterations,2)];
            this.History.kf.P = [zeros(1,4) ; zeros(nr_iterations,4)];
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

            %Update history. Note that the memory for the arrays is
            %usually pre-allocated, but matlab extends it automatically if required...
            if(~obj.controller.variables.SaveReducedHistory || mod(time,0.2)<0.05)
                obj.History.idx=obj.History.idx+1;
                obj.History.t(obj.History.idx) = time;
                obj.History.p(obj.History.idx,:) = [obj.posx,obj.posy,obj.posz];
                obj.History.v(obj.History.idx,:) = [obj.vx,obj.vy,obj.vx];
                obj.History.z(obj.History.idx,:) = measurements;
                obj.History.kf.z_exp(obj.History.idx,:) = obj.controller.kf.z_exp';
                obj.History.kf.x(obj.History.idx,:) = obj.controller.kf.x';
                obj.History.kf.x_xy_glob(obj.History.idx,:) = obj.controller.est_thermal_pos;
                obj.History.kf.P(obj.History.idx,:) = [obj.controller.kf.P(1,1) obj.controller.kf.P(2,2) obj.controller.kf.P(3,3) obj.controller.kf.P(4,4)];
            end
            
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
            colours = (obj.vz(1) + 10)/(10--10);
            colours = max(min(colours,1.0),0.0);
            C=colormap(axis,'jet');
            colours=ceil(colours*(length(C)-1)+1);

            obj.Clear();
            obj.h_patch=Aircraft.display_ac_patch(axis,obj.posx,obj.posy,obj.posz,C(colours,:),obj.pathangle);
            switch obj.controller.sm.state
                case StateMachine.thermalling
                    [obj.h_objective(1),obj.h_objective(2)]=Aircraft.display_objective(axis,obj.controller.est_thermal_pos(1),obj.controller.est_thermal_pos(2),obj.posx,obj.posy,obj.posz,'r:^','r-.');
                    if(obj.controller.ThermalTrackingActive==false) [obj.h_objective(3),obj.h_objective(4)]=Aircraft.display_objective(axis,obj.controller.Waypoints(obj.controller.currentWaypoint,1),obj.controller.Waypoints(obj.controller.currentWaypoint,2),obj.posx,obj.posy,obj.posz,'b:^','b-.'); end;
                    
                    if(obj.controller.thermal_estimate_updated)
                        obj.controller.thermal_estimate_updated = false;
                        obj.h_thermal = Aircraft.display_thermal_cov(axis,obj.controller.est_thermal_pos(1),obj.controller.est_thermal_pos(2),obj.posz,obj.controller.kf.P(3,3),obj.controller.kf.P(4,4));

                        %If the UKF or the PF is active, plot the (valid) sigma points or particles
                        if(obj.controller.KFtype==2 || obj.controller.KFtype==3)
                            obj.h_sigmaPoints = zeros(0);
                            num_points_to_plot = size(obj.controller.sigma_points_glob,2);
                            if(num_points_to_plot>0) obj.h_sigmaPoints = plot(axis,obj.controller.sigma_points_glob(3,:),obj.controller.sigma_points_glob(4,:),'o','MarkerSize',3); end;
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
                if(obj.controller.thermal_estimate_updated)
                    delete(obj.h_thermal);
                    delete(obj.h_sigmaPoints);
                end
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

