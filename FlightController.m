classdef FlightController < handle
    %FlightController Clas representing the control algorithm of the
    %aircraft.
    %   FlightController maintains and EKF, a StateMachine and a
    %   LowPassFilter.
    %   The update() method updates these components and calls update_state()
    %   and determine_turnrate().
    %   update_state() determines whether to switch control mode based
    %   on available data (position and velocity, estimated updraft and
    %   updraft gradient).
    %   determine_turnrate() then calculates turnrate according to the current flight
    %   mode.
    
    
    properties
        Waypoints;
        currentWaypoint=1;
    end
    properties (SetAccess=protected)
        variables;              %Holds the variables we want to be configurable
        turnrate=0;
        sm;                     %StateMachine
        ekf;
        heading_controller;
        
        V;
        pathangle;
        thermalability;
        est_thermal_pos;
        lpf;
        prev_time;
        sinkrate;
        current_time;
        deltaT;
        
        posx;
        posy;
        posz;
        printfnct;
        
        map;
        
        pf;
        
        %For filter
        prev_posx;
        prev_posy;
        prev_posz;
    end
    properties (SetAccess=private)
        %Derivative variables
        
        pathangleold=0;
        
        distance_to_next_wp=0;
        
        search_centre = [0,0];
        
        nav_bearing=0;
        
        filter_skips;
        filter_iterations;
        
        
    end
    methods
        function this=FlightController(variables,sinkrate,posx,posy,posz,V,pathangle,printfnct)
            this.posx=posx;
            this.posy=posy;
            this.posz=posz;
            
            % For filter
            this.prev_posx=posx;
            this.prev_posy=posy;
            this.prev_posz=posz;
            
            this.V=V;
            this.pathangle=pathangle;
            this.pathangleold=pathangle;
            
            this.nav_bearing=this.pathangle;
            
            this.variables=variables;
            this.printfnct=printfnct;
            Vref=15;
            this.printfnct('Initialised');
            
            %Derivative variables
            this.sinkrate=sinkrate;
            
            this.lpf = LowPassFilter(0.9);
            
            this.map = ThermalMap(@this.print);
            this.pf = PathFinder(this.map);
            
            this.sm=StateMachine(@this.print);
            %this.sm.set(StateMachine.searching,0);
            this.sm.set(StateMachine.cruising,0);
            
            max_turnrate = 9.81*this.V*tan(deg2rad(60));
            this.heading_controller = Heading_Controller(variables.k_p,variables.k_d,variables.k_i,max_turnrate);
            this.search_centre = [posx,posy+(5)];
            this.est_thermal_pos = [0,0];
            
            %---------------------%
            %----Set up ekf-------%
            n=4;
            m=2;
            x=[5;15;-5;0];                                   % inital state estimate
            P = diag([1 10 100 100]);    %Initial state unknown so large variance
            
            q=0.1;    %std of process
            q = q*50/variables.filter_rate;
            
            %r=0.1;    %std of measurement
            Q=q^2*eye(n); % covariance of process
            R=zeros(m,m);
            R(1,1) = variables.measurement_noise^2;
            R(2,2) = variables.measurement_noise_z2^2;
            
            this.ekf=ExtendedKalmanFilter_thermal(P,x,Q,R);
            %obj.ekf=ExtendedKalmanFilter_arduino(P,x,Q,R);
            
            this.filter_skips=floor(50/variables.filter_rate);
            this.filter_iterations=0;
            
        end
        function update(this,measurements,posx,posy,posz,pathangle,V,time)
            this.prev_time=this.current_time;
            this.current_time=time;
            this.deltaT=time-this.prev_time;
            if numel(this.deltaT)==0
                this.deltaT=0.02;
            end
            %Assume these come from GPS and are relatively exact.
            this.posx=posx;
            this.posy=posy;
            this.posz=posz;
            
            %Provide measurement to low pass filter
            this.lpf.update(measurements(1));
            
            this.pathangle=pathangle;
            
            this.V = V;
            
            %Update the Kalman filter
            
            if this.sm.state==StateMachine.thermalling
                % Try 10HZ
                if (mod(this.filter_iterations,this.filter_skips)==0)
                %if 1 %((mod(this.current_time,0.1)<1e-6)||(mod(this.current_time,0.1)>0.099))
                    %this.ekf.update(measurements,V*this.deltaT*cos(this.pathangleold),V*this.deltaT*sin(this.pathangleold));
                    this.ekf.update(measurements,this.posx-this.prev_posx,this.posy-this.prev_posy,pathangle,this.variables.roll_param);
                    this.prev_posx = this.posx;
                    this.prev_posy = this.posy;
                    this.prev_posz = this.posz;
                    
                    %Estimated global position of thermal
                    this.est_thermal_pos = [posx+this.ekf.x(3),posy+this.ekf.x(4)];
                end
                %obj.print(sprintf('Cov %f %f %f %f',obj.ekf.P(1,1),obj.ekf.P(2,2),obj.ekf.P(3,3),obj.ekf.P(4,4)));
                this.filter_iterations = this.filter_iterations+1;
            end
            %Estimate the climb we can achieve
            this.thermalability=this.calc_thermalability(this.ekf.x,this.variables.thermalling_radius);
            
            %Distance to next waypoint
            this.distance_to_next_wp = norm([posx-this.Waypoints(this.currentWaypoint,1),posy-this.Waypoints(this.currentWaypoint,2)]);
            
            %Check to see if we should switch to a different flight mode.
            this.update_state;
            
            %Calculate desired aircraft heading
            this.determine_heading;
            
            %Determine the turn rate (roll angle) required to effect
            %heading
            this.determine_turnrate;
            
            %Save this pathangle for the next iteration. In real life
            %should perhaps use an average.
            this.pathangleold=this.pathangle;
            
        end
        function print(this,message)
            this.printfnct(sprintf('FC: %s',message));
        end
        function update_variable(this,name,value)
            this.variables.(name)=value;
        end
    end
    
    
    methods (Access=private)
        function update_state(this)
            t=this.current_time;
            switch this.sm.state
                case StateMachine.searching
                    %IF updraft is strong AND minimum search time is met
                    if this.lpf.filtered(1)>0.8*this.sinkrate && this.sm.elapsed_time(t)>this.variables.min_search_time
                        %Filter needs to be reset. Because it runs all the
                        %time, it will probably have decided during the
                        %search or cruise phase that a thermal exists but
                        %is small or far away. Reset it so the new data
                        %doesn't have to contend with the old.
                        %We reset it to 10m ahead of the aircraft, but give
                        %it a high covariance P so it will adjust quickly.
                        this.print('Filter reset');
                        this.ekf.reset([5;15;cos(this.pathangle)*10;sin(this.pathangle)*10],diag([2, 10, 20, 20]));
                        this.sm.set(StateMachine.thermalling,t);
                        this.heading_controller.reset_I();
                        this.turnrate=0;
                        this.prev_posx = this.posx;
                        this.prev_posy = this.posy;
                        this.prev_posz = this.posz;
                    end
                case StateMachine.thermalling
                    if (this.sm.elapsed_time(t)>this.variables.min_thermal_latch_time);
                        if (this.posz>this.variables.ceiling)
                            %Altitude limit
                            this.print('Topped out.');
                            this.add_estimate_to_map();
                            this.sm.set(StateMachine.cruising,t);
                            this.heading_controller.reset_I();
                            this.turnrate=0;
                        elseif (this.thermalability<FlightController.MacCready(this.posz,this.sinkrate))
                            this.print(sprintf('MacCready speed not met (%2.2f/%2.2f)',this.thermalability,FlightController.MacCready(this.posz,this.sinkrate)));
                            this.add_estimate_to_map();
                            this.sm.set(StateMachine.cruising,t);
                            this.heading_controller.reset_I();
                            this.turnrate=0;
                        end
                    end
                case StateMachine.cruising
                    %Do we need to move to next waypoint?
                    if norm([this.posx,this.posy]-this.Waypoints(this.currentWaypoint,:))<this.variables.Waypointtol
                        this.print(sprintf('Reached waypoint %d',this.currentWaypoint));
                        if (this.currentWaypoint)==size(this.Waypoints,1)
                            %Final waypoint. Start again.
                            this.print('At final waypoint.');
                            this.heading_controller.reset_I();
                            this.currentWaypoint=1;
                        else
                            %Go to next waypoint.
                            this.currentWaypoint=this.currentWaypoint+1;
                            this.print(sprintf('Waypoint %d next',this.currentWaypoint));
                            this.heading_controller.reset_I();
                        end
                        %newpath=pf.plan([this.posx,this.posy,this.posz],[80,80,10])
                        %Now the actual route array should be updated
                    end
                    %Is there a thermal we should catch?
                    %Depends how close to destination, how much altitude,
                    %how strong a thermal.
                    %incentive = obj.posz, obj.lpf.filtered, obj.distance_to_next_wp
                    %incentive=this.sinkrate*2;
                    if this.sm.elapsed_time(t)>this.variables.min_cruise_time
                        incentive = FlightController.MacCready(this.posz,this.sinkrate)*0.5;
                        if this.lpf.filtered > incentive
                            this.print(sprintf('Incentive met (%2.2f/%2.2f)',this.lpf.filtered,FlightController.MacCready(this.posz,this.sinkrate)*0.5));
                            this.print('Filter reset');
                            this.ekf.reset([5;15;cos(this.pathangle)*10;sin(this.pathangle)*10],diag([2, 10, 20, 20]));
                            %obj.sm.set(StateMachine.investigating_straight,t)
                            this.sm.set(StateMachine.thermalling,t)
                            this.heading_controller.reset_I();
                            this.prev_posx = this.posx;
                            this.prev_posy = this.posy;
                            this.prev_posz = this.posz;
                        end
                        %Could also have some conditions to enter search mode,
                        %eg. if altitude is getting low
                        if (this.posz < this.variables.begin_search_altitude)
                            this.sm.set(StateMachine.searching,t);
                            this.search_centre=[this.posx,this.posy];
                        end
                    end
            end
            
        end
        function determine_heading(this)
            switch this.sm.state
                case StateMachine.searching
                    %Fly a spiral pattern
                    angle = atan2(this.search_centre(2)-this.posy,this.search_centre(1)-this.posx);
                    this.nav_bearing = angle - ((pi/2) + this.variables.search_pitch_angle);
                case StateMachine.thermalling
                    %Orbit the thermal centre
                    this.nav_bearing=this.calc_bearing_thermalling(this.ekf.x,this.pathangle,this.variables);
                case StateMachine.cruising
                    %Navigate towards waypoint
                    wp=this.Waypoints(this.currentWaypoint,:);
                    this.nav_bearing = atan2(wp(2)-this.posy,wp(1)-this.posx);
            end
        end
        
        function determine_turnrate(this)
            angle_error = FlightController.wrap360(this.nav_bearing-this.pathangle);
            this.turnrate = this.heading_controller.update(angle_error);
        end
        
        function add_estimate_to_map(this)
            this.map.add_data_point_filter(this.posx,this.posy,this.ekf.x,this.ekf.P);
        end
    end
    
    methods (Static)
        
        function nav_bearing = calc_bearing_thermalling(x, pathangle, variables)
            rad = variables.thermalling_radius;
            dist = norm([x(3),x(4)]);
            if dist < rad
                % Aim at 90* to thermal, plus a bit to widen the turn
                nav_bearing = atan2(x(4),x(3));
                error = FlightController.wrap360(nav_bearing-pathangle);
                nav_bearing = nav_bearing - sign(error)*(pi/2+deg2rad(5));
            else     % If outside the circle, aim at tangent
                nav_bearing = atan2(x(4),x(3))+asin(rad/dist);     % Aim for the tangent to the circle
            end
        end
        function nav_bearing = calc_bearing_cruising(posx,posy,pathangle,destination, variables)
            nav_bearing = atan2(destination(2)-posy,destination(1)-posx);
        end
        function result = calc_thermalability(x,r)
            %Estimate the achievable climb. This depends on all three of:
            %thermal strength (x(1)), radius (x(2)) and the radius at which
            %aircraft circles. This avoids the aircraft circling estimated
            %thermals which have converged to high strength but very small
            %radius.
            result=x(1)*exp(-r^2/x(2)^2);
        end
        function result=wrap360(angle)
            if angle<-pi
                result=angle+2*pi;
            elseif angle>pi
                result=angle-2*pi;
            else
                result=angle;
            end
        end
        function vmc=MacCready(alt,sinkrate)
            h = [0 50 100 150 200 250 300 350 400];
            v = 2.0*(h/400); %.^0.3;
            vmc = interp1(h,v,alt,'linear') + sinkrate;
        end
        
    end
end

