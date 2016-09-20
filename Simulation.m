classdef Simulation < handle
    %Simulation Class to control all aspects (environment, aircraft)
    %   The simulation class contains the aircraft and the environment.
    %   It also keeps track of aircraft positions.Updating the simulation 
    %   causes the aircraft positions to be updated according to their
    %   speed and heading angle. Heading angle is updated according to the
    %   turnrate commanded by the aircraft Flight Controler. 
    %   The Dislay method plots thermals and aircraft positions and objectives 
    %   onto the specified axis.
    %   The UpdateControlVariables method updates the control variables of
    %   the flight controllers of the aircraft in TheAircraft array.
    
    properties
        TheAircraft
        axis
        environment
        AcPlotHandles;
        ETPlotHandles
        ETvPlotHandles
        TCPlotHandles;
        MapPlotHandles;
        currenttime=0;
    end
    
    methods
        function obj=Simulation(axis_to_use)
            fprintf('Initialising simulation...\n');
            
            set(axis_to_use,'ButtonDownFcn',@obj.axis_clicked_fcn);
            
            pathangle = 0;
            
            V=15;
            
            sinkrate=V/10;
            
            %------------------------------%
            %------Control variables-------%
            %------------------------------%
            variables.search_latch_threshhold=      0.5*sinkrate;
            variables.cruise_latch_threshhold=      2.0*sinkrate;
            variables.ceiling =                     400;
            variables.begin_search_altitude =       -0;
            variables.filter_rate           =       10;
            variables.measurement_noise     =       0.0;%0.5; %TODO This should be the covariance. Is it?
            variables.measurement_noise_z2  =       0.0;%0.4; %TODO This should be the covariance. Is it?
            variables.thermalling_radius =          20;
            variables.roll_param            =       20.9537; %Techpod at nominal airspeed
            % turnrate = (g/V)*tan(phi)

           
            variables.search_pitch_angle =          deg2rad(5.0);
            variables.min_thermal_latch_time=       5;

            variables.floor=                        0;
            variables.Waypointtol                  =5;
            variables.min_cruise_time              =5;
            variables.min_search_time              =5;
            variables.k_p =                         2.0;
            variables.k_d =                         0.0;
            variables.k_i =                         0.01;

            obj.axis=axis_to_use;
            
            
            %figure;
            hold(obj.axis,'on');
            set(obj.axis, 'CameraViewAngle', get(obj.axis,'CameraViewAngle'));
            axis(obj.axis,'equal');
            %obj.axis=gca;
            t=0;
            grid on;
            size=100;
            % initial state covraiance
            N=1000;                                          % total dynamic steps
            %xV = zeros(n,N);          %estmate              % allocate memory
            
            %pV = zeros(3,N);
            
            %vV=zeros(3,N);
            
            X = [-70,  -20, 70, 0 , 0 ];
            Y = [-60,  -20, 0,  50, -20];
            
            obj.environment=Environment([-size,size],[-size,size],5,@GaussianThermal,obj.axis,X,Y);
            %obj.environment=Environment_grid([-size,size],[-size,size],450,@FlightGearThermal,obj.axis,X,Y);
            %obj.environment=Environment_random([-size,size],[-size,size],5,@FlightGearThermal,obj.axis,X,Y);
            
            obj.environment.print;
            
            
            
            aircraft1=Aircraft(-60,-0,200,V,pathangle,variables,sinkrate,obj.environment,'Aircraft 1');
            %aircraft2=Aircraft(-70,-50,200,V,pathangle,variables,sinkrate,obj.environment,'Aircraft 2');
            
            
            obj.TheAircraft=[aircraft1];%,aircraft2];

            Points = [100,100; -100,-100];
            
            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).controller.Waypoints = Points;
                %obj.TheAircraft(i).controller.sm.set(StateMachine.cruising, obj.currenttime);
            end
        end
        
        function Update(obj,dt)
            title(obj.axis,sprintf('Time: %4.1f seconds',obj.currenttime));
            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).update(obj.currenttime+dt);
            end
            
            
            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).Display(obj.axis);
            end
            
            obj.currenttime = obj.currenttime+dt;
            
        end
        
        
        function UpdateControlVariables(obj,variables)
            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).controller.variables = variables;
            end
        end
        
        function axis_clicked_fcn(obj,h,~)
            point = get(h,'CurrentPoint');
            new_waypoint = [point(1,1),point(1,2)];
            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).controller.Waypoints(end+1,:) = new_waypoint;
            end
            fprintf('New WP at [%3.1f,%3.1f]\n',point(1,1),point(1,2));
        end
    end
    
end

