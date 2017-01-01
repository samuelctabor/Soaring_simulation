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
        execution_frequency = 25; %Simulation execution frequency [Hz]
        fastforwardfactor = 1.0; %Fast forward for simulation speed up (dt stays same, this is just to calculate faster!)
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
            variables.filter_rate           =       5; %Kalman Filtering frequency [Hz]
            variables.actual_noise          =       0.0;
            variables.actual_noise_z2       =       0.0;
            variables.measurement_noise     =       0.2; %This is the standard deviation
            variables.measurement_noise_z2  =       10000.1; %This is the standard deviation
            variables.process_noise_q1      =       0.001; %This is the standard deviation for W
            variables.process_noise_q2      =       0.1; %This is the standard deviation for R
            variables.process_noise_q3      =       0.2; %This is the standard deviation for x,y
            variables.thermalling_radius =          20;
            variables.roll_param            =       20.9537; %Techpod at nominal airspeed
            variables.bSimulateSilently     =       false; %Set to true to avoid all output (drawing & text)
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
            
            number_thermals = 1;
            X = 0;
            Y = 0;
            %X = [-70,  -20, 70, 0 , 0 ];
            %Y = [-60,  -20, 0,  50, -20];
            
            obj.environment=Environment([-size,size],[-size,size],number_thermals,@GaussianThermal,obj.axis,X,Y);
            %obj.environment=Environment_grid([-size,size],[-size,size],450,@FlightGearThermal,obj.axis,X,Y);
            %obj.environment=Environment_random([-size,size],[-size,size],5,@FlightGearThermal,obj.axis,X,Y);
            obj.environment.print;

            Points{1} = [-100,100,0; -100,-100,0]; %Straight line up
            Points{2} = [-50,100,0; 50,-100,0 ; 150,100,0]; %Zick-Zack
            Points{3} = [-100,100,0; -0,-50,1]; %Straight line up then open loop loiter near center
            ChooseWaypointsNr = 3;
            
            obj.TheAircraft=Aircraft(-100,-100,200,V,pathangle,variables,sinkrate,obj.environment,'Aircraft 1',obj.execution_frequency,Points{ChooseWaypointsNr});
            %aircraft2=Aircraft(-70,-50,200,V,pathangle,variables,sinkrate,obj.environment,'Aircraft 2');
        end
        
        function Update(obj,dt,bSimulateSilently)
            if(nargin<3); bSimulateSilently=false; end

            for i=1:length(obj.TheAircraft)
                obj.TheAircraft(i).update(obj.currenttime+dt);
            end
            
            if(~bSimulateSilently)
                title(obj.axis,sprintf('Time: %4.1f seconds',obj.currenttime));
                for i=1:length(obj.TheAircraft)
                    obj.TheAircraft(i).Display(obj.axis);
                end
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

