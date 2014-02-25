classdef StateMachine < handle
    %StateMachine Class just to keep track of what state we're in, how long
    %etc
    %   In ArduPilot 
    % CRUISING corresponds to AUTO
    % THERMALLING corresponds to LOITER
    % The other flight modes are not necessarily necessary
    
    properties(Constant)
        %state;
        %---------------------%
        %- State indicators---%
        %global searching thermaling cruising investigating
        searching = 1;
        thermalling= 2;
        cruising  = 3;
        investigating_straight=4;
        investigating_curve=5;
    end
    properties(SetAccess=protected)
        state_time=-10;
        printfnct;
        state=1;
    end
    methods
        function obj=StateMachine(printfnct)
            obj.printfnct=printfnct;
        end
        function set(obj,state,time)
            obj.state_time=time;
            obj.state=state;
            switch state
                case obj.thermalling
                    obj.print('State THERMALLING')
                case obj.searching
                    obj.print('State SEARCHING')
                case obj.cruising
                    obj.print('State CRUISING')
                case obj.investigating_straight
                    obj.print('State INVESTIGATING - straight')
                case obj.investigating_curve
                    obj.print('State INVESTIGATING - curve')
            end
        end
        function t=elapsed_time(obj,currenttime)
            t=currenttime-obj.state_time;
        end
        function print(obj,message)
            obj.printfnct(message);
        end
        
    end
    
end

