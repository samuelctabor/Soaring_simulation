classdef Heading_Controller < handle
    %Heading_Controller Implemenents PID control of aircraft heading
    %   Detailed explanation goes here
    
    properties
        k_p=0;
        k_d=0;
        k_i=0;
        max_output = 1.0;
    end
    properties (SetAccess=private)
        error_int=0;
        prev_error=0;
    end
    methods
        function obj=Heading_Controller(k_p,k_d,k_i,max_turnrate)
            obj.k_p=k_p;
            obj.k_d=k_d;
            obj.k_i=k_i;
            obj.max_output = max_turnrate;
        end
        function output=update(obj,error)
            %obj.error_int=obj.error_int*0.9;
            obj.error_int=obj.error_int+error;
            error_deriv = error-obj.prev_error;
            obj.prev_error=error;
            
            output = obj.k_p * error + obj.k_d * error_deriv + obj.k_i * obj.error_int;
            
            if output>obj.max_output
                output = obj.max_output;
            elseif (output<-obj.max_output)
                output = - obj.max_output;
            end
            
        end
        function reset_I(obj)
            obj.error_int=0;
        end
    end
end

