classdef LowPassFilter < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x=0.8;      %Filtering coefficient - determines time constant
        filtered=0;
    end
    
    methods
        function obj=LowPassFilter(x)
            obj.x=x;
        end
        function update(LPF,z)
            LPF.filtered=LPF.x*LPF.filtered + (1-LPF.x)*z;
        end
    end
    
end

