classdef RollMomentSensor < handle
    %RollMomentSensor Class to represent a sensor that measures the roll
    %moment induced by the thermal updraft
    %   Sensor has a set covariance
    
    properties (SetAccess=protected)
        estimated_roll_moment;
        covariance;
    end
    
    methods
        function obj=RollMomentSensor(covariance)
            obj.covariance=covariance;
        end
        function update(obj,exact_roll_moment)
            %Add noise
            obj.estimated_roll_moment = exact_roll_moment+sqrt(obj.covariance)*randn; %TODO check
        end
    end
    
end

