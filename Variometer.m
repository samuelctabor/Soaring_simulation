classdef Variometer < handle
    %UpdraftSensor Class to represent a sensor
    %   Sensor has a set covariance
    
    properties (SetAccess=protected)
        estimated_updraft;
        covariance;
    end
    
    methods
        function obj=Variometer(covariance)
            obj.covariance=covariance;
        end
        function update(obj,exact_updraft)
            %Add noise
            obj.estimated_updraft = exact_updraft+obj.covariance*randn; %TODO check; Should be wrong, see other sensor
        end
    end
    
end

