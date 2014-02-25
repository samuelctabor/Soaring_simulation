classdef GPS < handle
    %GPS class implementing GPS functions
    %   Detailed explanation goes here
    
    properties (SetAccess=protected)
        covariance=[0,0,0];
        lat=0;
        long=0;
        height=0;
        R=6371E3;
        base=[0,0,0];
        speed=[0,0,0];
    end
    
    methods
        function obj=GPS(covariance,base)
            obj.covariance=covariance;
            obj.base=base;  %GPS location of 0,0,0 point
        end
        function update(obj,exact_position_xyz,exact_speed_xyz)
            obj.lat=obj.base(1)+rad2deg((exact_position_xyz(1)/obj.R));
            R1=obj.R*sin(deg2rad(obj.lat));
            obj.long=obj.base(2)+rad2deg((exact_position_xyz(2)/R1));
            obj.height=obj.base(3)+exact_position_xyz(3)+obj.R;
            obj.speed=exact_speed_xyz;
        end
        function [lat,long,height]=get_GPS_reading(obj)
            lat=obj.lat         +randn*obj.covariance(1);
            long=obj.long       +randn*obj.covariance(2);
            height=obj.height   +randn*obj.covariance(3);
        end
    end
    
end

