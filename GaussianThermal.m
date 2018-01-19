classdef GaussianThermal
    %GaussianThermal Class to represent a Gaussian thermal updraft
    %distribution.
    %   The Gaussian distribution is used in the literature as a model of
    %   thermal updraft distribution. It is of the form w =
    %   W*exp(-r^2/R^2). The thermal therefore has properties W,R,x and y
    %   representing thermal strength, size, and location.
    %   The localgradient is used to later on calculate a thermal-induced
    %   roll moment as described in the Journal of Field Robotics publication
    %   at www.dx.doi.org/10.1002/rob.21765
    
    properties
        thermalx=0;
        thermaly=0;
        strength=5.0;
        radius=10.0;
    end
    
    methods
        function GT=GaussianThermal(varargin) %(x,y,s,r)
            if nargin>0
                GT.thermalx=double(varargin{1});
                GT.thermaly=double(varargin{2});
                GT.strength=double(varargin{3});
                GT.radius  =double(varargin{4});
            end
        end
        function [z, localgradient]=ExactMeasurement(GT,posx,posy,yaw)
            %Return the updraft at specified location
            %func=@(posx,posy)GT.strength*exp(-((GT.thermalx-posx)^2+(GT.thermaly-posy)^2)/GT.radius^2);
            %z=func(posx,posy);
            z=GT.strength*exp(-((GT.thermalx-posx)^2+(GT.thermaly-posy)^2)/GT.radius^2);
            %delta=0.1;
            %localgradient(1,1) = (func(posx+delta,posy)-func(posx-delta,posy))/(2*delta);
            %localgradient(2,1) = (func(posx,posy+delta)-func(posx,posy-delta))/(2*delta);
            %Note that these gradient equations assume x-axis north,
            %y-axis east, yaw_corr north = 0deg and east = 90deg
            yaw_corr = -yaw+0.5*pi();
            r = sqrt((GT.thermalx-posx)^2+(GT.thermaly-posy)^2);
            sinAngle = (cos(yaw_corr)*(GT.thermalx-posx) - sin(yaw_corr)*(GT.thermaly-posy)) / r;
            localgradient = -2.0 * r * z / GT.radius^2 * sinAngle; 
        end
        function h=Display(GT,axis)  
            %Unused
            i=1;
            %reven=sqrt(-s(2)^2*log(sinkrate/s(1)));
            for th=0:0.1:2*pi
                xc(i)=GT.radius*cos(th)+GT.thermalx;
                yc(i)=GT.radius*sin(th)+GT.thermaly;
                i=i+1;

            end
            xt=-2*GT.radius:2*GT.radius;
            for i=1:length(xt)
                f(i) = GT.ExactMeasurement(xt(i),0);
            end

            plot(axis,GT.thermalx,GT.thermaly,'ro');
            plot(axis,xc,yc,'r-');
            plot(axis,xt+GT.thermalx,f+GT.thermaly,'r--');
            plot(axis,xt+GT.thermalx,ones(size(xt))*GT.thermaly,'r-');
        end
    end
    
end

