classdef FlightGearThermal
    %GaussianThermal Class to represent a Gaussian thermal updraft
    %distribution.
    %   The Gaussian distribution is used in the literature as a model of
    %   thermal updraft distribution. It is of the form w =
    %   W*exp(-r^2/R^2). The thermal therefore has properties W,R,x and y
    %   representing thermal strength, size, and location.
    
    properties
        thermalx=0;
        thermaly=0;
        strength=5.0;
        radius=10.0;
    end
    
    methods
        function GT=FlightGearThermal(varargin) %(x,y,s,r)
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
            
            r = norm([GT.thermalx-posx,GT.thermaly-posy])/GT.radius;
            
            
            rup=1.3;
            rsink=1.8;
            vupmax=GT.strength;
            vupmin=-0.1*GT.strength;
            globalsink = 0.0; %-0.5;
            
            
            if ((r>=0) && (r<rup))
                z = vupmax*cos(r * pi / (2*rup));
            elseif ((r>=rup)&&(r<(rup+rsink)/2))
                %z(i) = vupmin*    cos((r -           (rup+rsink)/2) *   pi  / (2*    (  (rup+rsink)/2    - rup)));
                z = vupmin*    cos((r -           (rup+rsink)/2) *   pi  / (2*    (  (rup+rsink)/2    - rup)));
                %Vup = v_up_min * cos (( dist_center -  (Rup+Rsink)/2.0 ) * PI / ( 2.0* (  ( Rup+Rsink)/2.0 -Rup )));
            elseif ((r>=(rup+rsink)/2) && (r<=rsink))
                z = (globalsink + vupmin)/2 + (globalsink-vupmin)/2.0 * cos((r-rsink)*pi/((rsink-rup)/2));
            else
                z = globalsink;
            end
            
            
            localgradient = 0;
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
            xt=-4*GT.radius:4*GT.radius;
            for i=1:length(xt)
                f(i) = GT.ExactMeasurement(xt(i),0);
            end
            
            plot(axis,GT.thermalx,GT.thermaly,'bo');
            plot(axis,xc,yc,'b-');
            plot(axis,xt+GT.thermalx,f+GT.thermaly,'b--');
            plot(axis,xt+GT.thermalx,ones(size(xt))*GT.thermaly,'b-');
        end
    end
    
end

