classdef ThermalMap < handle
    %ThermalMap Map object to allow storing and sharing of updraft
    %information
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        datapoints = zeros(100,6);
        pointer = 1;
        printfnct;
    end
    
    methods 
        function this=ThermalMap(printfnct)
            this.printfnct=printfnct;
        end
        function add_data_point(this,x,y,str,rad,error)
            time=0;
            addr = this.pointer;
            
            %Check if there is already a point close by
            idx = (this.datapoints(:,3)>1e-6);
            isclose=(((this.datapoints(idx,1)-x).^2+(this.datapoints(idx,2)-y).^2).^0.5)<50.0;
            if sum(isclose)>0
                points=find(isclose);
                if (this.datapoints(points(1),6)>error)
                % This measurement is more accurate, overwrite
                    addr=points(1);
                    this.print(sprintf('Error %f, overwriting',error));
                else
                    % Leave it
                    this.print(sprintf('Error %f, ignoring',error));
                    return;
                end
            else %Not close, append
                this.pointer = this.pointer+1;
                this.print(sprintf('Error %f, appending',error));
            end
            this.datapoints(addr,:)=[x,y,str,rad,time,error];
            
            
        end
        function add_data_point_filter(this,posx,posy,X,P)
            this.add_data_point(posx+X(3),posy+X(4),X(1),X(2),det(P));
        end
        function point=get_data_point(this,pointer)
            point = this.datapoints(pointer,:);
        end
        function map = get_map(this)
            map=this.datapoints;
        end
        function print(this,message)
            this.printfnct(sprintf('Map: %s',message));
        end
    end
    
end

