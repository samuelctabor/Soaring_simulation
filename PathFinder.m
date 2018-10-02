classdef PathFinder
    %PathFinder Class to implement thermal pathfinding
    %   This will use the A* (possibly D* in future) algorithm to plot an
    %   optimal route to a desired point given a map of estimated thermal
    %   updraft locations and strengths along the way
    
    properties
        map;
        printfnct;
    end
    
    methods
        function pf = PathFinder(map, func)
            pf.map = map;
            pf.printfnct = func;
        end
        function path_3d = plan(this, pos, goal)
            % using mostly code from Thermal_path.m
            start_pos = pos;
            goal_pos = goal;
            
            
            [thermals_pos, thermals_strength] = PathFinder.map_to_arrays(this.map);

            LoDmax=2;
            cruise_spd=5;
            
            this.print('Forming graph');
            [G, nodes_xyz] = Graph_from_Thermals(thermals_pos,thermals_strength,100,0,start_pos,goal_pos, LoDmax, cruise_spd, 5);
            
            for i=1:size(G,1)
                for j=1:size(G,2)
                    if G(i,j)~=0
                        plot3([nodes_xyz(i,1),nodes_xyz(j,1)],[nodes_xyz(i,2),nodes_xyz(j,2)],[nodes_xyz(i,3),nodes_xyz(j,3)],'b-');
                    end
                end
            end
            
            this.print('Solving path');
            [path, score] = A_star_alg(G,1,2,nodes_xyz);
            
            for i=1:length(path)-1
                plot3(  [nodes_xyz(path(i),1), nodes_xyz(path(i+1),1)], ...
                    [nodes_xyz(path(i),2), nodes_xyz(path(i+1),2)], ...
                    [nodes_xyz(path(i),3), nodes_xyz(path(i+1),3)],'r.-');
            end
            
            path_3d = nodes_xyz(path,:);
            
        end
        
        function print(this, message)
            this.printfnct(['PF: ', message]);
        end
    end
    methods (Static)
        function [positions,strengths]=map_to_arrays(map)
            idx = (map.datapoints(:,4)~=0);
            positions=[map.datapoints(idx,1:2),zeros([length(find(idx)),1])];
            strengths=[map.datapoints(idx,3)];
        end
    end
    
end

