function [planned_path,total_nodecount] = path_planner(heightmap,pose_start,pose_goal)
% flag_arrive=false;
planned_path=[];
total_nodecount=0;

while 1
    % pose = [x, y]
    [partial_path,pose_start,nodecount] = planner_ground(heightmap,pose_start,pose_goal); % ground
    planned_path=[planned_path;partial_path];   % connect paths
    total_nodecount=total_nodecount+nodecount;
    
    plot(partial_path(:,2),partial_path(:,1),'Color',[0 0.9 0],'Linewidth',2,'DisplayName','ground path');pause(0.1)
    plot(partial_path(end,2),partial_path(end,1),'co','LineWidth',2,'MarkerSize',5,'DisplayName','takeoff');pause(0.1)
    
    if isnan(pose_start) % no new start -> reached goal
        break
    end
    
    [partial_path,pose_start,nodecount] = planner_aerial(heightmap,pose_start,pose_goal);    % aerial
    planned_path=[planned_path;partial_path];   % connect paths
    total_nodecount=total_nodecount+nodecount;
    
    plot(partial_path(:,2),partial_path(:,1),'Color',[0 0.5 0.9],'Linewidth',2,'DisplayName','aerial path');pause(0.1)
    
    if isnan(pose_start) % no new start
        break
    end
end