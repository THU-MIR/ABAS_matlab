function [partial_path,new_start,nodecount] = planner_aerial(heightmap,pose_start,pose_goal)
% yaw: global frame
global err
err=0;
dist_min=20;    % TODO: semantic map
dist_desired=dist_min+10;
dist_max=dist_desired+10;
height_min=heightmap+dist_min;
height_desired=heightmap+dist_desired;
height_max=heightmap+dist_max;
pose_goal(3)=pose_goal(3)+dist_desired; % !!TEMPORARY!!
switch length(pose_start)
    case 3
        % no initial yaw
        initial_yaw=coord2yaw(pose_goal(1)-pose_start(1),pose_goal(2)-pose_start(2));
    otherwise
        initial_yaw=pose_start(4);
end
[gx,gy]=gradient(heightmap);
initial_cost=abs(pose_start(1)-pose_goal(1))+abs(pose_start(2)-pose_goal(2))+abs(pose_start(3)-pose_goal(3));
% list: [x, y, z, yaw, F, G, H, x_parent, y_parent, z_parent, yaw_parent]
% index: 1  2  3   4   5  6  7     8         9         10         11
open_list=[pose_start(1:3),initial_yaw,initial_cost,0,initial_cost,pose_start(1:3),initial_yaw];   % put the start node into open list
closed_list=[]; % set closed list

new_start=NaN;
% flag_takeoff=false;

while ~isempty(open_list) %&& ~flag_takeoff
    [row,~]=find(open_list(:,5)==min(open_list(:,5)));  % find the node with the lowest cost
    if length(row)>=2   % if there is multiple nodes with the lowest cost
        row=row(open_list(row,6)==min(open_list(row,6)));  % select the node with lower G cost
        row=row(end);
    end
    node_current=open_list(row(1),:);    % set the node with the lowest cost as current node
    
    switch node_current(7)
        case 0
            break;
    end

    open_list(row,:)=[];    % remove current node from open list
    closed_list(end+1,:)=node_current;  % put current node into closed list    
    
    [node_successor]=successor_gen_2D(node_current,pose_goal,gx,gy,height_min,height_desired,height_max); % generate successor nodes

    [open_list,closed_list]=successor_check_2D(node_successor,open_list,closed_list);  % check if any successor in list
    
%     visulize_search(node_successor,[0 0.9 0])   % visulize the searching process

%     flag_takeoff = takeoff_judge(closed_list,open_list);
end

% if flag_takeoff
%     [~,r]=min(closed_list(:,7));    % pick the node with the lowest H
%     new_start=closed_list(r,:);     % set as the new start
%     partial_path=path_reconstruct(new_start,closed_list);   % reconstruct the path
% %     partial_path(end,:)=[];
%     new_start=closed_list(r,1:4);
% else
    partial_path=path_reconstruct(node_current,closed_list);   % reconstruct the path
% end

nodecount=length(closed_list(:,1))+length(open_list(:,1));



function [open_list,closed_list]=successor_check_2D(node_successor,open_list,closed_list)

len=length(node_successor(:,1));
i=1;
while len>=i
    index_open=sum(open_list(:,1:2)==node_successor(i,1:2),2);  % !!ONLY COMPARE X,Y!!
    row_open=find(index_open==2,1);
    if ~isempty(row_open)   % if in OPEN LIST
        if (node_successor(i,5)<open_list(row_open,5))
            open_list(row_open,5:11)=node_successor(i,5:11);
            continue;
        end
    else    % if not in OPEN LIST
        index_closed=sum(closed_list(:,1:2)==node_successor(i,1:2),2);
        row_closed=find(index_closed==2,1);

        if isempty(row_closed)    % if not in CLOSED LIST
            open_list(end+1,:)=node_successor(i,:);
        end
    end
    i=i+1;
end

function [node_successor]=successor_gen_2D(node_current,pose_goal,gx,gy,height_min,height_desired,height_max)

[imin,imax,jmin,jmax,successor_count]=boundary_check(node_current,gx);   % get boundary
neighbor_count=1;
x_curr=node_current(1); y_curr=node_current(2); z_curr=node_current(3); yaw_curr=node_current(4);


node_successor=zeros(successor_count,11);   % create successor list
% node_successor(:,5:7)=Inf;  % set total & energy cost to inf
for i=imin:imax
    for j=jmin:jmax
        if i==0&&j==0
            continue;   % skip central node
        end
        
        x_next=x_curr+i; y_next=y_curr+j; z_next=z_curr; yaw_next=neighbor2yaw(i,j);
        z_min=height_min(x_next,y_next); z_desired=height_desired(x_next,y_next); z_max=height_max(x_next,y_next);

        accessible=accessibility_check([x_curr, y_curr, z_curr, yaw_curr],[x_next, y_next, z_next, yaw_next],gx,gy);
        z_next=z_filter(z_next,z_min,z_desired,z_max);
        if accessible
            node_successor(neighbor_count,1:4)=[x_next, y_next, z_next, yaw_next];
            
            [F,G,H]=total_cost_2D(node_current,node_successor(neighbor_count,1:4),pose_goal,gx,gy);  % compute costs
            node_successor(neighbor_count,5:7)=[F,G,H];
            node_successor(neighbor_count,8:11)=node_current(1:4);
            neighbor_count=neighbor_count+1;
            
        else
            node_successor(end,:)=[];   % remove unavailable node
        end
    end
end

% DIRECTION
% --------------
%     x  y  yaw
% --------------
% 1   1  0    0
% 2   1  1   45
% 3   0  1   90
% 4  -1  1  135
% 5  -1  0  180
% 6   1 -1  -45
% 7   0 -1  -90
% 8  -1 -1 -135
% --------------


function [J,G,H]=total_cost_2D(node_current,node_next,node_goal,gx,gy)

H=H_cost(node_next,node_goal);  % A* heuristic cost
G=G_cost(node_current,node_next,gx,gy);   % movement cost
J=H+G;
% fprintf('%.2f, %.2f\n',H,G)

function G=G_cost(node_current,node_next,gx,gy)

G_curr=node_current(6);

x_curr=node_current(1); y_curr=node_current(2); z_curr=node_current(3);
x_next=node_next(1); y_next=node_next(2); z_next=node_next(3);
gx_curr=gx(x_curr,y_curr); gy_curr=gy(x_curr,y_curr);
gx_next=gx(x_next,y_next); gy_next=gy(x_next,y_next);

switch x_next-x_curr
    case 0
        G_next=1;
%         var_grad=abs(gy_next-gy_curr);
    otherwise
        switch y_next-y_curr
            case 0
                G_next=1;
%                 var_grad=abs(gx_next-gx_curr);
            otherwise
                G_next=1.41421356;
%                 var_grad=abs(gx_next+gy_next-gx_curr-gy_curr);
        end
end

% var_grad=abs(atan(gx_next+gy_next-gx_curr-gy_curr));
var_grad=abs(gx_next+gy_next-gx_curr-gy_curr);
k_grad=0.6;

G_z=abs(z_next-z_curr);

% k_climb=1.2;
% k_descend=1.0;
% if z_next>z_curr
%     G_z=(z_next-z_curr)*k_climb;	% climb
% else
%     G_z=(z_curr-z_next)*k_descend;  % descend
% end
G_z=0;
G=G_curr+G_next+G_z+var_grad*k_grad;


function H=H_cost(node_next,node_goal)

x_next=node_next(1); y_next=node_next(2); z_next=node_next(3);
x_goal=node_goal(1); y_goal=node_goal(2); z_goal=node_goal(3);

% H_z=abs(z_next-z_goal);

k_climb=1.2;
k_descend=1.0;
if z_goal>z_next
    H_z=(z_goal-z_next)*k_climb;	% climb
else
    H_z=(z_next-z_goal)*k_descend;  % descend
end
H_z=0;
H=abs(x_next-x_goal)+abs(y_next-y_goal)+H_z; % Manhattan dist
% H=sqrt((x_next-x_goal)^2+(y_next-y_goal)^2+(z_next-z_goal)^2);  % Euclidian dist


function [imin,imax,jmin,jmax,successor_count]=boundary_check(node_current,map)
boundary=size(map);

switch node_current(1)
    case 1
        imin=0;imax=1;
    case boundary(1)
        imin=-1;imax=0;
    otherwise
        imin=-1;imax=1;
end

switch node_current(2)
    case 1
        jmin=0;jmax=1;
    case boundary(2)
        jmin=-1;jmax=0;
    otherwise
        jmin=-1;jmax=1;
end
successor_count=(imax-imin+1)*(jmax-jmin+1)-1;


function yaw=neighbor2yaw(x,y)
switch x
    case 0
        switch y
            case 1
                yaw=90;
            case -1
                yaw=-90;
        end
    case 1
        switch y
            case 0
                yaw=0;
            case 1
                yaw=45;
            case -1
                yaw=-45;
        end
    case -1
        switch y
            case 0
                yaw=180;
            case 1
                yaw=135;
            case -1
                yaw=-135;
        end
end


function accessible=accessibility_check(curr_pos,next_pos,gx,gy)
x_curr=curr_pos(1); y_curr=curr_pos(2);  yaw_curr=curr_pos(4);
x_next=next_pos(1); y_next=next_pos(2);  yaw_next=next_pos(4);
% z_curr=curr_pos(3);z_next=next_pos(3);
if abs(yaw_next-yaw_curr)>45
    accessible=false;
else
    gx_next=gx(x_next,y_next);
    gy_next=gy(x_next,y_next);
    var_gx=gx_next-gx(x_curr,y_curr);
    var_gy=gy_next-gy(x_curr,y_curr);
    
    max_climb=5.0;
    max_descend=-5.2;
    
%     grad_curr=[gx(x_curr,y_curr),gy(x_curr,y_curr)];
%     grad_next=[gx(x_next,y_next),gy(x_next,y_next)];
    
% +45deg: (gx_next+gy_next)/sqrt(2)
% -45deg: (gx_next-gy_next)/sqrt(2)
    
    switch yaw_next
        case 90
            traversable=var_gy<max_climb&&var_gy>max_descend;
        case -90
            traversable=var_gy<max_climb&&var_gy>max_descend;
        case 0
            traversable=var_gx<max_climb&&var_gx>max_descend;
        case 180
            traversable=var_gx<max_climb&&var_gx>max_descend;
        case 45
            traversable=(var_gx+var_gy)/sqrt(2)<max_climb&&(var_gx+var_gy)/sqrt(2)>max_descend;
        case -135
            traversable=(var_gx+var_gy)/sqrt(2)<max_climb&&(var_gx+var_gy)/sqrt(2)>max_descend;
        case 135
            traversable=(var_gx-var_gy)/sqrt(2)<max_climb&&(var_gx-var_gy)/sqrt(2)>max_descend;
        case -45
            traversable=(var_gx-var_gy)/sqrt(2)<max_climb&&(var_gx-var_gy)/sqrt(2)>max_descend;
    end
%     accessible=traversable&&abs(z_curr-z_next)<5;
    accessible=traversable;
%     fprintf('tr: %d\n',traversable);
%     if ~traversable
%         plot(y_next,x_next,'.','Color',[0.5 0 0.9]);
%         pause(0.01)
%     end
    % yellow -> unstandable; purple -> untraversable; red -> both
end

function yaw=coord2yaw(x,y)
yaw=atan2d(y,x);
if yaw>=-22.5 && yaw<22.5
    yaw=0;
elseif yaw>=22.5 && yaw<67.5
    yaw=45;
elseif yaw>=67.5 && yaw<112.5
    yaw=90;
elseif yaw>=112.5 && yaw<157.5
    yaw=135;
elseif yaw>=157.5 && yaw<=180 || yaw>-157.5 && yaw<=-180
    yaw=180;
elseif yaw>=-67.5 && yaw<-22.5
    yaw=-45;
elseif yaw>=-112.5 && yaw<-67.5
    yaw=-90;
elseif yaw>=-157.5 && yaw<-112.5
    yaw=-135;
end

function z_filtered=z_filter(z_next,z_min,z_desired,z_max)
kp=1.1;
k_iir=0.7;

err=z_desired-z_next;

% z_next = k_iir*z_next + (1-k_iir)*(z_next + err);
z_next = k_iir*z_next + (1-k_iir)*(z_next + kp*err);
if z_next<=z_min
    z_filtered=z_min;
elseif z_next>=z_max
    z_filtered=z_max;
else
    z_filtered=z_next;
end