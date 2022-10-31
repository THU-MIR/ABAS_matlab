function planned_path=path_reconstruct(node_current,closed_list)

planned_path(1,:)=node_current(1:3);
planned_path(2,:)=node_current(8:10);
node_current(1:7)=[];
n=2;
flag=true;

while flag
    n=n+1;
    row_parent=sum(closed_list(:,1:3)==node_current(1:3),2)==3;
    node_current=closed_list(row_parent,8:10);
    planned_path(n,:)=node_current;
    if sum(node_current==closed_list(1,1:3))==3
        flag=false;
    end
end
planned_path=flipud(planned_path);	% change direction from START to GOAL