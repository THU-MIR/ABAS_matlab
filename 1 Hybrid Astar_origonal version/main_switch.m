clc
clear
close all
% %% 画图2022.10.12
% heightmap=double(imread('12m.tif'));
% heightmap=heightmap(127:2595,107:4668)*0.05;
% heightmap=heightmap-min(min(heightmap));
% heightmap=imresize(heightmap, 0.1);
% figure
% surf(heightmap,'EdgeColor','none');
% axis equal
% set(gcf,'Position',[100 100 1100 600]); %调节图片的比例
%% 画等高线图
heightmap=double(imread('12m.tif'));
heightmap=heightmap(127:2595,107:4668)*0.05;
heightmap=heightmap-min(min(heightmap));
heightmap=imresize(heightmap, 0.1);
[gx,gy]=gradient(heightmap);
quiver(gx,gy)
axis equal
hold on
contour(heightmap)
%画二维起始点和终点
%可用的起始点组合：

%低高海拔
% start_xy=[240,2700]*0.1;
% goal_xy=[2180,4160]*0.1;
%模态切换点（620.2810）

%低海拔
% start_xy=[510,400]*0.1;
% goal_xy=[100,3220]*0.1;
%模态切换点（360，780）

%高海拔
start_xy=[1380,360]*0.1;
goal_xy=[1800,2100]*0.1;
%模态切换点（1600，480）

%待测试的起始点组合：   （地图的大小：X20:2470;Y20:4550）
% start_xy=[1380,360]*0.1;
% goal_xy=[1800,2100]*0.1;


start=[start_xy,heightmap(start_xy(1),start_xy(2))];
plot(start_xy(2),start_xy(1),'xr');
goal=[goal_xy,heightmap(goal_xy(1),goal_xy(2))];
plot(goal_xy(2),goal_xy(1),'xg');
pause(0.1)%动画的延迟设置
%画二维路径
[path,nodecount] = path_planner(heightmap,start,goal);  
%% 画三维地图
figure
surf(heightmap,'EdgeColor','none');
set(gcf,'Position',[100 100 500 400]); %调节图片的比例
axis equal
hold on
z=zeros(length(path(:,1)),1);
for i=1:length(path(:,1))
    z(i)=heightmap(path(i,1),path(i,2));
end

%% 画起始点和终点
plot3(goal_xy(2),goal_xy(1),heightmap(goal_xy(1),goal_xy(2)),'xg'); %开始点是绿色
plot3(start_xy(2),start_xy(1),heightmap(start_xy(1),start_xy(2)),'xr');
%% 画路径
plot3(path(:,2),path(:,1),path(:,3),'--r','Linewidth',1)
% 画路径上下界限
figure;hold on;grid on
plot(1:length(path(:,1)),path(:,3),'Linewidth',2,'DisplayName','path')
plot(1:length(path(:,1)),z,'DisplayName','ground')
plot(1:length(path(:,1)),z+20,'--','DisplayName','z_{min}')
plot(1:length(path(:,1)),z+30,'-','DisplayName','z_{desired}')
plot(1:length(path(:,1)),z+40,'--','DisplayName','z_{max}')
set(gcf,'Position',[100 100 800 400]); %调节图片的比例
legend;title('Height')