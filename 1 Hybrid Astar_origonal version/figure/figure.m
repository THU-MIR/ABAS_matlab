clc
clear
close all
%%
heightmap=double(imread('12m.tif'));
heightmap=heightmap(127:2595,107:4668)*0.05;
heightmap=heightmap-min(min(heightmap));
heightmap=imresize(heightmap, 0.1);
[gx,gy]=gradient(heightmap);
figure
surf(heightmap,'EdgeColor','none');
axis equal
set(gca,'xtick',[],'xcolor','k') 
set(gca,'ytick',[],'ycolor','k') 
set(gca,'ztick',[],'zcolor','k') 
start_xy=[1380,360]*0.1;
goal_xy=[1800,2100]*0.1;

hold on
plot3(goal_xy(2),goal_xy(1),heightmap(goal_xy(1),goal_xy(2)),'xg','LineWidth',1.0);
plot3(start_xy(2),start_xy(1),heightmap(start_xy(1),start_xy(2)),'xr','LineWidth',1.0);

start_xy1=[240,2700]*0.1;
goal_xy1=[2180,4160]*0.1;
hold on
plot3(goal_xy1(2),goal_xy1(1),heightmap(goal_xy1(1),goal_xy1(2)),'og','LineWidth',1.0);
plot3(start_xy1(2),start_xy1(1),heightmap(start_xy1(1),start_xy1(2)),'or','LineWidth',1.0);
%低海拔
start_xy2=[510,400]*0.1;
goal_xy2=[100,3220]*0.1;
hold on
plot3(goal_xy2(2),goal_xy2(1),heightmap(goal_xy2(1),goal_xy2(2)),'*g','LineWidth',1.0);
plot3(start_xy2(2),start_xy2(1),heightmap(start_xy2(1),start_xy2(2)),'*r','LineWidth',1.0);


%% 画等高线图
%% 画等高线图
figure
heightmap=double(imread('12m.tif'));
heightmap=heightmap(127:2595,107:4668)*0.05;
heightmap=heightmap-min(min(heightmap));
heightmap=imresize(heightmap, 0.1);
[gx,gy]=gradient(heightmap);
quiver(gx,gy)
axis equal
hold on
contour(heightmap,'LineWidth',1.0)
plot(start_xy(2),start_xy(1),'xr','LineWidth',1.0);
plot(goal_xy(2),goal_xy(1),'xg','LineWidth',1.0);
hold on
plot(start_xy1(2),start_xy1(1),'or','LineWidth',1.0);
plot(goal_xy1(2),goal_xy1(1),'og','LineWidth',1.0);
hold on
plot(start_xy2(2),start_xy2(1),'*r','LineWidth',1.0);
plot(goal_xy2(2),goal_xy2(1),'*g','LineWidth',1.0);
