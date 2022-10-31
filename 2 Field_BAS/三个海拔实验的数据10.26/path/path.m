%% 对得到的三个实验的路径的整理(包括整合、异常点剔除，衔接，测试，绘图)
clc
clear
close all
%% 画地图
global heightmap 
heightmap=double(imread('12m.tif'));
heightmap=heightmap(127:2595,107:4668)*0.05;    
heightmap=heightmap-min(min(heightmap));
%path=realmax('double');
heightmap=imresize(heightmap,0.1); 
figure
surf(heightmap,'EdgeColor','none');

set(gca,'xtick',[],'xcolor','k') 
set(gca,'ytick',[],'ycolor','k') 
set(gca,'ztick',[],'zcolor','k') 
start_xy=[1380,360]*0.1;
goal_xy=[1800,2100]*0.1;
%% 画起始点
hold on
plot3(goal_xy(2),goal_xy(1),heightmap(goal_xy(1),goal_xy(2)),'*b','LineWidth',1.0);
plot3(start_xy(2),start_xy(1),heightmap(start_xy(1),start_xy(2)),'*r','LineWidth',1.0);

start_xy1=[240,2700]*0.1;
goal_xy1=[2180,4160]*0.1;
hold on
plot3(goal_xy1(2),goal_xy1(1),heightmap(goal_xy1(1),goal_xy1(2)),'*b','LineWidth',1.0);
plot3(start_xy1(2),start_xy1(1),heightmap(start_xy1(1),start_xy1(2)),'*r','LineWidth',1.0);
%低海拔
start_xy2=[510,400]*0.1;
goal_xy2=[100,3220]*0.1;
hold on
plot3(goal_xy2(2),goal_xy2(1),heightmap(goal_xy2(1),goal_xy2(2)),'*b','LineWidth',1.0);
plot3(start_xy2(2),start_xy2(1),heightmap(start_xy2(1),start_xy2(2)),'*r','LineWidth',1.0);
%% 高低海拔
load 'path_gd';
load 'path_gd_1';
load 'path_gd_2';
path_gd_orige=path_gd;%原始的路径
a1=length(path_gd_orige);
path_gd_orige(a1,3)= heightmap (path_gd_orige(a1,1),path_gd_orige(a1,2)); %处理错误值
path_gd_new=[path_gd_1;path_gd_2];%模态更新的路径
a2=length(path_gd_new);
path_gd_new(a2,3)= heightmap (path_gd_new(a2,1),path_gd_new(a2,2)); %处理错误值



%% 低海拔  
load 'path_d';
load 'path_d_1';
load 'path_d_2';
path_d_orige=path_d;%原始的路径
b1=length(path_d_orige);
path_d_orige(b1,3)= heightmap (path_d_orige(b1,1),path_d_orige(b1,2)); %处理错误值
path_d_new=[path_d_1;path_d_2];%模态更新的路径
b2=length(path_d_new);
path_d_new(b2,3)= heightmap (path_d_new(b2,1),path_d_new(b2,2)); %处理错误值

%% 高海拔
load 'path_g';
load 'path_g_1';
load 'path_g_2';
path_g_orige=path_g;%原始的路径
c1=length(path_g_orige);
path_g_orige(c1,3)= heightmap (path_g_orige(c1,1),path_g_orige(c1,2)); %处理错误值
path_g_new=[path_g_1;path_g_2];%模态更新的路径
c2=length(path_g_new);
path_g_new(c2,3)= heightmap (path_g_new(c2,1),path_g_new(c2,2)); %处理错误值

%% 分别在三维地图上绘图

%绘制曲线
%gd
hold on
plot3(path_gd_orige(:,2),path_gd_orige(:,1),path_gd_orige(:,3),'-r','Linewidth',2) 
hold on
plot3(path_gd_new(:,2),path_gd_new(:,1),path_gd_new(:,3),'-g','Linewidth',2) 
%d
hold on
plot3(path_d_orige(:,2),path_d_orige(:,1),path_d_orige(:,3),'-r','Linewidth',2) 
hold on
plot3(path_d_new(:,2),path_d_new(:,1),path_d_new(:,3),'-g','Linewidth',2) 
%g
hold on
plot3(path_g_orige(:,2),path_g_orige(:,1),path_g_orige(:,3),'-r','Linewidth',2) 
hold on
plot3(path_g_new(:,2),path_g_new(:,1),path_g_new(:,3),'-g','Linewidth',2) 

