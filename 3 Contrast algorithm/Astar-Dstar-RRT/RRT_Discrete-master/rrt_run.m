%%离散动作的rrt

clc
clear
% tic
environment = load('environment.mat');
x = 15;
source = [1,1];
goal = [4,4];
% source = [1,int32(147/x)];% 设置成int型的数值运算速度比较快
% goal = [int32(244/x),3];

%set(gcf,'Position',[100 100 1600 800]);
[edges,vertices] = rrt_implement(environment.empty_world, source, goal, 1);
% toc


%% 0627日工作记录
%搜索76个点值时消耗的时间为15.167秒

%天牛算法的时间为8.192秒（迭代次数为每个点数为30次）  7.626秒（迭代次数为10次）



%[edges,vertices] = rrt_start_implement(environment.empty_world, source, goal, 3, 3);