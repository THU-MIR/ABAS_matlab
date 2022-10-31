% Shirine El Zaatari - Autonomous Mobile Robotics - A* path planning algorithm

%UNCOMMENT THIS SECTION in order to generate your own map
 Num_vertical_div = 20;
 Num_horizontal_div = 10;
%[map,goal,initial,grid]=GenerateMap(Num_vertical_div,Num_horizontal_div);  %可以自己设置开始点目标点以及障碍物的位置，可以借鉴



% OR UNCOMMENT THIS SECTION to randomize a load already made map
[grid,map, goal, initial]=DejaMadeMap;

%A* and outputing result as image
path = Astar(map,initial,goal);
VisualizePath(path,grid);


