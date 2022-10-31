%% 飞车不同路线soc计算,能量的耗费情况  想要精确的计算必须要知道车具体工况下的具体参数
clc
clear
close all
load('path1.mat')
load('path_new.mat')
%path_di=path1(1:681,1:3);
path_di=path_new(1:670,1:3);
U=100;
%open_list=[pose_start(1:3),initial_yaw,initial_cost,0,initial_cost,pose_start(1:3),initial_yaw]; 
soc_di(1)=0.98  ;
C_total= 3000000 ;

%地面行驶
for i=1:1:(length(path_di)-1)
    if path_di(i)<path_di(i+1)
        distance_di_pa(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
    else  distance_di_pa(i)=0;
    end
end
for i=1:1:(length(path_di)-1)
    if path_di(i)>path_di(i+1)
        distance_di_jiang(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
    else  distance_di_jiang(i)=0;
    end
end
for i=1:1:(length(path_di)-1)
    if path_di(i)==path_di(i+1)
        distance_di_ping(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
    else  distance_di_ping(i)=0;
    end
end
t_di_pa=distance_di_pa/2;
t_di_ping=distance_di_ping/3;
t_di_jiang=distance_di_jiang/4;
energy_di_pa=t_di_pa*460;

energy_di_ping=t_di_pa*405;
energy_di_jiang=t_di_jiang*200;
energy_di=energy_di_pa+energy_di_ping+energy_di_jiang;
energy_di=energy_di';
for i=1:1:(length(path_di)-1)
soc_di(i+1)=soc_di(i)-(energy_di(i)/C_total);
end
% figure
% plot(soc_di,'b','LineWidth',1.2)
% xlabel('time/(s)'), ylabel('soc');




%空中飞行
%681之后是飞行
path_air=path1(671:1252,1:3);
soc_air(1)=soc_di(length(soc_di));
for i=1:1:(length(path_air)-1)
    if path_air(i)<path_air(i+1)
        distance_air_pa(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
    else  distance_air_pa(i)=0;
    end
end
for i=1:1:(length(path_air)-1)
    if path_air(i)>path_air(i+1)
        distance_air_jiang(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
    else  distance_air_jiang(i)=0;
    end
end
for i=1:1:(length(path_air)-1)
    if path_air(i)==path_air(i+1)
        distance_air_ping(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
    else  distance_air_ping(i)=0;
    end
end
t_air_pa=distance_air_pa/1;
t_air_ping=distance_air_ping/5;
t_air_jiang=distance_air_jiang/1.3;
energy_air_pa=t_air_pa*1530;

energy_air_ping=t_air_pa*1000;
energy_air_jiang=t_air_jiang*1264;
energy_air=energy_air_pa+energy_air_ping+energy_air_jiang;
energy_air=energy_air';
for i=1:1:(length(path_air)-1)
soc_air(i+1)=soc_air(i)-(energy_air(i)/C_total);
end
soc=[soc_di,soc_air];
figure

plot(soc,'r-','LineWidth',3.0)
ylabel('SOC','Fontname','Times New Roman','FontSize',17);
xlabel('Step','Fontname','Times New Roman','FontSize',17);
%xlim([0,1283]);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ax.GridColor = [0 .5 .5];
ax.GridLineStyle = '--';
ax.GridAlpha = 0.9;
set(gcf,'Position',[100 100 1100 300]);

% %%
% %%  新路线
% soc_di1(1)=0.98;
% path_di=path_new(1:670,1:3);
% for i=1:1:(length(path_di)-1)
%     if path_di(i)<path_di(i+1)
%         distance_di_pa(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
%     else  distance_di_pa(i)=0;
%     end
% end
% for i=1:1:(length(path_di)-1)
%     if path_di(i)>path_di(i+1)
%         distance_di_jiang(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
%     else  distance_di_jiang(i)=0;
%     end
% end
% for i=1:1:(length(path_di)-1)
%     if path_di(i)==path_di(i+1)
%         distance_di_ping(i)=sqrt((path_di(i,1)-path_di(i+1,1))^2+(path_di(i,2)-path_di(i+1,2))^2+(path_di(i,2)-path_di(i+1,2))^2);
%     else  distance_di_ping(i)=0;
%     end
% end
% t_di_pa=distance_di_pa/2;
% t_di_ping=distance_di_ping/3;
% t_di_jiang=distance_di_jiang/4;
% energy_di_pa=t_di_pa*460;
% 
% energy_di_ping=t_di_pa*405;
% energy_di_jiang=t_di_jiang*200;
% energy_di=energy_di_pa+energy_di_ping+energy_di_jiang;
% energy_di=energy_di';
% for i=1:1:(length(path_di)-1)
% soc_di1(i+1)=soc_di1(i)-(energy_di(i)/C_total);
% end
% % figure
% % plot(soc_di,'b','LineWidth',1.2)
% % xlabel('time/(s)'), ylabel('soc');
% 
% 
% 
% 
% %空中飞行
% %681之后是飞行
% path_air=path_new(671:1251,1:3);
% soc_air1(1)=soc_di1(length(soc_di));
% for i=1:1:(length(path_air)-1)
%     if path_air(i)<path_air(i+1)
%         distance_air_pa(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
%     else  distance_air_pa(i)=0;
%     end
% end
% for i=1:1:(length(path_air)-1)
%     if path_air(i)>path_air(i+1)
%         distance_air_jiang(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
%     else  distance_air_jiang(i)=0;
%     end
% end
% for i=1:1:(length(path_air)-1)
%     if path_air(i)==path_air(i+1)
%         distance_air_ping(i)=sqrt((path_air(i,1)-path_air(i+1,1))^2+(path_air(i,2)-path_air(i+1,2))^2+(path_air(i,2)-path_air(i+1,2))^2);
%     else  distance_air_ping(i)=0;
%     end
% end
% t_air_pa=distance_air_pa/1;
% t_air_ping=distance_air_ping/5;
% t_air_jiang=distance_air_jiang/1.3;
% energy_air_pa=t_air_pa*1530;
% 
% energy_air_ping=t_air_pa*1000;
% energy_air_jiang=t_air_jiang*1264;
% energy_air=energy_air_pa+energy_air_ping+energy_air_jiang;
% energy_air=energy_air';
% for i=1:1:(length(path_air)-1)
% soc_air1(i+1)=soc_air1(i)-(energy_air(i)/C_total);
% end
% soc1=[soc_di1,soc_air1];
% hold on
% plot(soc1,'color',[16 137 148]/255,'LineWidth',2.0)
% ylabel('SOC','Fontname','Times New Roman','FontSize',17);
% xlabel('Step','Fontname','Times New Roman','FontSize',17);
% %xlim([0,1283]);
% ax = gca;
% ax.XGrid = 'off';
% ax.YGrid = 'on';
% ax.GridColor = [0 .5 .5];
% ax.GridLineStyle = '--';
% ax.GridAlpha = 0.9;
% set(gcf,'Position',[100 100 1100 300]);


