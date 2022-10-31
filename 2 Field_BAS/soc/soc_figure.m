%两条路径的SOC对比
clc
clear
close all
load('soc1.mat')
load('soc2.mat')
plot(soc1,'color',[16 137 148]/255,'LineWidth',2.0)
ylabel('SOC','Fontname','Times New Roman','FontSize',17);
xlabel('Step','Fontname','Times New Roman','FontSize',17);
%xlim([0,1283]);
ax = gca;       
ax.XGrid = 'off';
ax.YGrid = 'on';
ax.GridColor = [0 .5 .5];
ax.GridLineStyle = '--';
ax.GridAlpha = 0.9;
set(gcf,'Position',[100 100 1100 700]);
hold on
plot(soc,'r','LineWidth',3.0)
legend('优化前','优化后','location','SouthOutside','FontSize',17);
