clc
clear
close all
% 2018.4.20 Grasping force control demo

%% load datafile

% For Ubuntu data path
path1 = '/home/gx/grasping/src/Grasping_control/record/';
name_prefix = 2002;
% For Windows data path
path2 = 'Grasping_data\';

data = load([path1,num2str(name_prefix),'_grasp.txt']); 
time  = data(:,1);
ref_force = data(:,2);
real_force = data(:,3);
% smooth
span1 = 0.05;
method = 'loess';
real_force_s = smooth(real_force,span1,method);

%% plot figure
figure
plot(time,ref_force,'LineWidth',2); hold on;grid on;
plot(time, real_force_s,'LineWidth',2);

axis([-inf,inf,0,2]);
L1 = legend('Ref force','Real force');
set(gca, 'Fontname', 'Times New Roman', 'Fontsize', 8);   
xlabel('Time (s)','Fontname', 'Times New Roman','FontSize',8);
ylabel('Contact force (N)','Fontname', 'Times New Roman','FontSize',8);
set(gcf,'Position',[100 100 400 400/5*4],'color','w');
% print('grasping','-dpdf','-r300'); %print pdf


