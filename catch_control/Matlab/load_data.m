clc;
close all;
clear;

name_prefix = 1;

data_path = '/home/dzf/match_copy-master/catch_control/record_servoj_sim';
suffix_sim = '_real_tracking.txt';
suffix_real  = '_servoj_tracking_real_robot.txt';
path = [data_path , num2str(name_prefix), suffix_sim];


result = load(path);
t = result(:,1);
x_sim = result(:,2);
y_sim = result(:,3);
z_sim = result(:,4);
x_real = result(:,5);
y_real = result(:,6);
z_real = result(:,7);

% position = result(:,2);
% velocity = result(:,3);

% plot(t,position,'LineWidth',1.2);hold on;
% plot(t,velocity,  'LineWidth',1.2);
% legend('Ref','real'); grid on; grid minor;
% xlabel('time (s)');
% ylabel('z_position(m)')
% axis([0,20,-inf,inf])

% plot(t,x_sim,'LineWidth',1.2);hold on;
% plot(t,x_real,  'LineWidth',1.2);
figure(1)
plot(t,y_sim,'LineWidth',1.2,'color','r');hold on;
plot(t,y_real,  'LineWidth',1.2,'color','g');hold on;
% figure(2)
% plot( t(1:end-1),diff(t));
% plot(t,z_sim,'LineWidth',1.2);hold on;
% plot(t,z_real,  'LineWidth',1.2,'color','g');
% legend('Ref','real'); grid on; grid minor;
% axis([0,20,-inf,inf])








