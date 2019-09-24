clc
clear
close  all


% load('jnt.mat');


% kp = 100;
% ki = 10;
% kd = 20;
%% Filter design
fc = 2;
fs = 500;


[b,a] = butter(1,fc/(fs/2));
% freqz(b,a);
TT = tf(b,a,1/fs);
rate_limit = 0.4;
% sim('joint_vel_filter.slx');
