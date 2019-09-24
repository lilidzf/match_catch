clc
close all;
clear;


fc = 0.3;   
fs = 500;

[b,a] = butter(1,fc/(fs/2));
TT = tf(b,a,1/fs);


% PID parameters

x_kp = 100;
x_ki = 0;
x_kd = 0; r_kp = 0;r_ki = 0;r_kd = 0;

% rate limit

x_rate = 1000/250;
r_rate = 500/250;

%  P.FreqUnits='HZ';
% h = bodeplot(TT);
% setoptions(h,'FreqUnits','Hz','PhaseVisible','off');
% 








