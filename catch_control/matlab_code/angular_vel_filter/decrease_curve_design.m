clc
close all
clear

x = 0:0.001:1;
y = (1-x.^2).*exp(-x.^2);

figure
plot(x,y);


