clc
clear
close all
% load model
fuzzy_control_model = readfis('fuzzy_control_reasoning.fis');

%inputs
case_num = 4;
switch case_num
    case 1
        theta = 0;
        y = 40;
        x = 20;
    case 2
        theta = 90;
        y = -30;
        x = 10;
    case 3
        theta = -140;
        y = 30;
        x = 40;
    case 4
        theta = -10;
        y = 10;
        x = 50;
    otherwise
        disp("Error!");
        return;
end

%fixed param
L = 2.5;
T = 0.1;
v = 0.5;

%boundary
y_boundary = 0.1;
theta_boundary = 0.01;

%storage for map
locus_x=[];locus_x(1)=x;
locus_y=[];locus_y(1)=y;
locus_theta=[];locus_theta(1)=theta;
locus_u=[];locus_u(1)=0;


%fuzzy controller
index = 0;
while (abs(y) > y_boundary || abs(theta) > theta_boundary)
index = index + 1;
 %infer u
 u = evalfis(fuzzy_control_model, [y theta]);
 
 %degree to rad
 u = deg2rad(u);
 theta_rad = deg2rad(theta);
 
 %update theta,x and y
 theta_current = theta_rad + v * T * tan(u) / L;
 x_current = x + v * T * cos(theta_rad);
 y_current = y + v * T * sin(theta_rad);
%storage
theta = rad2deg(theta_current);
 x = x_current;
 y = y_current;
 locus_x(index + 1)=x;
 locus_y(index + 1)=y;
 locus_theta(index + 1)=theta;
 locus_u(index + 1)=u;
end

t = 0:0.1:0.1*index;

figure

subplot(2,2,1),plot(locus_x,locus_y,'-k'),
title('x-y'),grid on
subplot(2,2,2),plot(t,locus_y,'-k')
title('t-y'),grid on
subplot(2,2,3),plot(t,locus_u,'-k')
title('t-u'),grid on
subplot(2,2,4),plot(t,locus_theta,'-k')
title('t-theta'),grid on


