close all
clc
clear

%inputs
case_num = 1;
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

%tunabel param (stopping criteria)
y_threshold = 0.1;
theta_threshold = 0.01;

% control param
K_y = 4.5;


%storage for map
locus_x=[];locus_x(1)=x;
locus_y=[];locus_y(1)=y;
locus_theta=[];locus_theta(1)=theta;
locus_u=[];locus_u(1)=0;


% P controller
K_P_y = 0.02; %gain of y
K_P_theta=0.01; %gain of theta
index = 0;
while ((abs(y) > y_threshold || abs(theta) > theta_threshold) && index < 10000)
 index = index + 1;
 %compute error of theta and y
 error_theta = 0 - theta;
 error_y = 0 - y;
 
 %compute u
 u = K_P_y*error_y+K_P_theta*error_theta;
 
 %update theta,x and y
 theta_rad = deg2rad(theta);
 theta_curr = theta_rad + v * T * tan(u) / L;
 x_current = x + v * T * cos(theta_rad);
 y_current = y + v * T * sin(theta_rad);
 
 %storage for map
 theta = rad2deg(theta_curr);
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