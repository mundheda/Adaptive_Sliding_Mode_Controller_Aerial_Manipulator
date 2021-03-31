function [xdesired,u] = pid_position(m,x,xdot,xint,traj_x,traj_y,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 % Controller gains, tuned by hand and intuition.
 Kd = 1.714;
 Kp = 2.857;
 Ki = 1.142;
 
 T = [1, t, t^2, t^3, t^4, t^5, t^6, t^7, t^8, t^9];
 T_dot = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4, 6*t^5, 7*t^6, 8*t^7, 9*t^8];
 T_double_dot = [0, 0, 2, 6*t, 12*t^2, 20*t^3, 30*t^4, 42*t^5, 56*t^6, 72*t^7];
 
 x_d = T*traj_x;
 x_d_dot = T_dot*traj_x;
 x_double_dot = T_double_dot*traj_x;
 y_d = T*traj_y;
 y_d_dot = T_dot*traj_y;
 y_double_dot = T_double_dot*traj_y;
 
 xdesired = [x_d;y_d];
 xdesireddot = [x_d_dot;y_d_dot];
 xdesireddoubledot = [x_double_dot;y_double_dot];
 
 % Compute required force
 u = m*(xdesireddoubledot-Ki*xint-Kd*(xdot-xdesireddot)-Kp*(x-xdesired));
end

