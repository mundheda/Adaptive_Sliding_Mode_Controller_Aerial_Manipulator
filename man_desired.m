function [theta_m,theta_dot_m,theta_ddot_m] = man_desired(traj_theta_1,traj_theta_2,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 theta_m = zeros(2,1);
 theta_dot_m = zeros(2,1);
 theta_ddot_m = zeros(2,1);
 
 T = [1, t, t^2, t^3, t^4, t^5, t^6, t^7, t^8, t^9];
 T_dot = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4, 6*t^5, 7*t^6, 8*t^7, 9*t^8];
 T_double_dot = [0, 0, 2, 6*t, 12*t^2, 20*t^3, 30*t^4, 42*t^5, 56*t^6, 72*t^7];
 
 theta_m(1) = T*traj_theta_1;
 theta_m(2) = T*traj_theta_2;
 theta_dot_m(1) = T_dot*traj_theta_1;
 theta_dot_m(2) = T_dot*traj_theta_2;
 theta_ddot_m(1) = T_double_dot*traj_theta_1;
 theta_ddot_m(2) = T_double_dot*traj_theta_2;
end

