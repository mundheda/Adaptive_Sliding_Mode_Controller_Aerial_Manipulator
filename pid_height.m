% Compute system inputs and updated state.
 function [h_d,u] = pid_height(m,g,h,hdot,hint,traj_h,t) 
 % Controller gains, tuned by hand and intuition.
 Kd = 3.4286;
 Kp = 7.1429;
 Ki = 1.4286;
 
 T = [1, t, t^2, t^3, t^4, t^5, t^6, t^7, t^8, t^9];
 T_dot = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4, 6*t^5, 7*t^6, 8*t^7, 9*t^8];
 T_double_dot = [0, 0, 2, 6*t, 12*t^2, 20*t^3, 30*t^4, 42*t^5, 56*t^6, 72*t^7];
 
 h_d = T*traj_h;
 h_d_dot = T_dot*traj_h;
 h_double_dot = T_double_dot*traj_h;
 u = m*((g-h_double_dot) + Ki*hint + Kd*(hdot-h_d_dot) + Kp*(h-h_d));
 end
