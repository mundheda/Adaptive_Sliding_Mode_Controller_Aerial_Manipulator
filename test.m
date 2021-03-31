 clear;
 %Simulation times, in seconds.
 start_time = 0;
 end_time_1 = 20;
 end_time_2 = 30;
 dt = 0.005;
 times_1 =  start_time:dt:end_time_1;
 times_2 =  end_time_1+dt:dt:end_time_2;
 times = start_time:dt:end_time_2;
 %Number of times to be executed
 N = numel(times_1) + numel(times_2);
 %Mass in Kg
 m = 3.5;
 %acceleration due to gravity in 'ms-2'
 g = 9.8;
 % Length of of drone arm in 'm'
 L = 0.3;
 % TH=ct*rho*n^2*(D^4); thrust model
 ct=0.1;
 rho=1.29;
 D = 8*2.54/100;
 %Rotor Force constant
 k = 9.9865e-5;
 % Drag friction coefficient in Body frame along xyz axes
 kd = 0.0001;
 %drag Fd = -cd*0.5*rho*A*v2
 cd = 0.08;
 %b = cd*0.5*rho*0.1;
 %Rotor Moment Constant
 b = 1.6e-2;
 %Inertia Values
 Ixx = 0.3;
 Iyy = 0.3;
 Izz = 0.6;
 I = [Ixx,0,0;
     0,Iyy,0;
     0,0,Izz;];
 m_man = [0.2;0.2];
 I1 = 4.256e-5;
 I2 = 8.321e-5;
 r = [0.05,0.075];
 l = [0.1,0.15];
 % Initial simulation state.
 init = [0;0;0];
 x = zeros(3, 1);
 xdot = zeros(3, 1);
 xint = zeros(3, 1);
 a = zeros(3,1);
 omega = zeros(3,1);
 omegadot = zeros(3,1);
 theta = zeros(3,1);
 thetaint = zeros(3,1);
 thetadot = zeros(3,1);
 thetadesired = zeros(3,1);
 
 theta_m = zeros(2,1);
 theta_dot_m = zeros(2,1);
 theta_ddot_m = zeros(2,1);
 
 input = zeros(4,1);
 plt_a = zeros(1,N);
 plt_b = zeros(1,N);
 plt_c = zeros(1,N);
 plt_x = zeros(1,N);
 plt_y = zeros(1,N);
 plt_z = zeros(1,N);
 plt_theta_1 = zeros(1,N);
 plt_theta_2 = zeros(1,N);
 plt_theta_dot_1 = zeros(1,N);
 plt_theta_dot_2 = zeros(1,N);
 plt_theta_ddot_1 = zeros(1,N);
 plt_theta_ddot_2 = zeros(1,N);
 T_req_1 = zeros(1,N);
 T_req_2 = zeros(1,N);
 T_req_3 = zeros(1,N);
 T_req_4 = zeros(1,N);
 plt_force_x = zeros(1,N);
 plt_force_y = zeros(1,N);
 plt_force_z = zeros(1,N);
 plt_tau_x = zeros(1,N);
 plt_tau_y = zeros(1,N);
 plt_tau_z = zeros(1,N);

 i = 1;
 xdesired = [0;0;0];
 traj_x = traj(start_time,end_time_1,x(1),xdesired(1));
 traj_y = traj(start_time,end_time_1,x(2),xdesired(2));
 traj_h = traj(start_time,end_time_1,x(3),xdesired(3));
 
for t = times_1
 R_i_b = transpose(rot_matrix(theta));
 [T_m,F_m] = man_dynamics(m_man,I1,I2,r,l,theta_m,theta_dot_m,theta_ddot_m,omega,omegadot,a,R_i_b);
 %input from Height Controller
 [h_d,u_h] = pid_height(m,g,x(3),xdot(3),xint(3),traj_h,t);
 total = u_h/k/(cos(theta(1))*cos(theta(2)));
 
 %input from Position controller.
 [xdes,u_pos] = pid_position(m,x(1:2),xdot(1:2),xint(1:2),traj_x,traj_y,t);
 u_pos = -1/(k*total)*u_pos;
 thetadesired(1) = asin(u_pos(1)*sin(theta(3))-u_pos(2)*cos(theta(3)));  
 thetadesired(2) = asin((u_pos(2)*sin(theta(3))+u_pos(1)*cos(theta(3)))/cos(thetadesired(1)));
 
 %input from Attitude Controller
 e = pid_attitude(theta,thetadot,thetaint,thetadesired);
 % Compute total thrust
 p1 = e(1)*Ixx/2/k/L;   
 p2 = e(2)*Iyy/2/k/L;
 p3 = e(3)*Izz/4/b;
 %Get square of angular velocities of rotors
 input(3) = (total/4 - p2 - p3);
 input(1) = (total/4 + p2 - p3);
 input(2) = (total/4 - p1 + p3);
 input(4) = (total/4 + p1 + p3);
 
 % Compute linear and angular accelerations.
 a = acceleration(input, theta, xdot, m, g, k, kd, F_m);
 omegadot = angular_acceleration(input,omega, I, L, b, k, T_m);
 
 %for plotting variables
 T_req_1(i) = k * input(1);
 T_req_2(i) = k * input(2);
 T_req_3(i) = k * input(3);
 T_req_4(i) = k * input(4);
 plt_a(i) = theta(1);
 plt_b(i) = theta(2);
 plt_c(i) = theta(3);
 plt_x(i) = x(1);
 plt_y(i) = x(2);
 plt_z(i) = x(3);
 plt_theta_1(i) = theta_m(1);
 plt_theta_2(i) = theta_m(2);
 plt_theta_dot_1(i) = theta_dot_m(1);
 plt_theta_dot_2(i) = theta_dot_m(2);
 plt_theta_ddot_1(i) = theta_ddot_m(1);
 plt_theta_ddot_2(i) = theta_ddot_m(2);
 plt_force_x(i) = F_m(1);
 plt_force_y(i) = F_m(2);
 plt_force_z(i) = F_m(3);
 plt_tau_x(i) = T_m(1);
 plt_tau_y(i) = T_m(2);
 plt_tau_z(i) = T_m(3);
 %Update omega
 omega = omega + dt * omegadot;
 %Get thetadot from omega
 thetadot = omega2thetadot(theta,omega);
 %Update theta
 theta = theta + dt * thetadot;
 %Update thetaint 
 thetaint = thetaint + dt*(theta-thetadesired);
 xdot = xdot + dt * a;  
 x = x + dt * xdot;
 xint = xint + dt*(x-[xdes(1);xdes(2);h_d]);
 i = i+1;
end

j = i;
traj_x = traj(end_time_1,end_time_2,x(1),xdesired(1));
traj_y = traj(end_time_1,end_time_2,x(2),xdesired(2));
traj_h = traj(end_time_1,end_time_2,x(3),xdesired(3));
traj_theta_1 = traj(end_time_1,end_time_2,0,-30);
traj_theta_2 = traj(end_time_1,end_time_2,0,20);

for t = times_2
 R_i_b = transpose(rot_matrix(theta));
 [theta_m,theta_dot_m,theta_ddot_m] = man_desired(traj_theta_1,traj_theta_2,t);
 [T_m,F_m] = man_dynamics(m_man,I1,I2,r,l,theta_m,theta_dot_m,theta_ddot_m,omega,omegadot,a,R_i_b);
 %input from Height Controller
 [h_d,u_h] = pid_height(m,g,x(3),xdot(3),xint(3),traj_h,t);
 total = u_h/k/(cos(theta(1))*cos(theta(2)));
 
 %input from Position controller.
 [xdes,u_pos] = pid_position(m,x(1:2),xdot(1:2),xint(1:2),traj_x,traj_y,t);
 u_pos = -1/(k*total)*u_pos;
 thetadesired(1) = asin(u_pos(1)*sin(theta(3))-u_pos(2)*cos(theta(3)));  
 thetadesired(2) = asin((u_pos(2)*sin(theta(3))+u_pos(1)*cos(theta(3)))/cos(thetadesired(1)));
 
 %input from Attitude Controller
 e = pid_attitude(theta,thetadot,thetaint,thetadesired);
 % Compute total thrust
 p1 = e(1)*Ixx/2/k/L;   
 p2 = e(2)*Iyy/2/k/L;
 p3 = e(3)*Izz/4/b;
 %Get square of angular velocities of rotors
 input(3) = (total/4 - p2 - p3);
 input(1) = (total/4 + p2 - p3);
 input(2) = (total/4 - p1 + p3);
 input(4) = (total/4 + p1 + p3);
 
 % Compute linear and angular accelerations.
 a = acceleration(input, theta, xdot, m, g, k, kd, F_m);
 omegadot = angular_acceleration(input,omega, I, L, b, k, T_m);
 
 %for plotting variables
 T_req_1(j) = k * input(1);
 T_req_2(j) = k * input(2);
 T_req_3(j) = k * input(3);
 T_req_4(j) = k * input(4);
 plt_a(j) = theta(1);
 plt_b(j) = theta(2);
 plt_c(j) = theta(3);
 plt_x(j) = x(1);
 plt_y(j) = x(2);
 plt_z(j) = x(3);
 plt_theta_1(j) = theta_m(1);
 plt_theta_2(j) = theta_m(2);
 plt_theta_dot_1(j) = theta_dot_m(1);
 plt_theta_dot_2(j) = theta_dot_m(2);
 plt_theta_ddot_1(j) = theta_ddot_m(1);
 plt_theta_ddot_2(j) = theta_ddot_m(2);
 plt_force_x(j) = F_m(1);
 plt_force_y(j) = F_m(2);
 plt_force_z(j) = F_m(3);
 plt_tau_x(j) = T_m(1);
 plt_tau_y(j) = T_m(2);
 plt_tau_z(j) = T_m(3);
 %Update omega
 omega = omega + dt * omegadot;
 %Get thetadot from omega
 thetadot = omega2thetadot(theta,omega);
 %Update theta
 theta = theta + dt * thetadot;
 %Update thetaint 
 thetaint = thetaint + dt*(theta-thetadesired);
 xdot = xdot + dt * a;  
 x = x + dt * xdot;
 xint = xint + dt*(x-[xdes(1);xdes(2);h_d]);
 j = j+1;
end

figure(1);
subplot(3,1,1);
plot(times,rad2deg(plt_a));
xlabel("time ");
ylabel("Roll(in deg)");
subplot(3,1,2);
plot(times,rad2deg(plt_b));
xlabel("time ");
ylabel("Pitch(in deg)");
subplot(3,1,3);
plot(times,rad2deg(plt_c));
xlabel("time ");
ylabel("Yaw(in deg)");

figure(2);
subplot(3,1,1);
plot(times,plt_x);
xlabel("time ");
ylabel("X");
subplot(3,1,2);
plot(times,plt_y);
xlabel("time ");
ylabel("Y");
subplot(3,1,3);
plot(times,plt_z);
xlabel("time ");
ylabel("Z");

figure(3);
subplot(2,1,1);
plot(times,plt_theta_1);
xlabel("time ");
ylabel("angle moved by Joint 1");
subplot(2,1,2);
plot(times,plt_theta_2);
xlabel("time ");
ylabel("angle moved by Joint 2");

figure(4);
subplot(2,1,1);
plot(times,plt_theta_dot_1);
xlabel("time ");
ylabel("omega of Joint 1");
subplot(2,1,2);
plot(times,plt_theta_dot_2);
xlabel("time ");
ylabel("omega of Joint 2");

figure(5);
subplot(2,1,1);
plot(times,plt_theta_ddot_1);
xlabel("time ");
ylabel("alpha of Joint 1");
subplot(2,1,2);
plot(times,plt_theta_ddot_2);
xlabel("time ");
ylabel("alpha of Joint 2");

figure(6);
subplot(3,1,1);
plot(times,plt_force_x);
xlabel("time ");
ylabel("F_x by Manipulator");
subplot(3,1,2);
plot(times,plt_force_y);
xlabel("time ");
ylabel("F_y by Manipulator");
subplot(3,1,3);
plot(times,plt_force_z);
xlabel("time ");
ylabel("F_z by Manipulator");

figure(7);
subplot(3,1,1);
plot(times,plt_tau_x);
xlabel("time ");
ylabel("Tau_x by Manipulator");
subplot(3,1,2);
plot(times,plt_tau_y);
xlabel("time ");
ylabel("Tau_y by Manipulator");
subplot(3,1,3);
plot(times,plt_tau_z);
xlabel("time ");
ylabel("Tau_z by Manipulator");

figure(8);
subplot(2,2,1);
plot(times,T_req_1);
title("Thrust from Motor 1")
xlabel("time");
ylabel("Thrust in 'N'");
subplot(2,2,2);
plot(times,T_req_2);
title("Thrust from Motor 2")
xlabel("time");
ylabel("Thrust in 'N'");
subplot(2,2,3);
plot(times,T_req_3);
title("Thrust from Motor 3")
xlabel("time");
ylabel("Thrust in 'N'");
subplot(2,2,4);
plot(times,T_req_4);
title("Thrust from Motor 4")
xlabel("time");
ylabel("Thrust in 'N'");


