function [T_m,F_m] = man_dynamics(m,I1,I2,r,l,theta_m,theta_dot_m,theta_ddot_m,omega,omegadot,a,R_i_b)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
R_b_0 = transpose(rot_matrix([0,-90,-90]));
R_1_0 = rot_dh(theta_m(1),0);
R_2_1 = rot_dh(theta_m(2),0);
R_b_1 = transpose(R_1_0)*R_b_0;
R_2_0 = R_1_0*R_2_1;

a_b = R_i_b*a;

I1 = [0,0,0;0,0,0;0,0,I1];
I2 = [0,0,0;0,0,0;0,0,I2];

b1 = transpose(R_1_0);
omega_1 = R_b_1*omega + b1*[0;0;theta_dot_m(1)];
alpha_1 = R_b_1*omegadot + b1*[0;0;theta_ddot_m(1)] + cross(omega_1,b1*[0;0;theta_dot_m(1)]);
ac1 = R_b_1*a_b + cross(alpha_1,[r(1);0;0]) + cross(omega_1,cross(omega_1,[r(1);0;0]));
ae1 = R_b_1*a_b + cross(alpha_1,[l(1);0;0]) + cross(omega_1,cross(omega_1,[l(1);0;0]));

b2 = transpose(R_2_0);
omega_2 = transpose(R_2_1)*omega_1 +  b2*[0;0;sum(theta_dot_m)];
alpha_2 = R_b_1*omegadot + b2*[0;0;sum(theta_ddot_m)] + cross(omega_2,b2*[0;0;sum(theta_dot_m)]);
ac2 = transpose(R_2_1)*ae1 + cross(alpha_2,[r(2);0;0]) + cross(omega_2,cross(omega_2,[r(2);0;0]));

g2 = transpose(R_2_0)*R_b_0*R_i_b*[0;0;9.8];
g1 = transpose(R_1_0)*R_b_0*R_i_b*[0;0;9.8];

f2 = m(2)*ac2 - m(2)*g2;
t2 = -cross(f2,[r(2);0;0]) + I2*alpha_2 + cross(omega_2,I2*omega_2);

f1 = R_2_1*f2 + m(1)*ac1 - m(1)*g1;
t1 = R_2_1*t2 - cross(f1,[r(2);0;0]) + cross(R_2_1*f2,[l(1)-r(1);0;0]) + I1*alpha_1 + cross(omega_1,I1*omega_1);

T_m = transpose(R_b_1)*t1;
F_m = transpose(R_b_1)*f1;

end

