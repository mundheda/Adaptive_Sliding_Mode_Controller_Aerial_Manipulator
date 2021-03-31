function a = acceleration(inputs, angles, xdot, m, g, k, kd, F_m)
gravity = [0; 0; g];
R = rot_matrix(angles);
T = thrust(inputs, k) + F_m;
Fd = -kd*xdot;
a = gravity + 1/m*R*T + Fd;
end

