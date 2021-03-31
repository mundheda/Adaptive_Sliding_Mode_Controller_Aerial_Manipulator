function omegadot = angular_acceleration(input,omega, I, L, b, k, T_m)
tau = torques(input, L, b, k) + T_m;
omegadot = I\(tau - cross(omega, I * omega));
end

