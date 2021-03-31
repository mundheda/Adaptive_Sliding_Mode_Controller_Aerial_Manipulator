function [thetadot] = omega2thetadot(theta,omega)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
M = [1,0,-sin(theta(2));
      0,cos(theta(1)),sin(theta(1))*cos(theta(2));
      0,-sin(theta(1)),cos(theta(2))*cos(theta(1));
      ];
 thetadot = M\omega;
end

