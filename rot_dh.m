function R_dh = rot_dh(theta,alpha)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

R_dh = [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha);
     sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha);
     0, sin(alpha), cos(alpha)];
end

