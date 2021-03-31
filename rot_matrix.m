function R = rot_matrix(theta)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
R1 = [1,0,0;
    0,cos(theta(1)),-sin(theta(1));
    0,sin(theta(1)),cos(theta(1));
    ];
R2 = [cos(theta(2)),0,sin(theta(2));
    0,1,0;
    -sin(theta(2)),0,cos(theta(2));
    ];
R3 = [cos(theta(3)),-sin(theta(3)),0;
      sin(theta(3)),cos(theta(3)),0;
      0,0,1;
      ];
R = R3*R2*R1;
end

