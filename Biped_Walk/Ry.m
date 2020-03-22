function a = Ry(theta)
%RY Rotation Tensor about Y-axis
%   Detailed explanation goes here
    a = [cos(theta)     0   sin(theta)  0;
         0              1   0           0;
         -sin(theta)    0   cos(theta)  0;
         0              0   0           1];
end