function a = Rz(theta)
%Rz Rotation Tensor about Z-axis
%   Detailed explanation goes here
    a = [cos(theta) -sin(theta) 0   0;
         sin(theta) cos(theta)  0   0;
         0          0           1   0;
         0          0           0   1];
end