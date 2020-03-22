function a = Rx(theta)
%Rx Rotation Tensor about X-axis
%   Detailed explanation goes here
    a = [1  0          0            0;
         0  cos(theta) -sin(theta)  0;
         0  sin(theta) cos(theta)   0;
         0  0          0            1];
end