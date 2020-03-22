function a = T(point)
%T Translation Tensor
%   Detailed explanation goes here
    a = [1  0   0   point(1);
         0  1   0   point(2);
         0  0   1   point(3);
         0  0   0   1       ];
end