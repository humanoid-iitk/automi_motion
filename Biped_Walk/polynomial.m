function a = polynomial(stepHeight, stepLength)
%POLYNOMIAL Provides foot polynomial with provided specs
%   Detailed explanation goes here
syms x
a = (27*stepHeight)/(4*stepLength^3)*x^3 + -(27*stepHeight)/(2*stepLength^2)*x^2 + (27*stepHeight)/(4*stepLength)*x + 0;
    
end

