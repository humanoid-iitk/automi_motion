function point = testPoint()
%TESTPOINT Summary of this function goes here
%   Detailed explanation goes here
    syms t1 t2 lulla1 lulla2 poi1 poi3 
    eqn = Ry(t1)*T([0 0 lulla1 1])*Ry(t2)*[0; 0; lulla2; 1] == [poi1; 0; poi3; 1]
    [solt1, solt2] = solve(eqn, [t1 t2]);
    theta1 = solt1(2);
    point = [lulla1*sin(theta1); 0; lulla1*cos(theta1); 1]
%     norm(point(1:3))
%     norm(point(1:3) - endPoint(1:3))
    
end

