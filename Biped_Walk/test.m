% stepHeight = 3;
% stepLength = 15;
% for x = linspace(0,stepLength, 200)
%     y = (27*stepHeight)/(4*stepLength^3)*x^3 + -(27*stepHeight)/(2*stepLength^2)*x^2 + (27*stepHeight)/(4*stepLength)*x + 0;
%     hold on 
%     plot(x, y, "k .")
%     axis([0 20 0 5])
%     pbaspect([4 1 1])
% end

% syms a b c d x
% eqn = a*x^3 + b*x^2 + c*x + d == 0;
% f = diff(eqn);
% s = solve(f, x);
% t =  -(b + (b^2 - 3*a*c)^(1/2))/(3*a);
% a*t^3 + b*t^2 + c*t + d

% syms a b c d stepLength stepHeight
% eqns = [a*stepLength^3 + b*stepLength^2 + c*stepLength + d == 0,
%         d == 0,
%         3*a*stepLength^2 + 2*b*stepLength + c == 0,
%         d - (b + (b^2 - 3*a*c)^(1/2))^3/(27*a^2) + (b*(b + (b^2 - 3*a*c)^(1/2))^2)/(9*a^2) - (c*(b + (b^2 - 3*a*c)^(1/2)))/(3*a) - stepHeight == 0];
% [sola, solb, solc, sold] = solve(eqns, [a b c d])

%------------------------------Coefficients--------------------------------
% a = (27*stepHeight)/(4*stepLength^3) 
% b = -(27*stepHeight)/(2*stepLength^2) 
% c = (27*stepHeight)/(4*stepLength) 
% d  = 0
%--------------------------------------------------------------------------

params = [  [15, 0];
            [15, 0];
            [11, 7];
            [20, 0];
            [25, 0];
            [10, 0]];
m = Humanoid(params);
% m.rightLeg(3, :) = [0 0 0 1];
% m.rightLeg(1, :) = [0 10 10 1];
% m.rightLeg = m.findLeg("right");
% plot_leg(m.rightLeg)
m.pose = m.init()
plot_body(m.pose);
