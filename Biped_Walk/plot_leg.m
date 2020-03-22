function a = plot_leg(leg)
%PLOT_LEG Summary of this function goes here
%   Detailed explanation goes here
    plotX = leg(:, 1);
    plotY = leg(:, 2);
    plotZ = leg(:, 3);
    plot3(plotX, plotY, plotZ, 'k- o')
    grid on
    pbaspect([1 1 1])
    axis([-15 15 -15 15 0 30])
end

