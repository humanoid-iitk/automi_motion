function a = plot_body(body)
%PLOT_BODY Summary of this function goes here
%   Detailed explanation goes here
    waist = body(1:3, :, :)
    rLeg = body(4:11, :, :)
    lLeg = body(12:19, :, :)
    torso = body(20:25, :, :)
    rArm = body(26:28, :, :)
    lArm = body(29:31, :, :)
    
    hold off 
    plot3(waist(:, 1), waist(:, 2), waist(:, 3), 'k-o')
    hold on
    grid on
    plot3(rLeg(:, 1), rLeg(:, 2), rLeg(:, 3), 'k-o')
    plot3(lLeg(:, 1), lLeg(:, 2), lLeg(:, 3), 'k-o')
    plot3(torso(:, 1), torso(:, 2), torso(:, 3), 'k-o')
    plot3(rArm(:, 1), rArm(:, 2), rArm(:, 3), 'k-o')
    plot3(lArm(:, 1), lArm(:, 2), lArm(:, 3), 'k-o')
    pbaspect([2 3 6])
    axis([-10 10 -15 15 0 60])
end

