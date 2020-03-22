stepHeight = 2;
stepLength = 12;
fn = polynomial(stepHeight, stepLength);

params = [  [15, 0];    % definition of Humanoid characteristics
            [15, 0];
            [11, 7];
            [20, 0];
            [25, 0];
            [10, 0]];
bot = Humanoid(params);   % declare humanoid model 'bot'

% step 0 (initialise to straight position)
bot.pose = bot.init();
updateBot();

% step 1 (start from init)
Rfoot_init = bot.rightLeg(3,:);
Lfoot_init = bot.leftLeg(3,:);
centerPelvis_init = bot.waist(2,:);
for t = linspace(0, stepLength/2, 20)
    x = t;
    y = 0;
    z = subs(fn);
    
    bot.rightLeg(3, :) = Rfoot_init + [x y z 1];
    
    
    waistAngle = 0;
    centrePelvis = centerPelvis_init + [x/2 sin((x/2)/(stepLength/2)*2*pi) 0 1];
    centrePelvis(3) = heightWaist;
    
end

function a = updateBot()    
    bot.waist       =  bot.pose(1:3, :, :);  % update bot coordinates
    bot.rightLeg    =  bot.pose(4:11, :, :);
    bot.leftLeg     =  bot.pose(12:19, :, :);
    bot.torso       =  bot.pose(20:25, :, :);
    bot.rightArm    =  bot.pose(26:28, :, :);
    bot.leftArm     =  bot.pose(29:31, :, :);
end
