stepHeight = 4;
stepLength = 4;
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
%updateBot(bot);
plot_body(bot)
bot.waist       =  bot.pose(1:3, :, :);  % update bot coordinates
bot.rightLeg    =  bot.pose(4:11, :, :);
bot.leftLeg     =  bot.pose(12:19, :, :);
bot.torso       =  bot.pose(20:25, :, :);
bot.rightArm    =  bot.pose(26:28, :, :);
bot.leftArm     =  bot.pose(29:31, :, :);

% step 1 (start from init)
Rfoot_init = bot.rightLeg(3,:);
Lfoot_init = bot.leftLeg(3,:);
centerPelvis_init = bot.waist(2,:);
for n = [0:10]
    
    
    for t = linspace(0, stepLength, 15)
        x = t;
        y = 0;
        z = subs(fn);
        
        syms kneerx kneerz kneelx kneelz

        
        avg = (bot.rightLeg(3,1,1)+bot.rightLeg(3,1,1))/2;
        
        
        waistAngle = 0;
        %centerPelvis = centerPelvis_init + [avg sin((avg)/(stepLength/2)*2*pi) 0 1];
        centerPelvis = centerPelvis_init + [avg 0 0 1];
        heightWaist = centerPelvis(3);
        
        
        if mod(n,2) == 0
            bot.rightLeg(3, :) = Rfoot_init + [x   y z 1];
        else
            bot.leftLeg(3, :) = Lfoot_init + [x y z 1];
        end
        
        %centrePelvis
        bot.pose = bot.findBody(centerPelvis, waistAngle);
        bot.rightLeg(1,[1,3]);
        hipr = bot.pose(4,[1,3]);
        footr = bot.pose(6,[1,3]);
        % bot.leftLeg     =  bot.pose(12:19, :, :);
        hipl = bot.pose(12,[1,3]);
        footl = bot.pose(14,[1,3]);
        %eqn1 = (kneerx-hipr(1,1))^2+(kneerz-hipr(1,2))^2 == 15^2;
        %eqn2 = (kneerx-footr(1,1))^2+(kneerz-footr(1,2))^2 == 15^2;
        %eqn3 = kneerx>hipr(1,1);
        
        eqn4 = (kneelx-hipl(1,1))^2+(kneelz-hipl(1,2))^2 == 15^2;
        eqn5 = (kneelx-footl(1,1))^2+(kneelz-footl(1,2))^2 == 15^2;
        eqn6 = kneelx>hipl(1,1);
        
        [knlx,knlz] = solve(eqn4,eqn5,eqn6);
        %[knrx,knrz] = solve(eqn1,eqn2,eqn3);
        
        rotation_angle = 0;
        a = [bot.pose(4,:)]
        u = bot.pose(4,:)- [bot.pose(4,[1,2]), 3,2];
        v = bot.pose(4,:) - bot.pose(6,:);
        Cosalpha = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
        alpha = real(acos(Cosalpha));
        dist_hf = norm(bot.pose(4,:)-bot.pose(6,:));
        c = acos((450-dist_hf*dist_hf)/450);
        theta = (pi-c)/2
        phi = pi/2 - alpha -theta;
        %th = T(bot.pose(4,:))*Rz(pi)
        th = T(a)*Rz(rotation_angle)*Ry(phi)*T([15 0 0])
        %th = a*Rz(rotation_angle)*Ry(-phi)*T([15 0 0])
        
        bot.pose(5,:) = th(:,4);
       
        %rz=Rz(rotation_angle);
        
        %bot.pose(5,:) = [knrx(1,1), -10, knrz(1,1),0];
        bot.pose(13,:) = [knlx(1,1), 10, knlz(1,1),0];    
        %
        %     updateBot(bot)
        %     bot.rightLeg = bot.findLeg("right")
        %     bot.leftLeg = bot.findLeg("left")
        %     bot.pose = pose;
        %     bot.pose(6,2,1) = 10;
        %     bot.pose(6,:,:)
        hold on
        rightleg = bot.rightLeg;
        waistbot = bot.pose(1,:);
        disthiptoknee = [norm(bot.pose(1,:)-bot.pose(5,:)), norm(bot.pose(12,:)-bot.pose(13,:))]  %knee is rightLeg(2,:)
        distkneefoot = [norm(bot.pose(5,:)-bot.pose(6,:)), norm(bot.pose(13,:)-bot.pose(14,:))]
        plot_body(bot);
        %view(90,0)
        pause(0.0075)
        
        
    end
     Rfoot_init = bot.rightLeg(3, :);
     Lfoot_init = bot.leftLeg(3, :);  
     
end
%plot_body(bot)

% function a = updateBot(bot)    
%     bot.waist       =  bot.pose(1:3, :, :);  % update bot coordinates
%     bot.rightLeg    =  bot.pose(4:11, :, :);
%     bot.torso       =  bot.pose(20:25, :, :);
%     bot.rightArm    =  bot.pose(26:28, :, :);
%     bot.leftArm     =  bot.pose(29:31, :, :);
% end
