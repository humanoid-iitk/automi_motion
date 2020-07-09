stepHeight = 3;
stepLength = 10;
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
Rknee_init = bot.rightLeg(2,:);
Lknee_init = bot.leftLeg(2,:);
Rfoot_init = bot.rightLeg(3,:);
Lfoot_init = bot.leftLeg(3,:);
centerPelvis_init = bot.waist(2,:);

newtheta = atan(25/stepLength);
newa = sqrt((step)*(step) + 25*25)
newc = acos((450 - newa*newa)/450);
newgamma = (pi-newc)/2;
phifinal = pi - newgamma - newtheta;

newalpha = acos(5/6);
phiinitial = pi/2 - newalpha

phi_interval = (phifinal - phiinitial)/15;

steps = 50;
for n = [1:steps]
    %mini = 100;
    %maxi = 0;
    
    count = 0;
    if mod(n,2)
        bot.pose(13,:) = Lknee_init;
        waistAngle = pi/72;
    else
        bot.pose(5,:) = Rknee_init;
        waistAngle = -pi/72;
    end    
    for t = linspace(0, stepLength, 15)
        x = t;
        y = 0;
        z = subs(fn);
        
        %syms kneerx kneerz kneelx kneelz

        
        avg = (bot.pose(6,1,1)+bot.pose(14,1,1))/2;
        
        
        %waistAngle = 0;
        %centerPelvis = centerPelvis_init + [avg sin((avg)/(stepLength/2)*2*pi) 0 1];
        centerPelvis = centerPelvis_init + [avg 0 0 1];
        step = mod(n,2);
        heightWaist = centerPelvis(3);
        bot.pose(13,:);
        bot.pose = bot.findBody(centerPelvis, waistAngle);
        bot.pose(5,:);
        bot.pose(13,:) = Lknee_init;
        bot.pose(5,:) = Rknee_init;
        %right leg works fine
        if mod(n,2)
            rotation_angle = 0;
            %bot.pose(13,:) = Lknee_init;
            rot = -phiinitial-phi_interval*t
            newth = T(bot.pose(6,:))*Rz(rotation_angle)*Ry(-phiinitial-phi_interval*t)*T([15 0 0]);
            bot.pose(5,:) = newth(:,4);
            
        else
           rotation_angle = 0;
            bot.pose(13,:) = Lknee_init;
            al = bot.pose(14,:);
            ul = bot.pose(15,:)-bot.pose(16,:);
            %u = bot.pose(4,:)- [bot.pose(4,[1,2]), 3,2];
            vl = bot.pose(12,:) - bot.pose(14,:);
            Cosalphal = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1)+0.02/steps*n;
            alphal = real(acos(Cosalpha));
            dist_hfl = norm(bot.pose(12,:)-bot.pose(14,:));
            htfl = (450-dist_hfl*dist_hfl)/450;
            betal = acos(htfl);
            phil = pi - betal - alphal;
            %delxr = bot.pose(4,1)-bot.pose(6,1);
    
            %theta = (pi-c)/2;
            %phi = pi/2 - alpha - theta;
            if not(isreal(phil))
                dist_hf;
                c;
            end            
            thl = T(al)*Rz(rotation_angle)*Ry(phil-pi/5+(pi/100)*n)*T([15 0 0]);
            bot.pose(13,:) = th(:,4);
            Lknee_init = bot.pose(13,:);
            
            bot.leftLeg(3, :) = Lfoot_init + [x y z 1];
        end
%         if mod(n,2)
%            
%             bot.rightLeg(3, :) = Rfoot_init + [x y z 1];
%         else
%            
%             bot.leftLeg(3, :) = Lfoot_init + [x y z 1];
%         end
%         %centrePelvis
%         rotation_angle = 0;
%         %bot.pose(13,:) = Lknee_init;
%         a = bot.pose(4,:);
%         u = bot.pose(4,:)- [bot.pose(4,[1,2]), 3,2];
%         v = bot.pose(4,:) - bot.pose(6,:);
%         Cosalpha = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
%         alpha = real(acos(Cosalpha));
%         dist_hf = norm(bot.pose(4,:)-bot.pose(6,:));
%         htf = (450-dist_hf*dist_hf)/450;
%         c = acos(htf+0.2*(1-htf));
%         delxr = bot.pose(4,1)-bot.pose(6,1);
%         
%         theta = (pi-c)/2;
%         phi = pi/2 - alpha - theta;
%          
%         th = T(a)*Rz(rotation_angle)*Ry(phi)*T([15 0 0]);
%         bot.pose(5,:) = th(:,4);
%         
%         
%         %bot.pose(5,:) = Rknee_init;
%         rotation_angle = 0;
%         % for left leg
%         
%         b = [bot.pose(12,:)];
%         u1 = bot.pose(12,:)- [bot.pose(12,[1,2]), 3,2];
%         v1 = bot.pose(12,:) - bot.pose(14,:);
%         Cosalphal = max(min(dot(u1,v1)/(norm(u1)*norm(v1)),1),-1);
%         l_alpha = real(acos(Cosalphal));
%         dist_hfl = norm(bot.pose(12,:)-bot.pose(14,:));
%         delxl = bot.pose(12,1)-bot.pose(14,1);
%         htfl = (450-dist_hfl*dist_hfl)/450;
%         lc = acos(htfl + 0.2*(1-htfl));
%         l_theta = (pi-lc)/2;
%         l_phi = pi/2 - l_alpha -l_theta;
%         if step
%             th_l = T(b)*Rz(rotation_angle)*Ry(l_phi)*T([15 0 0]);
%             bot.pose(13,:) = th_l(:,4);
        %bot.rightLeg(1,[1,3]);
        %hipr = bot.pose(4,[1,3]);
        %footr = bot.pose(6,[1,3]);
        % bot.leftLeg     =  bot.pose(12:19, :, :);
        %hipl = bot.pose(12,[1,3]);
        %footl = bot.pose(14,[1,3]);
        %eqn1 = (kneerx-hipr(1,1))^2+(kneerz-hipr(1,2))^2 == 15^2;
        %eqn2 = (kneerx-footr(1,1))^2+(kneerz-footr(1,2))^2 == 15^2;
        %eqn3 = kneerx>hipr(1,1);
        
        %eqn4 = (kneelx-hipl(1,1))^2+(kneelz-hipl(1,2))^2 == 15^2;
        %eqn5 = (kneelx-footl(1,1))^2+(kneelz-footl(1,2))^2 == 15^2;
        %eqn6 = kneelx>hipl(1,1);
        
        %[knlx,knlz] = solve(eqn4,eqn5,eqn6);
        %[knrx,knrz] = solve(eqn1,eqn2,eqn3);
        
        
        %bot.pose(13,1) = bot.pose(12,1)+6;
        %rz=Rz(rotation_angle);
        
        
        %bot.pose(5,:) = [knrx(1,1), -10, knrz(1,1),0];
        %bot.pose(13,:) = [knlx(1,1), 10, knlz(1,1),0];
        %
        %     updateBot(bot)
        %     bot.rightLeg = bot.findLeg("right")
        %     bot.leftLeg = bot.findLeg("left")
        %     bot.pose = pose;
        %     bot.pose(6,2,1) = 10;
        %     bot.pose(6,:,:)
        
        
        
       
        hold on
        
        waistbot = bot.pose(1,:);
        disthiptoknee = [norm(bot.pose(1,:)-bot.pose(5,:)), norm(bot.pose(12,:)-bot.pose(13,:))];  %knee is rightLeg(2,:)
        distkneefoot = [norm(bot.pose(5,:)-bot.pose(6,:)), norm(bot.pose(13,:)-bot.pose(14,:))];
        plot_body(bot);
        view(0,0)
        pause(0.0075)
        
        
    end
    %Lknee_init = bot.pose(13,:); 

    Rfoot_init = bot.rightLeg(3, :);
    Lfoot_init = bot.leftLeg(3, :);
    %mini
    %maxi
end
%plot_body(bot)

% function a = updateBot(bot)    
%     bot.waist       =  bot.pose(1:3, :, :);  % update bot coordinates
%     bot.rightLeg    =  bot.pose(4:11, :, :);
%     bot.torso       =  bot.pose(20:25, :, :);
%     bot.rightArm    =  bot.pose(26:28, :, :);
%     bot.leftArm     =  bot.pose(29:31, :, :);
% end
