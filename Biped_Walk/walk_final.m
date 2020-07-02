stepHeight = 3;
stepLength = 10;
fn = polynomial(stepHeight, stepLength);
step = stepLength;
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
rotation_angle = 0;
newtheta = atan(25/stepLength);
newa = sqrt((step)*(step) + 25*25)
newc = acos((450 - newa*newa)/450);
newgamma = (pi-newc)/2;
phifinal = pi - newgamma - newtheta;

newalpha = acos(5/6);
phiinitial = pi/2 - newalpha

phi_interval = (phifinal - phiinitial)/10;

steps = 20;
for n = [1:steps]
    %mini = 100;
    %maxi = 0;
    
    count = 0;
    if mod(n-1,2)
        bot.pose(13,:) = Lknee_init;
        waistAngle = pi/72;
        foot = bot.pose(6,:);
        if n==2    
            laterall = T(centerPelvis_init-[0 0 25 0])*Rz(rotation_angle)*T([stepLength/2 0 0 0])*Rz(pi/2)*T([10 0 0 0]);
        elseif n==4
            laterall = T(centerPelvis_init-[0 0 25 0])*Rz(rotation_angle)*T([stepLength/2 0 0 0])*Rz(pi/2)*T([20 0 0 0]);
        end
    else
        bot.pose(5,:) = Rknee_init;
        waistAngle = -pi/72;
        foot = bot.pose(14,:);
        if n==1
                lateralr = T(centerPelvis_init-[0 0 25 0])*Rz(rotation_angle)*T([stepLength/2 0 0 0])*Rz(-pi/2)*T([10 0 0 0]);
        elseif n==3
                lateralr = T(centerPelvis_init - [0 0 25 0])*Rz(rotation_angle)*T([stepLength/2 0 0 0])*Rz(-pi/2)*T([20 0 0 0]);    
        end   
        
        
    end    
    for t = linspace(0, stepLength, 15)
        x = t;
        y = 0;
        z = subs(fn);
        
        %syms kneerx kneerz kneelx kneelz

        
        avg = (bot.pose(6,1,1)+bot.pose(14,1,1))/2;
        
        
        %waistAngle = 0;
        %centerPelvis = centerPelvis_init + [avg sin((avg)/(stepLength/2)*2*pi) 0 1];
        
        
        if mod(n,2) == 0
            
            %centerPelvis = centerPelvis_init + [avg t 0 1];
            if n==2
               % centerPelvis = centerPelvis_init + [avg/2 t 0 1];
            end  
            %foot + [0 0 25 0] - centerPelvis;
            
            
            centerPelvis = centerPelvis + [laterall(:,4).']/15;%+[t y 0 1];
            
            bot.rightLeg(3, :) = Rfoot_init + [x   y z 1];
            waistAngle = -pi/36;
        else
             
            lateral(:,4);
            if n==1
                centerPelvis = centerPelvis_init + [lateralr(:,4).']/15;
                
            else
                centerPelvis = centerPelvis + [lateralr(:,4).']/15;
            end
            bot.leftLeg(3, :) = Lfoot_init + [x y z 1];
            waistAngle = +pi/36;
        end

        step = mod(n,2);
        heightWaist = centerPelvis(3);
        %centerPelvis
        bot.pose = bot.findBody(centerPelvis, waistAngle);
        
        bot.pose(5,:);
        bot.pose(13,:) = Lknee_init;
        bot.pose(5,:) = Rknee_init;
        
        if mod(n,2) == 0
            bot.rightLeg(3, :) = Rfoot_init + [x   y z 1];
        else
            bot.leftLeg(3, :) = Lfoot_init + [x y z 1];
        end
        %right leg works fine
        if mod(n+1,2)
            rotation_angle = 0;
            %bot.pose(13,:) = Lknee_init;
            rot = -phiinitial-phi_interval*t;            
            hipdif = -bot.pose(12,:)+centerPelvis;
            
            disthiptofoot = [ norm(bot.pose(4,:)-bot.pose(6,:))];  %knee is rightLeg(2,:)
            distkneefoot = [ norm(bot.pose(5,:)-bot.pose(6,:))];
            Rknee = bot.pose(5,:) + hipdif;%[0 t*(distkneefoot)/(disthiptofoot) 0 0];
            u = bot.pose(5,:) - bot.pose(6,:);
            v = Rknee - bot.pose(6,:);
            Cosalpha = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
            alpha = real(acos(Cosalpha));
            a= norm([lateral(:,4).']);
            
            
            newth = T(bot.pose(6,:))*Rz(rotation_angle)*Rx(0.6*alpha-2*t/15*alpha)*Ry(-phiinitial-phi_interval*t)*T([15 0 0]);
            bot.pose(5,:) = newth(:,4).';
            bot.pose(13,:) = bot.pose(13,:) + 0.8*(distkneefoot)/((disthiptofoot)*15)*([laterall(:,4).']);%(bot.pose(12,:)-centerPelvis);
            Lknee_init = bot.pose(13,:);
            Rknee_init = bot.pose(5,:);
            
        else
            rotation_angle = 0;
            %bot.pose(13,:) = Lknee_init;
            rotl = -phiinitial-phi_interval*t;
            hipdif = bot.pose(4,:)-centerPelvis;
            distlhiptolfoot = [ norm(bot.pose(12,:)-bot.pose(14,:))];  %knee is rightLeg(2,:)
            distlkneelfoot = [ norm(bot.pose(13,:)-bot.pose(14,:))];
            %Lknee = bot.pose(13,:) + [0 -t*(distlkneelfoot)/(distlhiptolfoot) 0 0];
            Lknee = bot.pose(13,:) + hipdif;
            u = bot.pose(13,:) - bot.pose(14,:);
            v = Lknee - bot.pose(14,:);
            Cosalpha = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
            alpha = real(acos(Cosalpha));
            a = norm([lateral(:,4).']);
            newthl = T(bot.pose(14,:))*Rz(rotation_angle)*Rx(-0.6*alpha+2*t/15*alpha)*Ry(-phiinitial-phi_interval*t)*T([15 0 0]);
            
            %newthl = newthl + (bot.pose(13,:) - bot.pose(14,:))*alpha;
            
            bot.pose(13,:) = newthl(:,4).';
            bot.pose(5,:) = bot.pose(5,:) + 0.8*(distlkneelfoot)/((distlhiptolfoot)*15)*([lateralr(:,4).']);%(distlhiptolfoot/distlkneelfoot)*t/15*(bot.pose(4,:)-centerPelvis);
            Lknee_init = bot.pose(13,:);
            Rknee_init = bot.pose(5,:);
            
        end
%         
        
       
        hold on
        
        waistbot = bot.pose(1,:);
        disthiptoknee = [norm(bot.pose(1,:)-bot.pose(5,:)), norm(bot.pose(12,:)-bot.pose(13,:))];  %knee is rightLeg(2,:)
        distkneefoot = [norm(bot.pose(5,:)-bot.pose(6,:)), norm(bot.pose(13,:)-bot.pose(14,:))];
        %centerPelvis
        %centerPelvis = bot.pose(2,:)
        plot_body(bot);
        %view(0,0)
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
