classdef Humanoid
    %HUMANOID Basic model of the humanoid with basic functions
    %   Detailed explanation goes here
    
    properties
        distHip_Knee
        distKnee_Foot
        distWaist
        distTorso
        distArms
        footBase
        pose
        waist
        rightLeg
        leftLeg
        torso
        rightArm
        leftArm
    end
    
    methods
        function obj = Humanoid(params)
            %HUMANOID Construct an instance of this class
            %   Detailed explanation goes here
            obj.distHip_Knee = params(1,1);
            obj.distKnee_Foot = params(2,1);
            obj.footBase = params(3,:);
            obj.distWaist = params(4,1);
            obj.distTorso = params(5,1);
            obj.distArms = params(6,1);
                       
            obj.rightLeg = zeros(8,4);
            obj.leftLeg = zeros(8,4);
            obj.waist = zeros(3,4);
            obj.torso = zeros(6,4);
            obj.rightArm = zeros(3, 4);
            obj.leftArm = zeros(3, 4);
            
            obj.rightLeg(:,4) = 1;
            obj.leftLeg(:,4) = 1;
            obj.waist(:,4) = 1;
            obj.torso(:,4) = 1;
            obj.rightArm(:,4) = 1;
            obj.leftArm(:,4) = 1;
        end
        
        function arm = findArm(obj, targetArm, theta1, theta2)
            if targetArm == "right"
                arm = obj.rightArm;
            else
                arm = obj.leftArm;
            end
            arm(2, :) = arm(1, :) + [-obj.distArms*sin(theta1) 0 -obj.distArms*cos(theta1) 0];
            arm(3, :) = arm(2, :) + [-obj.distArms*sin(theta2) 0 -obj.distArms*cos(theta2) 0];
        end
        
        function [waist, torso] = findWaist(obj, point, theta,phi)
            waist = zeros(3,4);
            waist(2, :) = point;
            waist(1, :) = waist(2, :) - [obj.distWaist*cos(theta)*sin(phi)/2 obj.distWaist*cos(theta)*cos(phi)/2 obj.distWaist*sin(theta)/2 0];
            waist(3, :) = waist(2, :) + [obj.distWaist*cos(theta)*sin(phi)/2 obj.distWaist*cos(theta)*cos(phi)/2 obj.distWaist*sin(theta)/2 0];
	        torso = zeros(5, 4);
            torso(1, :) = waist(2, :);
            torso(2, :) = torso(1, :) + [obj.distTorso*cos(pi/2 + theta)*sin(phi), obj.distTorso*cos(pi/2 + theta)*cos(phi) obj.distTorso*sin(pi/2 + theta) 0];
            torso(3, :) = torso(2, :) + [5*cos(pi/2 + theta)*sin(phi), 5*cos(pi/2 + theta)*cos(phi) 5*sin(pi/2 + theta) 0];
            torso(4, :) = torso(2, :);
            torso(5, :) = torso(2, :) - [obj.distWaist*cos(theta)*sin(phi)/2 obj.distWaist*cos(theta)*cos(phi)/2 obj.distWaist*sin(theta)/2 0];
            torso(6, :) = torso(2, :) + [obj.distWaist*cos(theta)*sin(phi)/2 obj.distWaist*cos(theta)*cos(phi)/2 obj.distWaist*sin(theta)/2 0];
        end
         
         function leg = findLeg(obj, targetLeg,rotation_angle)
            %FINDLEG Summary of this method goes here
            %   Detailed explanation goes here
            if targetLeg == "right"
                leg = obj.rightLeg;
            else
                leg = obj.leftLeg;
            end
            
            t1 = -atan((leg(1,2) - leg(3,2))/(leg(1,3) - leg(3,3)));
            leg1_3 = inv(T(leg(3,:))*Rx(t1))*(leg(1, :))';
            leg2_3 = [obj.distKnee_Foot*sin(2*atan((2*obj.distKnee_Foot*leg1_3(1) - (obj.distKnee_Foot^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) - (obj.distHip_Knee^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (leg1_3(1)^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (leg1_3(3)^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (2*obj.distKnee_Foot*obj.distHip_Knee*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2))/(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*leg1_3(3) - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)));
                      0;
                      obj.distKnee_Foot*cos(2*atan((2*obj.distKnee_Foot*leg1_3(1) - (obj.distKnee_Foot^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) - (obj.distHip_Knee^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (leg1_3(1)^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (leg1_3(3)^2*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2) + (2*obj.distKnee_Foot*obj.distHip_Knee*((- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)*(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee + obj.distHip_Knee^2 - leg1_3(1)^2 - leg1_3(3)^2))^(1/2))/(- obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*obj.distHip_Knee - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2))/(obj.distKnee_Foot^2 + 2*obj.distKnee_Foot*leg1_3(3) - obj.distHip_Knee^2 + leg1_3(1)^2 + leg1_3(3)^2)));
                      1];
%             leg2_3 = testPoint((leg1_3)', obj.distKnee_Foot, obj.distHip_Knee);
            leg(2, :) = T(leg(3,1:3))*Rx(t1)*(leg2_3);
            
            
            
            tp1=[1 0 0 -leg(3,1)
                0 1 0 -leg(3,2)
                0 0 1 -leg(3,3)
                0 0 0 1];
            u=leg(1,1)-leg(3,1);
            v=leg(1,2)-leg(3,2);
            w=leg(1,3)-leg(3,3);                         
            rx = [1,0,0,0;
                0,w/(sqrt(v*v + w*w)),-v/(sqrt(v*v + w*w)),0;
                0,v/(sqrt(v*v + w*w)),w/(sqrt(v*v + w*w)),0;
                0,0,0,1];
            ry = [(sqrt(v*v+w*w))/sqrt(u*u +v*v+w*w),0,-u/sqrt(u*u +v*v+ w*w),0;
                0,1,0,0;
                u/sqrt(u*u +v*v+ w*w),0,(sqrt(v*v+w*w))/sqrt(u*u +v*v+w*w),0;
                0,0,0,1];
            rz=Rz(rotation_angle);
            n=inv(tp1)*inv(rx)*inv(ry)*rz*ry*rx*tp1;
            leg(2,:) = leg(2,:)*(n');
            
            rzp=inv(tp1)*rz*tp1;
            leg(4, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(5, :) = (leg(3, :) + [-obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(6, :) = (leg(3, :) + [-obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(7, :) = (leg(3, :) + [obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(8, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            %leg = obj.rotation(targetLeg,rotation_angle);
%             T(leg(3,1:3))*Rx(t1)*(leg1_3)
%             leg(1, 1:3)
%             
%             dist13 = norm(leg2_3(1:3))
%             dist23 = norm(leg1_3(1:3) - leg2_3(1:3))
%             
%             dist1 = norm(leg(1,1:3) - leg(2,1:3))
%             dist2 = norm(leg(2, 1:3) - leg(3, 1:3))
         end
        
         
        function leg = rotation(obj,targetLeg,angle)
            if targetLeg == "right"
                leg = obj.rightLeg;
            else
                leg = obj.leftLeg;
            end
           tp1=[1 0 0 -leg(3,1)
                0 1 0 -leg(3,2)
                0 0 1 -leg(3,3)
                0 0 0 1];
            u=leg(1,1)-leg(3,1);
            v=leg(1,2)-leg(3,2);
            w=leg(1,3)-leg(3,3);                         
            rx = [1,0,0,0;
                0,w/(sqrt(v*v + w*w)),-v/(sqrt(v*v + w*w)),0;
                0,v/(sqrt(v*v + w*w)),w/(sqrt(v*v + w*w)),0;
                0,0,0,1];
            ry = [(sqrt(v*v+w*w))/sqrt(u*u +v*v+w*w),0,-u/sqrt(u*u +v*v+ w*w),0;
                0,1,0,0;
                u/sqrt(u*u +v*v+ w*w),0,(sqrt(v*v+w*w))/sqrt(u*u +v*v+w*w),0;
                0,0,0,1];
            rz=Rz(angle);
            n=inv(tp1)*inv(rx)*inv(ry)*rz*ry*rx*tp1;
            leg(2,:) = leg(2,:)*(n');
             rzp=inv(tp1)*rz*tp1;
            leg(4, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(5, :) = (leg(3, :) + [-obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(6, :) = (leg(3, :) + [-obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(7, :) = (leg(3, :) + [obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(8, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
        end    
        
        
        function pose = findBody(obj)
%             [obj.waist, obj.torso] = obj.findWaist(centrePelvis, waistAngle,angleYaw);
%             
%             obj.rightLeg(1, :) = obj.waist(1, :);
%             
%             obj.leftLeg(1, :) = obj.waist(3, :);
%             obj.rightLeg = obj.findLeg("right");
%             obj.leftLeg = obj.findLeg("left");
%             obj.rightLeg = obj.rotation("right",right_angle);
%             obj.leftLeg = obj.rotation("left",left_angle);
            
            obj.rightArm(1, :) = obj.torso(5, :);
            obj.leftArm(1, :) = obj.torso(6, :);
            
            obj.rightArm = obj.findArm("right", 0, 0);
            obj.leftArm = obj.findArm("left", 0, 0);
            
            pose = [obj.waist,
                    obj.rightLeg,
                    obj.leftLeg,
                    obj.torso,
                    obj.rightArm,
                    obj.leftArm];
            obj.pose = pose;
                
        end
        
        
        
%         function pose = init(obj)
%             %INIT Summary of this method goes here
%             %   Detailed explanation goes here
%             centerPelvis = [0, 0, obj.distKnee_Foot + obj.distHip_Knee - 5, 1];
%             waistAngle =0;
%             angleYaw=0;
%             right_angle=0;
%             left_angle=0;
%             obj.rightLeg(3, :) = [0 -obj.distWaist/2 0 1];
%             obj.leftLeg(3, :) = [0 obj.distWaist/2 0 1];
%             
%             pose = obj.findBody(centerPelvis, waistAngle,angleYaw,right_angle,left_angle);
%             obj.pose = pose;
%         end
       
        function t = turn_sim(obj,yawAngle,right_angle,left_angle)
           pause(10);
            centrePelvis = [0, 0, obj.distKnee_Foot + obj.distHip_Knee - 5, 1];
            obj.rightLeg(3, :) = [0 -obj.distWaist/2 0 1];
            obj.leftLeg(3, :) = [0 obj.distWaist/2 0 1];
            
            
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis,-pi/10,0);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.leftLeg = obj.findLeg( "left",0);
            obj.rightLeg = obj.findLeg( "right",0);
            
            obj.pose = obj.findBody();
            plot_body(obj.pose);
            pause(5);
            
           obj.rightLeg(3, :) = [0 -obj.distWaist/2 5 1];
            obj.rightLeg = obj.findLeg("right",0);
            obj.pose = obj.findBody();
            plot_body(obj.pose);
            pause(5);
            
            obj.rightLeg=obj.rotation("right",right_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
            pause(5);
            
            obj.rightLeg(3, :) = [0 -obj.distWaist/2 0 1];
            obj.rightLeg = obj.findLeg( "right",right_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
            pause(5);
            
            
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, pi/10,0);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.leftLeg = obj.findLeg( "left",0);
            obj.rightLeg = obj.findLeg( "right",right_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
            pause(5);
            
            obj.leftLeg(3, :) = [0 obj.distWaist/2 5 1];
            obj.leftLeg = obj.findLeg("left",0);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
             pause(5);
             
            obj.leftLeg=obj.rotation("left",left_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
             pause(5);
             
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, pi/10,yawAngle);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.rightLeg = obj.findLeg( "right",left_angle);
              obj.leftLeg = obj.findLeg("left",left_angle);
              
              
              obj.pose = obj.findBody();
              plot_body(obj.pose);
              pause(5);
               
            obj.leftLeg(3, :) = [0 obj.distWaist/2 0 1];
            obj.leftLeg = obj.findLeg("left",left_angle);
            obj.pose = obj.findBody();
             plot_body(obj.pose);
             pause(5);
             
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, 0,yawAngle);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.leftLeg = obj.findLeg( "left",left_angle);
            obj.rightLeg = obj.findLeg("right",left_angle);
            obj.pose = obj.findBody();
             plot_body(obj.pose);
        end
    end
end

