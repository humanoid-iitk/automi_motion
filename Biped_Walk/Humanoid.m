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
            leg(2, :) = T(leg(3,1:3))*Rx(t1)*(leg2_3);  
           
            tp=T(-leg(3,:));                            %translation of origin to foot
            u=leg(1,1)-leg(3,1);
            v=leg(1,2)-leg(3,2);
            w=leg(1,3)-leg(3,3);
            theta1 = atan(v/w);
            theta2=atan(u/sqrt(v*v+w*w));
            rx=Rx(theta1);
            ry=Ry(-theta2);
            rz=Rz(rotation_angle);
            n=inv(tp)*inv(rx)*inv(ry)*rz*ry*rx*tp;      %the equivalent matrix for rotation about an arbitrary axis
            leg(2,:) = leg(2,:)*(n');                   %rotate knee(leg(2,:)) about the line joining hip and foot
            
            rzp=inv(tp)*rz*tp;
            leg(4, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(5, :) = (leg(3, :) + [-obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(6, :) = (leg(3, :) + [-obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(7, :) = (leg(3, :) + [obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(8, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
         end
        
         
        function leg = rotation(obj,targetLeg,angle)
            if targetLeg == "right"
                leg = obj.rightLeg;
            else
                leg = obj.leftLeg;
            end
            tp=T(-leg(3,:));                         %translation of origin to foot
            u=leg(1,1)-leg(3,1);
            v=leg(1,2)-leg(3,2);
            w=leg(1,3)-leg(3,3);
            theta1 = atan(v/w);
            theta2=atan(u/sqrt(v*v+w*w));
            rx=Rx(theta1);
            ry=Ry(-theta2);
            rz=Rz(angle);
        
            n=inv(tp)*inv(rx)*inv(ry)*rz*ry*rx*tp;   %the equivalent matrix for rotation about an arbitrary axis
            leg(2,:) = leg(2,:)*(n');                %rotate knee(leg(2,:)) about the line joining hip and foot
            
            rzp=inv(tp)*rz*tp;
            leg(4, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(5, :) = (leg(3, :) + [-obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
            leg(6, :) = (leg(3, :) + [-obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(7, :) = (leg(3, :) + [obj.footBase(1)/2 -obj.footBase(2)/2 0 0])*(rzp');
            leg(8, :) = (leg(3, :) + [obj.footBase(1)/2 obj.footBase(2)/2 0 0])*(rzp');
        end    
        
        
        function pose = findBody(obj)
            
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
        
       
        function t = turn_sim(obj,yawAngle,rollAngle,right_angle,left_angle)
           pause(10);
            centrePelvis = [0, 0, obj.distKnee_Foot + obj.distHip_Knee - 5, 1];
            obj.rightLeg(3, :) = [0 -obj.distWaist/2 0 1];
            obj.leftLeg(3, :) = [0 obj.distWaist/2 0 1];
            
            %pose1
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis,-pi/10,0);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 0 1];
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 0 1];
            obj.leftLeg = obj.findLeg( "left",0);
            obj.rightLeg = obj.findLeg( "right",0);
            
            obj.pose = obj.findBody();
            plot_body(obj.pose);
            pause(5);
            
            %pose2
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 5 1];
            
            obj.rightLeg = obj.findLeg("right",0);
            obj.pose = obj.findBody();
            plot_body(obj.pose);
            pause(5);
            
            %pose3
            obj.rightLeg=obj.rotation("right",right_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
            pause(5);
            
            %pose4
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 0 1];
            obj.rightLeg = obj.findLeg( "right",right_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
            pause(5);
            
            %pose5
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, pi/10,0);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 0 1];
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 0 1];
            obj.leftLeg = obj.findLeg( "left",0);
            obj.rightLeg = obj.findLeg( "right",right_angle);
            obj.pose = obj.findBody();
             plot_body(obj.pose);
            pause(5);
            
            %pose6
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 5 1];
            obj.leftLeg = obj.findLeg("left",0);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
             pause(5);
             
             %pose7
            obj.leftLeg=obj.rotation("left",left_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
             pause(5);
             
             %pose7
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, pi/10,yawAngle);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 0 1];
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 0 1];
            obj.rightLeg = obj.findLeg( "right",left_angle);
            obj.leftLeg = obj.findLeg("left",left_angle);
            obj.pose = obj.findBody();
              plot_body(obj.pose);
              pause(5);
            
              %pose8
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 0 1];
            obj.leftLeg = obj.findLeg("left",left_angle);
            obj.pose = obj.findBody();
             plot_body(obj.pose);
             pause(5);
             
             %pose9
            [obj.waist, obj.torso] = obj.findWaist(centrePelvis, rollAngle,yawAngle);
            obj.rightLeg(1, :) = obj.waist(1, :);
            obj.leftLeg(1, :) = obj.waist(3, :);
            obj.rightLeg(3, :) = [obj.waist(1, 1) obj.waist(1, 2) 0 1];
            obj.leftLeg(3, :) = [obj.waist(3, 1) obj.waist(3, 2) 0 1];
            obj.leftLeg = obj.findLeg( "left",left_angle);
            obj.rightLeg = obj.findLeg("right",left_angle);
            obj.pose = obj.findBody();
             plot_body(obj.pose);
        end
    end
end

