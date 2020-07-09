# automi_sim
matlab simulation of various tasks for Automi

For Turn simulation run the commands as specified in test.m:
Set the values of params
params = [  [15, 0];        
            [15, 0];  
            [11, 7];
            [20, 0];
            [25, 0];
            [10, 0]];
m = Humanoid(params);
m.turn_sim(obj,yawAngle,rollAngle'right_angle,left_angle);

# Description of turn simulation

Function: turn_sim(obj,yawAngle,rollAngle,right_angle,left_angle)
Arguments: yawAngle= angle by which the bot turns about the z-axis
           rollAngle: angle by which waist and torso turn about the x-axis
           right_angle= angle by which right knee turns about the line joining hip and foot
           left_angle= angle by which right knee turns about the line joining hip and foot
           
        Pose 1: Centre of mass shifts above the left leg
        Pose 2: Right foot rises up
        Pose 3: Right leg rotates
        Pose 4: Right foot on ground again
        Pose 5: COM shifts above right leg
        Pose 6: Left foot rises up
        Pose 7: Waist turns by angle=YawAngle
        Pose 8: Left leg rotates
        Pose 9: Left foot again on ground


