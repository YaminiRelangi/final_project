clear('cam');
clear all
cam = webcam('HD Webcam eMeet C950'); %open the camera
framesAcquired = 0;
%load camera data
I = snapshot(cam);

%detect specific family
[id,loc,pose] = readAprilTag(I, "tag36h11");

%find centerpoints of each tag
for n = 1:size(loc,3)
    centerPoints(n,:) = [((loc(1,1,n)+loc(3,1,n))/2),((loc(1,2,n)+loc(2,2,n))/2)];
end

%Set the tag (tag 0) at the base to the origin, and all other tags in
%relation to tag 0
tag0loc = [centerPoints(1,1)-centerPoints(1,1),centerPoints(1,2)-centerPoints(1,2)];
tag1loc = [centerPoints(2,1)-centerPoints(1,1),abs(centerPoints(2,2)-centerPoints(1,2))];
tag2loc = [centerPoints(3,1)-centerPoints(1,1),abs(centerPoints(3,2)-centerPoints(1,2))];
tag3loc = [centerPoints(4,1)-centerPoints(1,1),abs(centerPoints(4,2)-centerPoints(1,2))];

%find the theta1's of the end effector and the objects in the workspace
%(all objects are assumed to be in the workspace). This is the starting
%angle for everything, and, because the objects don't move, can referenced
%to when the end effector obstructs the camera's view of the objects.
endAngle1 = atan2(tag1loc(2),tag1loc(1));
endAngle2 = atan2(tag2loc(2),tag2loc(1));
endAngle3 = atan2(tag3loc(2),tag3loc(1));

imshow(I)

%%
%Now that all the initial conditions are in place, we can begin the control
%of the robot arm to grab the fork.
%initialize arduino, shield, and the motor
a = arduino('COM3','Mega2560','Libraries','Adafruit\MotorShieldV2');
shield = addon(a,'Adafruit\MotorShieldV2'); %initialize arduino
dcm1 = dcmotor(shield,1);

%%
%Move motor 1 to position to pick up fork
while abs(endAngle1-endAngle2)>0.025
    %if the arm needs to move left to grab the fork
    if endAngle1 < endAngle2
        start(dcm1);
        dcm1.Speed = 0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %if the arm needs to move right to grab the fork
    if endAngle1 > endAngle2
        start(dcm1);
        dcm1.Speed = -0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %update the position of the end effector
    %load camera data
    I2 = snapshot(cam);

    %detect specific family
    [id2,loc2] = readAprilTag(I2, "tag36h11");

    %find centerpoints of the end effector tag
    centerPoints2 = [((loc2(1,1,2)+loc2(3,1,2))/2),((loc2(1,2,2)+loc2(2,2,2))/2)];

    %move centerpoint to origin frame
    tag1loc = [centerPoints2(1,1)-centerPoints(1,1),abs(centerPoints2(1,2)-centerPoints(1,2))];

    %Find angle of end effector
    endAngle1 = atan2(tag1loc(2),tag1loc(1));

    
end

stop(dcm1);

%%
%Engage electromagnet to pick up object
writeDigitalPin(a,'D52',1);

%%
%Send robot arm to drop-off location

while abs(endAngle1-1.5)>0.05
    %if the arm needs to move left to drop-off
    if endAngle1 < 1.5
        start(dcm1);
        dcm1.Speed = 0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %if the arm needs to move right to drop-off
    if endAngle1 > 1.5
        start(dcm1);
        dcm1.Speed = -0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %update the position of the end effector
    %load camera data
    I2 = snapshot(cam);

    %detect specific family
    [id2,loc2] = readAprilTag(I2, "tag36h11");

    %find centerpoints of the end effector tag
    centerPoints2 = [((loc2(1,1,2)+loc2(3,1,2))/2),((loc2(1,2,2)+loc2(2,2,2))/2)];

    %move centerpoint to origin frame
    tag1loc = [centerPoints2(1,1)-centerPoints(1,1),abs(centerPoints2(1,2)-centerPoints(1,2))];

    %Find angle of end effector
    endAngle1 = atan2(tag1loc(2),tag1loc(1));

    
end

stop(dcm1);

%%
%Disengage electromagnet to drop object
writeDigitalPin(a,'D52',0);

%%
%Move motor 1 to pick up knife

while abs(endAngle1-endAngle3)>0.025
    %if the arm needs to move left to grab the fork
    if endAngle1 < endAngle3
        start(dcm1);
        dcm1.Speed = 0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %if the arm needs to move right to grab the fork
    if endAngle1 > endAngle3
        start(dcm1);
        dcm1.Speed = -0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %update the position of the end effector
    %load camera data
    I2 = snapshot(cam);

    %detect specific family
    [id2,loc2] = readAprilTag(I2, "tag36h11");

    %find centerpoints of the end effector tag
    centerPoints2 = [((loc2(1,1,2)+loc2(3,1,2))/2),((loc2(1,2,2)+loc2(2,2,2))/2)];

    %move centerpoint to origin frame
    tag1loc = [centerPoints2(1,1)-centerPoints(1,1),abs(centerPoints2(1,2)-centerPoints(1,2))];

    %Find angle of end effector
    endAngle1 = atan2(tag1loc(2),tag1loc(1));

    
end

stop(dcm1);

%%
%Engage electromagnet to pick up knife
writeDigitalPin(a,'D52',1);

%%
%Send robot arm to drop-off location

while abs(endAngle1-1.2)>0.05
    %if the arm needs to move left to drop-off
    if endAngle1 < 1.2
        start(dcm1);
        dcm1.Speed = 0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %if the arm needs to move right to drop-off
    if endAngle1 > 1.2
        start(dcm1);
        dcm1.Speed = -0.3;
        pause(0.1);
        stop(dcm1);
    end
    
    %update the position of the end effector
    %load camera data
    I2 = snapshot(cam);

    %detect specific family
    [id2,loc2] = readAprilTag(I2, "tag36h11");

    %find centerpoints of the end effector tag
    centerPoints2 = [((loc2(1,1,2)+loc2(3,1,2))/2),((loc2(1,2,2)+loc2(2,2,2))/2)];

    %move centerpoint to origin frame
    tag1loc = [centerPoints2(1,1)-centerPoints(1,1),abs(centerPoints2(1,2)-centerPoints(1,2))];

    %Find angle of end effector
    endAngle1 = atan2(tag1loc(2),tag1loc(1));

    
end

stop(dcm1);

%%
%Disengage electromagnet to drop object
writeDigitalPin(a,'D52',0);
