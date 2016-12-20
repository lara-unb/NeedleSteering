close all;
clear all;
clc;

rotation_speed = 0.1;

%% Start communication with the Raspberry Pi TCP/IP server

ustep_device = UStepDeviceHandler(2);

%% Read command from user

while 1
    fprintf('\nTesting the needle rotation \t--\t speed = %f rev/s\n', rotation_speed);
    angle = input('Type the angle of revolutions in CW direction. If you want to change the speed, type 0\n');
    
    if(angle == 0)
        rotation_speed = input('Type the requested speed in rev/s\n');
    
    else
        ustep_device.rotateNeedleDegrees(angle, rotation_speed);
    end
    
    pause(1);
end
    