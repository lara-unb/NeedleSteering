close all;
clear all;
clc;

speed = 1.0;

%% Start communication with the Raspberry Pi TCP/IP server

ustep_device = UStepDeviceHandler(2);

%% Read command from user

while 1
    
    fprintf('\nMotor 1 motion test - moving motor 1 at the speed of %f mm/s\n', speed);
    displacement = input('Type the required displacement in mm. If you want to change the speed, type 0\n');
    
    if(displacement == 0)
        speed = input('Type the requested speed in mm/s\n');
        
    else
        ustep_device.translateFrontGripper(displacement, speed);
    end
    
    pause(1);
end
