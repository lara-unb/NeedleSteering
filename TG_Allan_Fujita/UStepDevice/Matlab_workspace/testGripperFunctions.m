close all;
clear all;
clc;

%% Start communication with the Raspberry Pi TCP/IP server

ustep_device = UStepDeviceHandler(2);

%% Read command from user

while 1
    
    fprintf('Gripper Test functions. Select one command:\n');
    command = input('1: Open Front Gripper \n2: Close Front Gripper \n3: Open Back Gripper \n4: Close Back Gripper\n');
    
    if(command == 1)
        fprintf('Sending the command OPEN_FRONT_GRIPPER to the device\n\n');
        ustep_device.openFrontGripper();
    elseif(command == 2)
        fprintf('Sending the command CLOSE_FRONT_GRIPPER to the device\n\n');
        ustep_device.closeFrontGripper();
    elseif(command == 3)
        fprintf('Sending the command OPEN_BACK_GRIPPER to the device\n\n');
        ustep_device.openBackGripper();
    elseif(command == 4)
        fprintf('Sending the command CLOSE_BACK_GRIPPER to the device\n\n');
        ustep_device.closeBackGripper();
    else
        fprintf('Invalid option\n\n');
    end
    
    pause(1);
end