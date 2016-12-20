close all;
clear all;
clc;

global CMD_SET_ENABLE;
global CMD_OPEN_FRONT_GRIPPER;		
global CMD_CLOSE_FRONT_GRIPPER;		
global CMD_OPEN_BACK_GRIPPER;		
global CMD_CLOSE_BACK_GRIPPER;
global CMD_ROTATE;
global CMD_TRANSLATE;
global CMD_MOVE_DC;
global CMD_MOVE_BACK;
global CMD_MOVE_MOTOR;
global CMD_MOVE_MOTOR_STEPS;
global CMD_SET_DIRECTION;
global CMD_SHUT_DOWN;

global MOTOR_INSERTION;
global MOTOR_ROTATION; 
global MOTOR_FRONT_GRIPPER;
global MOTOR_BACK_GRIPPER;

global DIRECTION_FORWARD;
global DIRECTION_BACKWARD;    
global DIRECTION_CLOCKWISE;
global DIRECTION_COUNTER_CLOCKWISE;
global DIRECTION_OPENING;
global DIRECTION_CLOSING;

global ENABLE_MOTOR;
global DISABLE_MOTOR;

communicationProtocolTable

% speed = 1904;
speed = 400;

insertion_speed = 2 * 5000;
insertion_displacement = 30 * 5000;

%% Start communication with the Raspberry Pi TCP/IP server

tcpip_client = tcpip('169.254.0.2',5555,'NetworkRole','Client');
set(tcpip_client,'InputBufferSize',7688);
set(tcpip_client,'Timeout',30);


%% Initialize the motor position

% closing_angles = [13300 13350 13400 13450 13500];
closing_angles = 1345;
n_angles = length(closing_angles);
grasp_results = zeros(1, n_angles);

for i_angle = 1:n_angles
    
    fprintf('Disabling the motor 4\n');
    fopen(tcpip_client);
    fwrite(tcpip_client, [CMD_SET_ENABLE MOTOR_FRONT_GRIPPER DISABLE_MOTOR]);
    fclose(tcpip_client);
    
    fprintf('Please move the front gripper to the completely open position\n');
    input('Hit ENTER you are done\n');
    
    fprintf('Closing the gripper with %d steps\n', closing_angles(i_angle));
    fopen(tcpip_client);
    fwrite(tcpip_client, [CMD_SET_ENABLE MOTOR_FRONT_GRIPPER ENABLE_MOTOR]);
    pause(0.5);
    fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_FRONT_GRIPPER DIRECTION_CLOSING]);
    pause(0.5);
    displacement = closing_angles(i_angle);
    fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_FRONT_GRIPPER typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
    fclose(tcpip_client);
    
    fprintf('Preparing to insert the needle into the gelatin for testing the grasp\n');
    input('Hit ENTER you are ready\n');
    
%     fprintf('Inserting the needle. Please check for slippage\n');
%     fopen(tcpip_client);
%     fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_INSERTION DIRECTION_FORWARD]);
%     pause(0.5);
%     fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_INSERTION typecast(insertion_displacement, 'uint8') typecast(insertion_speed, 'uint8')]);
%     fclose(tcpip_client);
%     input('Hit ENTER when the insertion is complete\n');
%     
%     fprintf('Retreating the needle. Continue checking for slippage\n');
%     fopen(tcpip_client);
%     fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_INSERTION DIRECTION_BACKWARD]);
%     pause(0.5);
%     fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_INSERTION typecast(insertion_displacement, 'uint8') typecast(insertion_speed, 'uint8')]);
%     fclose(tcpip_client);
    grasp_results(i_angle) = input('Was there any slippage? 1(YES), 2(NO), 3(NOT SURE)\n');
    fprintf('\n\n');
end

%% Stop the communication

