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

speed = 2 * 20000;

%% Start communication with the Raspberry Pi TCP/IP server

tcpip_client = tcpip('169.254.0.2',5555,'NetworkRole','Client');
set(tcpip_client,'InputBufferSize',7688);
set(tcpip_client,'Timeout',30);

%% Initialize the motor position

fprintf('Please disable the motor and move it to the position 0 mm (on the back limit)\n');
input('Hit ENTER you are done\n');

target_positions = [10 20 30 40 50 60 70 80 90 100];
n_positions = length(target_positions);
step_displacement = zeros(1, n_positions);

for i_position = 1:n_positions
    
    fprintf('Moving the motor to the position %f mm\n', target_positions(i_position));
    
    position_reached = 0;
    total_steps = 0;
    while(~position_reached)
        steps = input('Type the remaining steps to reach the target position:\n');
        displacement = 0;
        if(steps == 0)
            position_reached = 1;
            break;
        end
        
        total_steps = total_steps+steps;
        
        fopen(tcpip_client);
        if(steps < 0)
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_INSERTION DIRECTION_BACKWARD]);
            pause(0.5);
            displacement = -steps;
        else
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_INSERTION DIRECTION_FORWARD]);
            pause(0.5);
            displacement = steps;
        end
        
        fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
        fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_INSERTION typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
        fclose(tcpip_client);
    end
    
    step_displacement(i_position) = total_steps;
end

%% Stop the communication

