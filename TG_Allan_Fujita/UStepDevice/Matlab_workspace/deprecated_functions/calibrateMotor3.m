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

speed = 4 * 2000;

%% Start communication with the Raspberry Pi TCP/IP server

tcpip_client = tcpip('169.254.0.2',5555,'NetworkRole','Client');
set(tcpip_client,'InputBufferSize',7688);
set(tcpip_client,'Timeout',30);

%% Grasp the Needle

fprintf('Send commands to motor 4, in steps, until the needle is firmly grasped\n');

needle_grasped = 0;
while(~needle_grasped)
    steps = input('Type the remaining steps to firmly grasp the needle:\n');
    displacement = 0;
    if(steps == 0)
        needle_grasped = 1;
        break;
    end
    
    fopen(tcpip_client);
    if(steps < 0)
        fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_FRONT_GRIPPER DIRECTION_OPENING]);
        pause(0.5);
        displacement = -steps;
    else
        fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_FRONT_GRIPPER DIRECTION_CLOSING]);
        pause(0.5);
        displacement = steps;
    end
    
    fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
    fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_FRONT_GRIPPER typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
    fclose(tcpip_client);
end

%% Initialize the needle position

fprintf('Send commands to motor 3, in steps, until the needle is reaches the initial position\n');

initial_position_reached = 0;
while(~initial_position_reached)
    steps = input('Type the remaining steps to reach the initial position:\n');
    displacement = 0;
    if(steps == 0)
        initial_position_reached = 1;
        break;
    end
    
    fopen(tcpip_client);
    if(steps < 0)
        fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_COUNTER_CLOCKWISE]);
        pause(0.5);
        displacement = -steps;
    else
        fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_CLOCKWISE]);
        pause(0.5);
        displacement = steps;
    end
    
    fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
    fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_ROTATION typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
    fclose(tcpip_client);
end

%% Start calibration - positive part

fprintf('Send commands to motor 3, in steps, to perform clockwise rotations\n\n');

needle_angles_positive = 360*[1 2 3 4 5];
n_angles = length(needle_angles_positive);
step_displacement_positive = zeros(1, n_angles);

for i_angle = 1:n_angles
    
    fprintf('Performing the %dth rotation in clockwise direction\n', i_angle);
    
    angle_reached = 0;
    total_steps = 0;
    while(~angle_reached)
        steps = input('Type the remaining steps to reach the target position:\n');
        displacement = 0;
        if(steps == 0)
            angle_reached = 1;
            break;
        end
        
        total_steps = total_steps+steps;
        
        fopen(tcpip_client);
        if(steps < 0)
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_COUNTER_CLOCKWISE]);
            pause(0.5);
            displacement = -steps;
        else
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_CLOCKWISE]);
            pause(0.5);
            displacement = steps;
        end
        
        fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
        fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_ROTATION typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
        fclose(tcpip_client);
    end
    
    step_displacement_positive(i_angle) = total_steps;
end

%% Start calibration - negative part

fprintf('Send commands to motor 3, in steps, to perform counter clockwise rotations\n\n');

needle_angles_negative = 360*[-1 -2 -3 -4 -5];
n_angles = length(needle_angles_negative);
step_displacement_negative = zeros(1, n_angles);

for i_angle = 1:n_angles
    
    fprintf('Performing the %dth rotation in counter clockwise direction\n', i_angle);
    
    angle_reached = 0;
    total_steps = 0;
    while(~angle_reached)
        steps = input('Type the remaining steps to reach the target position:\n');
        displacement = 0;
        if(steps == 0)
            angle_reached = 1;
            break;
        end
        
        total_steps = total_steps+steps;
        
        fopen(tcpip_client);
        if(steps < 0)
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_COUNTER_CLOCKWISE]);
            pause(0.5);
            displacement = -steps;
        else
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_CLOCKWISE]);
            pause(0.5);
            displacement = steps;
        end
        
        fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
        fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_ROTATION typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
        fclose(tcpip_client);
    end
    
    step_displacement_negative(i_angle) = total_steps;
end


%% Start calibration - alternate part

fprintf('Send commands to motor 3, in steps, to perform alternated rotations\n\n');

n_alternate = 10;
step_displacement_alternate = zeros(1, n_alternate);

for i_alternate = 1:n_alternate
    
    fprintf('Performing the %dth rotation in alternate mode\n', i_alternate);
    if(floor(mod(i_alternate/2,2)))
        fprintf('\nThe next rotation should be in the counter clockwise direction\n\n');
    else
        fprintf('\nThe next rotation should be in the clockwise direction\n\n');
    end
    
    angle_reached = 0;
    total_steps = 0;
    while(~angle_reached)
        steps = input('Type the remaining steps to reach the target position:\n');
        displacement = 0;
        if(steps == 0)
            angle_reached = 1;
            break;
        end
        
        total_steps = total_steps+steps;
        
        fopen(tcpip_client);
        if(steps < 0)
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_COUNTER_CLOCKWISE]);
            pause(0.5);
            displacement = -steps;
        else
            fwrite(tcpip_client, [CMD_SET_DIRECTION MOTOR_ROTATION DIRECTION_CLOCKWISE]);
            pause(0.5);
            displacement = steps;
        end
        
        fprintf('Sending the MOVE_MOTOR_STEPS (%d) command with disp = %d and speed = %d\n', CMD_MOVE_MOTOR_STEPS, displacement, speed);
        fwrite(tcpip_client, [CMD_MOVE_MOTOR_STEPS MOTOR_ROTATION typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
        fclose(tcpip_client);
    end
    
    step_displacement_alternate(i_alternate) = total_steps;
end
