close all;
clear all;
clc;

%% Global parameters

aurora_present = 0;
telescoping_suport_mode = 1;

sensor_angle_inside_needle = -180.0;

% Needle initial orientation
needle_V0 = [0 0 1];
needle_N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

n_preparation_step = 4;
preparation_step_size = 10;
preparation_insertion_speed = 2.0;
preparation_rotation_speed = 0.1;
insertion_correction_speed = 1.0;

% Insertion trajectory
pre_insertion = 11.0;
step_size = 8;
insertion_speed = 1.0;

n_step = 14;
minimum_insertion = [1.00 1.00 1.00 1.00 2.00 1.00 1.00 1.00 1.00 1.00 1.00 2.00 2.00 1.00];
duty_cycle        = [0.00 0.00 0.00 0.25 0.50 0.25 0.00 0.00 0.00 0.00 0.00 0.50 0.50 0.00];
rotation_steps    = [0    0    0    0    1    0    0    0    0    0    0    0    0    0];

% n_step = 4;
% minimum_insertion = [1.00 1.00 2.00 1.00];
% duty_cycle        = [0.00 0.25 0.50 0.00];
% rotation_steps    = [0    0    1    0];

%% Configure the TCP/IP client for communicating with the Raspberry Pi

ustep_device = UStepDeviceHandler(n_step);

%% Configure the Aurora sensor object

aurora_device = AuroraDriver('/dev/ttyUSB0');
if(aurora_present)    
    aurora_device.openSerialPort();
    aurora_device.init();
    aurora_device.detectAndAssignPortHandles();
    aurora_device.initPortHandleAll();
    aurora_device.enablePortHandleDynamicAll();
    aurora_device.startTracking();
end

%% Set file name for storing the results

fprintf('Open Loop Trajectory Experiment\n');
simulateDutyCyclePlanarTrajectory(duty_cycle, rotation_steps, pre_insertion);
output_file_name = input('Type the name of the file for saving the results\n','s');

experiment_start = tic;

%% Adjust the needle starting position

fprintf('Place the needle inside the device \nMake sure to leave exactly %.2f mm of needle outside the device\n', pre_insertion);
input('When you are done, hit ENTER to close the front gripper\n');
ustep_device.closeFrontGripper();

if(aurora_present)
    
    % Moving the needle forward until it gets detected by the Aurora system
    fprintf('Adjusting the needle initial orientation\n');
    
    % Moving the needle backward
    if(telescoping_suport_mode)
        ustep_device.moveForward(preparation_step_size*n_preparation_step, preparation_insertion_speed);
    else
        for i_preparation_step = 1:n_preparation_step
            ustep_device.moveForward(preparation_step_size, preparation_insertion_speed);
        end
    end
    
    % Adjusting the needle orientation
    aurora_device.updateSensorDataAll();
    needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
    correction_angle = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);
    
    fprintf('Needle found! Correction angle = %.2f degrees \n', correction_angle);
    ustep_device.rotateNeedleDegrees(correction_angle, preparation_rotation_speed);
    
    aurora_device.BEEP('1');
    answer = input('\nDoes this angle seem correct? (y/n)\n','s');
    if(~(strcmp(answer, 'y') || strcmp(answer, 'Y') || strcmp(answer, 'yes') || strcmp(answer, 'Yes') || strcmp(answer, 'YES')))
        
        correction_angle = 0;
        while 1
            angle = input('Type the correction angle, in CW direction, to adjust the needle orientation:\n');
            
            if(isempty(angle))
                continue;
            end
            
            if(angle == 0)
                fprintf('Needle orientation adjusted.\n');
                fprintf('Next time, you should set the variable "sensor_angle_inside_needle" to %.2f\n', sensor_angle_inside_needle+correction_angle);
                break;
            else
                correction_angle = correction_angle + angle;
                ustep_device.rotateNeedleDegrees(angle, preparation_rotation_speed);
            end
        end
        
    end
    
    % Moving the needle backward
    if(telescoping_suport_mode)
        ustep_device.moveBackward(preparation_step_size*n_preparation_step, preparation_insertion_speed);
    else
        for i_preparation_step = 1:n_preparation_step
            ustep_device.moveBackward(preparation_step_size, preparation_insertion_speed);
        end
    end
    
    aurora_device.BEEP('2');
end

if(telescoping_suport_mode == 0)
    fprintf('Check if there is %.2f mm of needle outside the device\n', pre_insertion);
    answer = input('Is this correct? (y/n)\n','s');
    if(~(strcmp(answer, 'y') || strcmp(answer, 'Y') || strcmp(answer, 'yes') || strcmp(answer, 'Yes') || strcmp(answer, 'YES')))
        
        ustep_device.closeBackGripper();
        ustep_device.openFrontGripper();
        ustep_device.translateFrontGripper(-preparation_step_size, preparation_insertion_speed);
        ustep_device.closeFrontGripper();
        ustep_device.openBackGripper();
        
        correction_displacement = 0;
        while 1
            displacement = input('Type the required displacement in mm\n');
            
            if(isempty(displacement))
                continue;
            end
            
            if(displacement == 0)
                break;
            else
                correction_displacement = correction_displacement + displacement;
                ustep_device.translateFrontGripper(displacement, insertion_correction_speed);
            end
        end
        
        ustep_device.closeBackGripper();
        ustep_device.openFrontGripper();
        ustep_device.translateFrontGripper(preparation_step_size-correction_displacement, preparation_insertion_speed);
        ustep_device.closeFrontGripper();
        ustep_device.openBackGripper();
    end
end

preparation_time = toc(experiment_start);
insertion_start = tic;

%% Perform forward steps

if(telescoping_suport_mode)
    fprintf('\nAdjust the position of the telescoping support\n');
    input('Hit ENTER when you are ready\n');
end

fprintf('\nPreparing to start the experiment. Place the gelatin\n');
input('Hit ENTER when you are ready\n');

for i_step = 1:n_step
    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, mS = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, minimum_insertion(i_step), duty_cycle(i_step));
    
    % Verify if this step includes a 180 degrees rotation
    if(rotation_steps(i_step))
        fprintf('This steps contains a needle flip. Rotating the needle in 180 degrees\n');
        ustep_device.rotateNeedleDegrees(180, preparation_rotation_speed);
    end
    
    % Measure needle pose before moving
    ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
    ustep_device.moveDC(step_size, insertion_speed, minimum_insertion(i_step), duty_cycle(i_step));
    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, minimum_insertion(i_step), duty_cycle(i_step));
end

% Measure the final needle pose
ustep_device.savePoseForward(aurora_device, n_step+1);

insertion_time = toc(insertion_start);
removing_start = tic;

%% Perform backward steps

if(aurora_present)
    aurora_device.BEEP('3');
end
fprintf('\nNeedle insertion complete!\n');
input('Hit ENTER to start retreating the needle\n');

% Measure the final needle pose
ustep_device.savePoseBackward(aurora_device, n_step+1);

for i_step = n_step:-1:1
    fprintf('\nPerforming backward step %d/%d\n', i_step, n_step);
    
    % Move needle
    ustep_device.moveBackward(step_size, insertion_speed);
    
    % Measure needle pose before retreating
    ustep_device.savePoseBackward(aurora_device, i_step);

end

%% Save the results and close the program

if(aurora_present)    
    aurora_device.BEEP('4');
end
removing_time = toc(removing_start);
experiment_time = toc(experiment_start);

% Close the aurora system
if(aurora_present)
    aurora_device.stopTracking();
    delete(aurora_device);
end

% Save experiment results
save(sprintf('%s.mat',output_file_name));