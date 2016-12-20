close all;
clear all;
clc;

%% Global parameters

sensor_angle_inside_needle = -115.0;

% Needle initial orientation
needle_N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

n_cycle = 4;
n_step = 10;

step_size = 6.0;
insertion_speed = 1.0;

additional_steps = 0;

needle_angle = zeros(1, n_cycle);

%% Configure the TCP/IP client for communicating with the Raspberry Pi

ustep_device = UStepDeviceHandler(n_cycle);

%% Configure the Aurora sensor object

aurora_device = AuroraDriver('/dev/ttyUSB1');
aurora_device.openSerialPort();
aurora_device.init();
aurora_device.detectAndAssignPortHandles();
aurora_device.initPortHandleAll();
aurora_device.enablePortHandleDynamicAll();
aurora_device.startTracking();

%% Set file name for storing the results

fprintf('Step Repeatability Experiment\n');
output_file_name = input('Type the name of the file for saving the results\n','s');

%% Perform the initial measurement

input('Place the needle inside the device and hit ENTER to close the front gripper\n');
ustep_device.closeFrontGripper();

while(aurora_device.isSensorAvailable() == 0)
    fprintf('Cant read the needle EM sensor. Moving the needle %.2f mm forward \n', step_size);
    ustep_device.moveForward(step_size, insertion_speed);
end

aurora_device.updateSensorDataAll();
needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
needle_angle(1) = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);
 
ustep_device.savePoseForward(aurora_device, 1);

fprintf('Needle starting angle = %.2f\n', needle_angle(1));
starting_distance = input('Measure and type the needle distance to the front support: ');

%% Perform all cycles of movement

for i_cycle = 1:n_cycle
    
    fprintf('\nPerforming cycle %d/%d\n', i_cycle, n_cycle);
    
    % Moving the needle backward
    for i_step = 1:n_step
        fprintf('\nPerforming step backward %d/%d\n', i_step, n_step);
        ustep_device.moveBackward(step_size, insertion_speed);
    end
    
    % Moving the needle backward
    for i_step = 1:n_step
        fprintf('\nPerforming step forward %d/%d\n', i_step, n_step);
        ustep_device.moveForward(step_size, insertion_speed);
    end
    
    while(aurora_device.isSensorAvailable() == 0)
        fprintf('Cant read the needle EM sensor. Moving the needle %.2f mm forward \n', step_size);
        ustep_device.moveForward(step_size, insertion_speed);
        additional_steps = additional_steps + 1;
    end
    
    aurora_device.updateSensorDataAll();
    needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
    needle_angle(i_cycle+1) = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);
    
    angle_diff = needle_angle(i_cycle+1) - needle_angle(i_cycle);
    while(angle_diff > 180)
        needle_angle(i_cycle+1) = needle_angle(i_cycle+1) - 360;
        angle_diff = needle_angle(i_cycle+1) - needle_angle(i_cycle);
    end
    while(angle_diff < -180)
        needle_angle(i_cycle+1) = needle_angle(i_cycle+1) + 360;
        angle_diff = needle_angle(i_cycle+1) - needle_angle(i_cycle);
    end
    
    ustep_device.savePoseForward(aurora_device, i_cycle+1);
    
end

fprintf('Experiment finished\n')
ending_distance = input('Measure and type the needle distance to the front support again: ');


%% Print Results

needle_displacement = starting_distance - (ending_distance - additional_steps*step_size);
needle_displacement_per_step = needle_displacement / (n_cycle*n_step*2 + additional_steps);
fprintf('\nNeedle displacement error: Total = %.2f, Per Step = %.2f\n', needle_displacement, needle_displacement_per_step);

needle_angle_error = needle_angle(n_cycle+1) - needle_angle(1);
needle_angle_error_per_step = needle_angle_error / (n_cycle*n_step*2 + additional_steps);

fprintf('\nNeedle angle error: Total = %.2f, Per Step = %.2f\n', needle_angle_error, needle_angle_error_per_step);

%% Save the results and close the program

% Close the aurora system
aurora_device.stopTracking();
delete(aurora_device);

% Save experiment results
save(sprintf('%s.mat',output_file_name));