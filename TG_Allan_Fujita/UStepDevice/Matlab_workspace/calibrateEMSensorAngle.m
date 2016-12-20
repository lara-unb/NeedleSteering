close all;
clear all;
clc;

%% Global parameters

sensor_angle_inside_needle = 49.0;

% Needle initial orientation
needle_V0 = [0 0 1];
needle_N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

preparation_step_size = 20;
preparation_insertion_speed = 4.0;
preparation_rotation_speed = 0.1;

n_step = 6;
step_size = 15;
insertion_speed = 2.0;

%% Configure the TCP/IP client for communicating with the Raspberry Pi

ustep_device = UStepDeviceHandler(n_step);

%% Configure the Aurora sensor object

aurora_device = AuroraDriver('/dev/ttyUSB0');
aurora_device.openSerialPort();
aurora_device.init();
aurora_device.detectAndAssignPortHandles();
aurora_device.initPortHandleAll();
aurora_device.enablePortHandleDynamicAll();
aurora_device.startTracking();

%% Adjust the needle starting position

fprintf('Place the needle inside the device\n');
fprintf('Make sure to align the needle tip the end of the device\n');
input('When you are done, hit ENTER to close the front gripper\n\n');

% Moving the needle forward until it gets detected by the Aurora system
fprintf('Adjusting the needle initial orientation\n');
n_preparation_step = 0;
while(aurora_error.isSensorAvailable() == 0)
    
    fprintf('Cant read the needle EM sensor. Moving the needle %.2f mm forward \n', preparation_step_size);
    ustep_device.moveForward(preparation_step_size, preparation_insertion_speed);
    n_preparation_step = n_preparation_step + 1;
end

% Adjusting the needle orientation
aurora_device.updateSensorDataAll();
needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
correction_angle = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);

fprintf('Needle found! Correction angle = %.2f degrees \n', correction_angle);
ustep_device.rotateNeedleDegrees(correction_angle, preparation_rotation_speed);

% Moving the needle backward
for i_preparation_step = 1:n_preparation_step
    ustep_device.moveBackward(preparation_step_size, preparation_insertion_speed);
end

%% Perform the open loop trajectory

fprintf('\nPreparing to start the experiment. Place the gelatin\n');
input('Hit ENTER when you are ready\n');

for i_step = 1:n_step
    fprintf('\nPerforming step %d/%d: inserting %.2f mm\n', i_step, n_step, step_size);
    
    % Measure needle pose before moving
    ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
    ustep_device.moveForward(step_size, insertion_speed);
    
end

% Measure the final needle pose
ustep_device.savePoseForward(aurora_device, n_step+1);

%% Display the results

% Extract the values of X, Y and Z from the device object
figure;
[x_fw, ~] = ustep_device.plotNeedleX();
[y_fw, ~] = ustep_device.plotNeedleY();
[z_fw, ~] = ustep_device.plotNeedleZ();

subplot(3,1,1); plot(x_fw);
subplot(3,1,2); plot(y_fw);
subplot(3,1,3); plot(z_fw);

aurora_device.stopTracking();
delete(aurora_device);