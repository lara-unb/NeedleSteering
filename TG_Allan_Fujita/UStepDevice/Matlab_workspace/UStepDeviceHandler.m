classdef UStepDeviceHandler < handle
    
    %
    %   This class is just a wrapper for communicating with the UStepDevice
    %   via TCP/IP
    %
    
    % Author: André Augusto Geraldes
    % Email: andregeraldes@lara.unb.br
    % August 2015; Last revision:
    
    % Constants
    properties (Constant)
        
        % TCP/IP parameters
        server_address = '169.254.0.2';
        server_port = 5555;
        tcpip_buffer_size = 7688;
        tcpip_timeout = 180;
        
        %
        pose_saving_delay = 1.0;
        
        % Device mode
        MODE_MICROSTEP           = 1;
        MODE_TELESCOPING_SUPPORT = 2;
        
        % Methods for duty cycling
        DC_METHOD_BIDIRECTIONAL = 1;
        DC_METHOD_FLIPPING      = 2;
        
        % Communication protocol table (taken from the C++ code)
        CMD_OPEN_FRONT_GRIPPER	  = 1;
        CMD_CLOSE_FRONT_GRIPPER	  = 2;
        CMD_OPEN_BACK_GRIPPER	  = 3;
        CMD_CLOSE_BACK_GRIPPER	  = 4;
        CMD_ROTATE                = 5;
        CMD_TRANSLATE             = 6;
        CMD_MOVE_FORWARD          = 7;
        CMD_MOVE_BACKWARD         = 8;
        CMD_MOVE_DC_BIDIRECTIONAL = 9;
        CMD_MOVE_DC_FLIPPING      = 10;
        
        CMD_MOVE_DC_FLIPPING_PART1 = 11;
        CMD_MOVE_DC_FLIPPING_PART2 = 12;
        
        CMD_DONE                  = 42;
        CMD_SHUT_DOWN             = 255;
        
        CMD_MOVE_MOTOR            = 101;  % DISABLED COMMAND
        CMD_MOVE_MOTOR_STEPS      = 102;  % DISABLED COMMAND
        CMD_SET_DIRECTION         = 103;  % DISABLED COMMAND
        CMD_SET_ENABLE            = 104;  % DISABLED COMMAND
        
        % Low level communication table (taken from the C++ code)
        % Used only by the disabled commands
        MOTOR_INSERTION             = 1;
        MOTOR_ROTATION              = 2;
        MOTOR_FRONT_GRIPPER         = 3;
        MOTOR_BACK_GRIPPER          = 4;
        
        DIRECTION_FORWARD           = 0;
        DIRECTION_BACKWARD          = 1;
        DIRECTION_CLOCKWISE         = 0;
        DIRECTION_COUNTER_CLOCKWISE = 1;
        DIRECTION_OPENING           = 1;
        DIRECTION_CLOSING           = 0;
        
        ENABLE_MOTOR                = 0;
        DISABLE_MOTOR               = 1;
        
    end
    
    properties (GetAccess = public, SetAccess = private)
        
        % TCP/IP client object
        tcpip_client;       
        
        % Number of steps (used to pre-allocate the pose and command vectors)
        n_step;             
        
        % Needle poses measured when moving forward and backward
        % These vectors should have n_step+1 elements for storing initial
        % and final pose
        needle_pose_fw;     
        needle_pose_bw; 
        
        % Set of commands sent for each DC motion - should have n_step
        % elements
        command_sent;       
        
        % Rotation angle in CW direction applied to the needle tip between
        % each pair of steps - should have n_step-1 elements
        interstep_rotation; 
        
        % Device assembly mode (set in constructor)
        device_mode;
        
        % Selected method for duty cycling (set in constructor)
        duty_cycle_method;

    end
    
    
    % Public methods
    methods (Access = public)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %             CONSTRUCTOR              %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = UStepDeviceHandler(n_step)
            
            % Device assembly mode
            obj.device_mode = obj.MODE_MICROSTEP;
            
            % Standard method selected for Duty Cyclying
            obj.duty_cycle_method = obj.DC_METHOD_FLIPPING;
            
            % Configure the TCP/IP client object 
            obj.tcpip_client = tcpip(obj.server_address, obj.server_port,'NetworkRole','Client');
            set(obj.tcpip_client,'InputBufferSize', obj.tcpip_buffer_size);
            set(obj.tcpip_client,'Timeout', obj.tcpip_timeout);
            
            % Pre-alocate the vectors for storing the needle pose
            obj.n_step = n_step;

            pose_fw(1, n_step+1) = PoseMeasurement();
            pose_bw(1, n_step+1) = PoseMeasurement();
            obj.needle_pose_fw = pose_fw;
            obj.needle_pose_bw = pose_bw;          
              
            % Pre-alocate the vectors for storing all the DCcommands sent
            % to the device
            obj.interstep_rotation = zeros(1, n_step-1);
            command(1, n_step) = DCCommand();
            obj.command_sent = command;
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %    FUNCTIONS FOR SENDING COMMANDS    %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function openFrontGripper(obj)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, obj.CMD_OPEN_FRONT_GRIPPER);
            fclose(obj.tcpip_client);
        end
        
        function closeFrontGripper(obj)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, obj.CMD_CLOSE_FRONT_GRIPPER);
            fclose(obj.tcpip_client);
        end
        
        function openBackGripper(obj)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, obj.CMD_OPEN_BACK_GRIPPER);
            fclose(obj.tcpip_client);
        end
        
        function closeBackGripper(obj)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, obj.CMD_CLOSE_BACK_GRIPPER);
            fclose(obj.tcpip_client);
        end        
        
        function rotateNeedleDegrees(obj, angle, rotation_speed)
            revolutions = angle / 360.0;
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, [obj.CMD_ROTATE typecast(revolutions, 'uint8') typecast(rotation_speed, 'uint8')]);
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);
        end        
        
        function translateFrontGripper(obj, displacement, speed)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, [obj.CMD_TRANSLATE typecast(displacement, 'uint8') typecast(speed, 'uint8')]);
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);
        end
        
        function moveForward(obj, distance, speed)
            fopen(obj.tcpip_client);
            if(obj.device_mode == obj.MODE_MICROSTEP)
                fwrite(obj.tcpip_client, [obj.CMD_MOVE_FORWARD typecast(distance, 'uint8') typecast(speed, 'uint8')]);
            else
                fwrite(obj.tcpip_client, [obj.CMD_TRANSLATE typecast(distance, 'uint8') typecast(speed, 'uint8')]);
            end
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);
        end
           
        function moveBackward(obj, distance, speed)
            fopen(obj.tcpip_client);
            if(obj.device_mode == obj.MODE_MICROSTEP)
                fwrite(obj.tcpip_client, [obj.CMD_MOVE_BACKWARD typecast(distance, 'uint8') typecast(speed, 'uint8')]);
            else
                fwrite(obj.tcpip_client, [obj.CMD_TRANSLATE typecast(-distance, 'uint8') typecast(speed, 'uint8')]);
            end
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);
        end
        
        function moveDC(obj, step_size, insertion_speed, third_parameter, duty_cycle)
            fopen(obj.tcpip_client);
            if(obj.device_mode == obj.MODE_MICROSTEP)
                if(obj.duty_cycle_method == obj.DC_METHOD_BIDIRECTIONAL)
                    rotation_speed = third_parameter;
                    fwrite(obj.tcpip_client, [obj.CMD_MOVE_DC_BIDIRECTIONAL typecast(step_size, 'uint8') typecast(insertion_speed, 'uint8') typecast(rotation_speed, 'uint8') typecast(duty_cycle, 'uint8')]);
                else
                    minimum_insertion = third_parameter;
                    fwrite(obj.tcpip_client, [obj.CMD_MOVE_DC_FLIPPING typecast(step_size, 'uint8') typecast(insertion_speed, 'uint8') typecast(minimum_insertion, 'uint8') typecast(duty_cycle, 'uint8')]);
                end
            else
                if(obj.duty_cycle_method == obj.DC_METHOD_BIDIRECTIONAL)
                    fprintf('ERROR: Bidirectional duty cycle is not implemented on the Telescoping support mode!!!\n');
                else
                    minimum_insertion = third_parameter;
                    fwrite(obj.tcpip_client, [obj.CMD_MOVE_DC_FLIPPING_PART2 typecast(step_size, 'uint8') typecast(insertion_speed, 'uint8') typecast(minimum_insertion, 'uint8') typecast(duty_cycle, 'uint8')]);
                end
            end
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);
        end
        
        function moveDCPart1(obj, step_size, insertion_speed, minimum_insertion, duty_cycle)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, [obj.CMD_MOVE_DC_FLIPPING_PART1 typecast(step_size, 'uint8') typecast(insertion_speed, 'uint8') typecast(minimum_insertion, 'uint8') typecast(duty_cycle, 'uint8')]);
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);            
        end
        
        function moveDCPart2(obj, step_size, insertion_speed, minimum_insertion, duty_cycle)
            fopen(obj.tcpip_client);
            fwrite(obj.tcpip_client, [obj.CMD_MOVE_DC_FLIPPING_PART2 typecast(step_size, 'uint8') typecast(insertion_speed, 'uint8') typecast(minimum_insertion, 'uint8') typecast(duty_cycle, 'uint8')]);
            fread(obj.tcpip_client, 1);
            fclose(obj.tcpip_client);            
        end        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % FUNCTIONS FOR SAVING THE NEEDLE POSE %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
        function savePoseForward(obj, aurora_device, i_step)
            pause(obj.pose_saving_delay);
            aurora_device.updateSensorDataAll();
            if(aurora_device.isSensorAvailable())
                trans = aurora_device.port_handles(1,1).trans;
                rot = quatinv(aurora_device.port_handles(1,1).rot);
                error = aurora_device.port_handles(1,1).error;
                
                obj.needle_pose_fw(i_step).x = trans(1);
                obj.needle_pose_fw(i_step).y = trans(2);
                obj.needle_pose_fw(i_step).z = trans(3);
                obj.needle_pose_fw(i_step).orientation = rot;
                obj.needle_pose_fw(i_step).error = error;
            end  
        end
        
        function savePoseBackward(obj, aurora_device, i_step)
            pause(obj.pose_saving_delay);
            aurora_device.updateSensorDataAll();
            if(aurora_device.isSensorAvailable())
                trans = aurora_device.port_handles(1,1).trans;
                rot = quatinv(aurora_device.port_handles(1,1).rot);
                error = aurora_device.port_handles(1,1).error;
                
                obj.needle_pose_bw(i_step).x = trans(1);
                obj.needle_pose_bw(i_step).y = trans(2);
                obj.needle_pose_bw(i_step).z = trans(3);
                obj.needle_pose_bw(i_step).orientation = rot;
                obj.needle_pose_bw(i_step).error = error;
            end
        end
        
        function saveCommandsDC(obj, i_step, step_size, insertion_speed, rotation_speed, duty_cycle)
            obj.command_sent(i_step).step_size = step_size;
            obj.command_sent(i_step).insertion_speed = insertion_speed;
            obj.command_sent(i_step).rotation_speed = rotation_speed;
            obj.command_sent(i_step).duty_cycle = duty_cycle;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      FUNCTIONS FOR PLOTING DATA      %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        function [x_fw, x_bw] = plotNeedleX(obj)
            N = obj.n_step+1;
            x_fw = zeros(1, N);
            x_bw = zeros(1, N);
            for i_step = 1:N
                x_fw(i_step) = obj.needle_pose_fw(i_step).x;
                x_bw(i_step) = obj.needle_pose_bw(i_step).x;
            end
            
            hold on;
            [~, c] = find(x_bw ~= 0);
            index = c(1);
            plot(index:N, x_fw(index:end), 'r-');
            plot(index:N, x_bw(index:end), 'b-');
        end
        
        function [y_fw, y_bw] = plotNeedleY(obj)
            N = obj.n_step+1;
            y_fw = zeros(1, N);
            y_bw = zeros(1, N);
            for i_step = 1:N
                y_fw(i_step) = obj.needle_pose_fw(i_step).y;
                y_bw(i_step) = obj.needle_pose_bw(i_step).y;
            end
            
            hold on;
            [~, c] = find(y_bw ~= 0);
            index = c(1);
            plot(index:N, y_fw(index:end), 'r-');
            plot(index:N, y_bw(index:end), 'b-');
        end
        
       function [z_fw, z_bw] = plotNeedleZ(obj)
            N = obj.n_step+1;
            z_fw = zeros(1, N);
            z_bw = zeros(1, N);
            for i_step = 1:N
                z_fw(i_step) = obj.needle_pose_fw(i_step).z;
                z_bw(i_step) = obj.needle_pose_bw(i_step).z;
            end
            
            hold on;
            [~, c] = find(z_bw ~= 0);
            index = c(1);
            plot(index:N, z_fw(index:end), 'r-');
            plot(index:N, z_bw(index:end), 'b-');
        end        
        
        
    end
    
end