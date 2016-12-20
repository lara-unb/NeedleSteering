function varargout = aurora_demo(varargin)
% AURORA_DEMO MATLAB code for aurora_demo.fig
%      AURORA_DEMO, by itself, creates a new AURORA_DEMO or raises the existing
%      singleton*.
%
%      H = AURORA_DEMO returns the handle to a new AURORA_DEMO or the handle to
%      the existing singleton*.
%
%      AURORA_DEMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AURORA_DEMO.M with the given input arguments.
%
%      AURORA_DEMO('Property','Value',...) creates a new AURORA_DEMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before aurora_demo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to aurora_demo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help aurora_demo

% Last Modified by GUIDE v2.5 19-Aug-2015 21:16:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @aurora_demo_OpeningFcn, ...
    'gui_OutputFcn',  @aurora_demo_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before aurora_demo is made visible.
function aurora_demo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to aurora_demo (see VARARGIN)

% Choose default command line output for ex_guide_timergui
handles.output = hObject;

% START USER CODE
% Create a timer object to fire at 1/10 sec intervals
% Specify function handles for its start and run callbacks

global device sensor_missing out_volume partial_out_volume

global saved_trans saved_rot i_saved

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.2, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback function

x = 0;
y = 0;
z = 350;

axes(handles.axes_XY);
handles.plot_XY = plot(x, y, 'bo');
xlim([-350 350]);
ylim([-250 250]);

axes(handles.axes_XZ);
handles.plot_XZ = plot(x, z, 'bo');
xlim([-350 350]);
ylim([100 600]);

axes(handles.axes_YZ);
handles.plot_YZ = plot(y, z, 'bo');
xlim([-250 250]);
ylim([100 600]);

hideTrackingData(handles);

[RX RY RZ] = quat2angle([1 0 1 0]);

i_saved = 0;



pause(0.5);
device = AuroraDriver('/dev/ttyUSB0');
device.openSerialPort();
device.init();
device.detectAndAssignPortHandles();
device.initPortHandleAll();
device.enablePortHandleDynamicAll();

sensor_missing = 0;
out_volume = 0;
partially_out_volume = 0;

% END USER CODE

% Update handles structure
guidata(hObject,handles);

% --- Outputs from this function are returned to the command line.
function varargout = aurora_demo_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
x_pos = 0.1;
y_pos = 0.1;
set(gcf, 'units','normalized','outerposition',[x_pos y_pos 1-x_pos 1-y_pos]);


% --- Executes on button press in pushbutton_start.
function pushbutton_start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global device

% START USER CODE
% Only start timer if it is not running
if strcmp(get(handles.timer, 'Running'), 'off')
    device.startTracking();
    start(handles.timer);
    set(handles.text_tracking_mode, 'String', 'Tracking ON');
end
% END USER CODE

% --- Executes on button press in pushbutton_stop.
function pushbutton_stop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global device

% START USER CODE
% Only stop timer if it is running
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
    device.stopTracking();
    hideTrackingData(handles);
end
% END USER CODE

% START USER CODE
function update_display(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.

global device sensor_missing out_volume partial_out_volume

handles = guidata(hfigure);

device.updateSensorDataAll();

status = device.port_handles(1,1).sensor_status;

if(device.port_handles(1,1).out_of_volume)
    if(~out_volume)
        set(handles.text_out_volume, 'String', 'Out of Volume');
        out_volume = 1;
    end
else
    if(out_volume)
        set(handles.text_out_volume, 'String', '');
        out_volume = 0;
    end
end

if(device.port_handles(1,1).partial_out_of_volume)
    if(~partial_out_volume)
        set(handles.text_out_volume, 'String', 'Partially Out of Volume');
        partial_out_volume = 1;
    end
else
    if(partial_out_volume)
        set(handles.text_out_volume, 'String', '');
        partial_out_volume = 0;
    end
end


if(strcmp(status, '02') || strcmp(status, '04'))
    if(~sensor_missing)
        sensor_missing = 1;
        set(handles.plot_XY, 'MarkerEdgeColor', 'red');
        set(handles.plot_XZ, 'MarkerEdgeColor', 'red');
        set(handles.plot_YZ, 'MarkerEdgeColor', 'red');
        set(handles.text_sensor_status, 'String', 'Sensor Missing');
        set(handles.text_sensor_status, 'ForegroundColor', 'red');
        
        set(handles.text_Tx, 'String', 'Tx = ???');
        set(handles.text_Ty, 'String', 'Ty = ???');
        set(handles.text_Tz, 'String', 'Tz = ???');
        set(handles.text_Rx, 'String', 'Rx = ???');
        set(handles.text_Ry, 'String', 'Ry = ???');
        set(handles.text_Rz, 'String', 'Rz = ???');
        set(handles.text_error, 'String', 'error = ???');
    end
else
    if(sensor_missing)
        sensor_missing = 0;
        set(handles.plot_XY, 'MarkerEdgeColor', 'black');
        set(handles.plot_XZ, 'MarkerEdgeColor', 'black');
        set(handles.plot_YZ, 'MarkerEdgeColor', 'black');
        set(handles.text_sensor_status, 'String', 'Sensor Valid');
        set(handles.text_sensor_status, 'ForegroundColor', 'black');
    end
    
    trans = device.port_handles(1,1).trans;
    X = trans(1);
    Y = trans(2);
    Z = trans(3);
    
    rot = device.port_handles(1,1).rot;
    [RX, RY, RZ] = quat2angle(rot);
    
    error = 100*device.port_handles(1,1).error;
    
    set(handles.plot_XY, 'XData', X);
    set(handles.plot_XY, 'YData', Y);
    
    set(handles.plot_XZ, 'XData', X);
    set(handles.plot_XZ, 'YData', Z);
    
    set(handles.plot_YZ, 'XData', Y);
    set(handles.plot_YZ, 'YData', Z);
    
    
    set(handles.text_Tx, 'String', sprintf('Tx = %.2f', X));
    set(handles.text_Ty, 'String', sprintf('Ty = %.2f', Y));
    set(handles.text_Tz, 'String', sprintf('Tz = %.2f', Z));
%     set(handles.text_Rx, 'String', sprintf('Rx = %.2f', rad2deg(RX)));
%     set(handles.text_Ry, 'String', sprintf('Ry = %.2f', rad2deg(RY)));
%     set(handles.text_Rz, 'String', sprintf('Rz = %.2f', rad2deg(RZ)));
    set(handles.text_error, 'String', sprintf('error = %.2f', error));

    % DEBUG
    
    rot = quatinv(rot);
%     X0 = [1 0 0];
%     Y0 = [0 1 0];
%     Z0 = [0 0 1];
%     X1 = quatrotate(rot, X0);
%     Y1 = quatrotate(rot, Y0);
%     Z1 = quatrotate(rot, Z0);
%     set(handles.text_Rx, 'String', sprintf('X1 = (%.2f, %.2f, %.2f)', X1(1), X1(2), X1(3)));
%     set(handles.text_Ry, 'String', sprintf('Y1 = (%.2f, %.2f, %.2f)', Y1(1), Y1(2), Y1(3)));
%     set(handles.text_Rz, 'String', sprintf('Z1 = (%.2f, %.2f, %.2f)', Z1(1), Z1(2), Z1(3)));
    
    sensor_angle_inside_needle = 28.0;
    V0 = [0 0 1];
    N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

    V1 = quatrotate(rot, V0);
    N1 = quatrotate(rot, N0);
    correction_angle = measureNeedleCorrectionAngle(rot, N0);
    set(handles.text_Rx, 'String', sprintf('X1 = (%.2f, %.2f, %.2f)', V1(1), V1(2), V1(3)));
    set(handles.text_Ry, 'String', sprintf('Y1 = (%.2f, %.2f, %.2f)', N1(1), N1(2), N1(3)));
    set(handles.text_Rz, 'String', sprintf('C. Angle = %.2f', correction_angle));

    % END DEBUG
    
end

function hideTrackingData(handles)

set(handles.text_tracking_mode, 'String', 'Tracking OFF');
set(handles.text_sensor_status, 'String', '');
set(handles.text_Tx, 'String', '');
set(handles.text_Ty, 'String', '');
set(handles.text_Tz, 'String', '');
set(handles.text_Rx, 'String', '');
set(handles.text_Ry, 'String', '');
set(handles.text_Rz, 'String', '');
set(handles.text_out_volume, 'String', '');
set(handles.text_part_out_volume, 'String', '');

% END USER CODE


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global device;

global saved_trans saved_rot

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.
try
    if strcmp(get(handles.timer, 'Running'), 'on')
        stop(handles.timer);
        device.stopTracking();
        set(handles.text_sensor_status, 'String', '');
    end
    % Destroy timer
    delete(handles.timer)
    delete(device);
    % END USER CODE
catch
    fprintf('ERROR DELETING THE TIMER\n');
end

assignin('base', 'global_trans', saved_trans);
assignin('base', 'global_rot', saved_rot);

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on button press in pushbutton_save_pos.
function pushbutton_save_pos_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global device
global saved_trans saved_rot i_saved

i_saved = i_saved + 1;

saved_trans(i_saved, 1:3) = device.port_handles(1,1).trans
saved_rot(i_saved, 1:4) = device.port_handles(1,1).rot
