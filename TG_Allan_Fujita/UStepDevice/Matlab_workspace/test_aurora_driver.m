window = aurora_demo();
handles = guihandles(window);




% % clear all
% % 
% % device = AuroraDriver('/dev/ttyUSB0');
% % device.openSerialPort();
% % 
% % rep1 = device.ECHO('Hello World!')
% % 
% % rep2 = device.APIREV()
% % 
% % rep3 = device.BEEP('1')
% % pause(1);
% % rep4 = device.BEEP('2')
% % pause(1);
% % rep5 = device.BEEP('7')
% % pause(2);
% % rep6 = device.BEEP('9')
% % 
% % % device.INIT()
% % % 
% % % % TSTART needs INIT
% % % device.TSTART('80');
% % % 
% % % % BX needs to be in Tracking mode
% % % [reply, error] = device.BX('0001');
% % 
% % 
% % device.closeSerialPort();
% % delete(device);
% % clear device
% 
% clear all
% 
% %device.setBaudRate(57600);
% 
% obj = AuroraDriver('/dev/ttyUSB0');
% obj.openSerialPort();
% obj.init();
% obj.detectAndAssignPortHandles();
% obj.initPortHandleAll();
% obj.enablePortHandleDynamicAll();
% obj.startTracking();
% 
% 
% obj.updateSensorDataAll();
%   
% obj.stopTracking();
% 
% delete(obj);
% clear obj