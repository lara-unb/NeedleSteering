disp('Robotics, Vision & Control: (c) Peter Corke 1992-2011 http://www.petercorke.com')
tb = false;
rvcpath = fileparts( mfilename('fullpath') );

robotpath = fullfile(rvcpath, 'robot');
if exist(robotpath,'dir')
    addpath(robotpath);
    tb = true;
    startup_rtb
end

visionpath = fullfile(rvcpath, 'vision');
if exist(visionpath,'dir')
    addpath(visionpath);
    tb = true;
    startup_mvtb
end

if tb
    addpath(fullfile(rvcpath, 'vgg_multiview'));
%     addpath(fullfile(rvcpath, 'vgg_multiview/private'));
    addpath(fullfile(rvcpath, 'vgg_general'));
    addpath(fullfile(rvcpath, 'vgg_numerics'));
    addpath(fullfile(rvcpath, 'vgg_ui'));
    addpath(fullfile(rvcpath, 'common'));
    addpath(fullfile(rvcpath, 'simulink'));
end

clear tb rvcpath robotpath visionpath 
