close all;
clear all;
% clc;

%% Global parameters

% Result files for the Micro Step mode
MSD_file{1} = 'Results_150826_OL02_01.mat';
MSD_file{2} = 'Results_150826_OL02_02.mat';
MSD_file{3} = 'Results_150826_OL02_04.mat';
MSD_file{4} = 'Results_150826_OL02_05.mat';
MSD_file{5} = 'Results_150826_OL02_06.mat';
n_MSD_file = length(MSD_file);

% Result files for the 
TSD_file{1} = 'Results_150830_TSD01_01.mat';
TSD_file{2} = 'Results_150830_TSD01_02.mat';
TSD_file{3} = 'Results_150830_TSD01_03.mat';
TSD_file{4} = 'Results_150830_TSD01_04.mat';
TSD_file{5} = 'Results_150830_TSD01_05.mat';
n_TSD_file = length(TSD_file);

MSD_error_x = zeros(1, n_MSD_file);
MSD_error_y = zeros(1, n_MSD_file);
MSD_error_z = zeros(1, n_MSD_file);
MSD_error_total = zeros(1, n_MSD_file);
MSD_delta_z = zeros(1, n_MSD_file);
MSD_delta_theta = zeros(1, n_MSD_file);

for i_MSD_file = 1:n_MSD_file
    clear angle
    load(MSD_file{i_MSD_file});
    analyzeOpenLoopExperimentResults;
    
    MSD_error_x(i_MSD_file) = error_x;
    MSD_error_y(i_MSD_file) = error_y;
    MSD_error_z(i_MSD_file) = error_z;
    MSD_error_total(i_MSD_file) = error_total;
    
    MSD_delta_z(i_MSD_file) = delta_z;
    MSD_delta_theta(i_MSD_file) = delta_theta;
end

TSD_error_x = zeros(1, n_TSD_file);
TSD_error_y = zeros(1, n_TSD_file);
TSD_error_z = zeros(1, n_TSD_file);
TSD_error_total = zeros(1, n_TSD_file);
TSD_delta_z = zeros(1, n_TSD_file);
TSD_delta_theta = zeros(1, n_TSD_file);

for i_TSD_file = 1:n_TSD_file
    clear angle
    load(TSD_file{i_TSD_file});
    analyzeOpenLoopExperimentResults;
    
    TSD_error_x(i_TSD_file) = error_x;
    TSD_error_y(i_TSD_file) = error_y;
    TSD_error_z(i_TSD_file) = error_z;
    TSD_error_total(i_TSD_file) = error_total;
    
    TSD_delta_z(i_TSD_file) = delta_z;
    TSD_delta_theta(i_TSD_file) = delta_theta;
end

TSD_delta_theta(1) = TSD_delta_theta(1) + 360;
MSD_delta_theta(2) = -5.92;
MSD_delta_theta(4) = -3.28;
MSD_delta_theta(5) = 5.43;

%% Calculate the means and standard deviations

MSD_error_x_mean = mean(MSD_error_x);
MSD_error_y_mean = mean(MSD_error_y);
MSD_error_z_mean = mean(MSD_error_z);
MSD_error_total_mean = mean(MSD_error_total);
MSD_delta_theta_mean = mean(MSD_delta_theta);

MSD_error_x_std = std(MSD_error_x);
MSD_error_y_std = std(MSD_error_y);
MSD_error_z_std = std(MSD_error_z);
MSD_error_total_std = std(MSD_error_total);
MSD_delta_theta_std = std(MSD_delta_theta);

TSD_error_x_mean = mean(TSD_error_x);
TSD_error_y_mean = mean(TSD_error_y);
TSD_error_z_mean = mean(TSD_error_z);
TSD_error_total_mean = mean(TSD_error_total);
TSD_delta_theta_mean = mean(TSD_delta_theta);

TSD_error_x_std = std(TSD_error_x);
TSD_error_y_std = std(TSD_error_y);
TSD_error_z_std = std(TSD_error_z);
TSD_error_total_std = std(TSD_error_total);
TSD_delta_theta_std = std(TSD_delta_theta);

%% Display the results in a table forma

fprintf('MSD FINAL RESULTS\n\n');
fprintf('\terror-X \terror-Y \terror-Z \terror-Total \tDTheta\n');
for i = 1:n_MSD_file
    fprintf('%d:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', i, MSD_error_x(i), MSD_error_y(i), MSD_error_z(i), MSD_error_total(i), MSD_delta_theta(i));
end
fprintf('mean:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', MSD_error_x_mean, MSD_error_y_mean, MSD_error_z_mean, MSD_error_total_mean, MSD_delta_theta_mean);
fprintf('std:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', MSD_error_x_std, MSD_error_y_std, MSD_error_z_std, MSD_error_total_std, MSD_delta_theta_std);
fprintf('\n\n\n');

fprintf('TSD FINAL RESULTS\n\n');
fprintf('\terror-X \terror-Y \terror-Z \terror-Total \tDTheta\n');
for i = 1:n_TSD_file
    fprintf('%d:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', i, TSD_error_x(i), TSD_error_y(i), TSD_error_z(i), TSD_error_total(i), TSD_delta_theta(i));
end
fprintf('mean:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', TSD_error_x_mean, TSD_error_y_mean, TSD_error_z_mean, TSD_error_total_mean, TSD_delta_theta_mean);
fprintf('std:\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.1f\n', TSD_error_x_std, TSD_error_y_std, TSD_error_z_std, TSD_error_total_std, TSD_delta_theta_std);
