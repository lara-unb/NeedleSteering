close all;
clear all;
clc;

% Result files for the Micro Step mode
file{1} = 'Results_150824_DC10_04.mat';
file{2} = 'Results_150824_DC08_01.mat';
file{3} = 'Results_150824_DC11_01.mat';
file{4} = 'Results_150824_DC12_01.mat';
file{5} = 'Results_150825_DC13_01.mat';
file{6} = 'Results_150825_DC14_01.mat';
file{7} = 'Results_150825_DC15_01.mat';
file{8} = 'Results_150825_DC16_01.mat';
file{9} = 'Results_150825_DC17_01.mat';
file{10} = 'Results_150825_DC18_01.mat';
file{11} = 'Results_150825_DC19_01.mat';
file{12} = 'Results_150826_DC20_01.mat';
n_file = length(file);
n_exp = n_file;

dc_request = zeros(1, n_file);
dc_true = zeros(1, n_file);
radius = zeros(1, n_file);
curvature = zeros(1, n_file);

needle_tip_angle = zeros(1, n_file);

radius_0 = 0;

for i_exp = 1:n_exp
    load(file{i_exp});
    processExperimentResults;
    dc_request(i_exp) = duty_cycle;
    dc_true(i_exp) = duty_cycle_real;
    radius(i_exp) = experiment_radius;
    curvature(i_exp) = 1/experiment_radius;
    
    needle_tip_angle(i_exp) = mean([needle_correction_angle_fw(end) needle_correction_angle_bw(end)]);
    
    if(duty_cycle == 0)
        radius_0 = experiment_radius;
    end
end

dc_obtained = zeros(1, n_file);

for i_exp = 1:n_exp
    dc_obtained(i_exp) = 1 - radius_0 / radius(i_exp);
    if(dc_request(i_exp) == 1)
        dc_obtained(i_exp) = 2 - dc_obtained(i_exp);
    end
end


% Sort all vectors

dc_request_sorted = zeros(1, n_file);
dc_true_sorted = zeros(1, n_file);
radius_sorted = zeros(1, n_file);
curvature_sorted = zeros(1, n_file);
dc_obtained_sorted = zeros(1, n_file);

for i_exp = 1:n_exp
    [~, c] = find(dc_request == min(dc_request));
    min_index = c(1);
    
    dc_request_sorted(i_exp) = dc_request(min_index);
    dc_true_sorted(i_exp) = dc_true(min_index);
    radius_sorted(i_exp) = radius(min_index);
    curvature_sorted(i_exp) = curvature(min_index);
    dc_obtained_sorted(i_exp) = dc_obtained(min_index);
    
    dc_request(min_index) = 9;
end

p = polyfit(dc_true_sorted, dc_obtained_sorted, 1);

figure
plot(dc_request_sorted, curvature_sorted, 'b*');

figure
plot(dc_true_sorted, dc_obtained_sorted, 'b*');
hold on;
plot([0 1.1], p(2) + p(1)*[0 1.1], 'r-');
plot([0 1.1], [0 1.1], 'g-');