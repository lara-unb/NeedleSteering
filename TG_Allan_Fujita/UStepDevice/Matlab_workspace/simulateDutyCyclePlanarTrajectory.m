function simulateDutyCyclePlanarTrajectory(duty_cycle, rotation_steps, pre_insertion)

% Global parameters
r = 0.178;
step_size = 8;
v = 0.001;

n_duty_cycle = length(duty_cycle);
n_rotation = length(rotation_steps);

if(n_duty_cycle ~= n_rotation)
    fprintf('simulateDutyCyclePlanarTrajectory:: Error - duty cycle and rotation vector sizes mismatch\n');
else
    n_step = n_duty_cycle;
    
    curvature_sign = 1 -2*mod(cumsum(rotation_steps), 2);
    k = (1/r) * (1 - duty_cycle) .* curvature_sign;
    
    px = zeros(1, n_step*step_size + 1 + pre_insertion);
    py = zeros(1, n_step*step_size + 1 + pre_insertion);
    theta = zeros(1, n_step*step_size + 1 + pre_insertion);
    
    for i_mm = 1:pre_insertion
        
        index = i_mm;
        
        d = (2/(k(1)^2))*(1 - cos(k(1)*v));
        t = theta(index) + (k(1)*v)/2;
        
        px(index+1) = px(index) + d*cos(t);
        py(index+1) = py(index) + d*sin(t);
        theta(index+1) = theta(index) + k(1)*v;
    end
    
    
    for i_step = 1:n_step
        
        
        for i_mm = 1:step_size
            
            index = step_size*(i_step-1) + i_mm + pre_insertion;
            
            d = (2/(k(i_step)^2))*(1 - cos(k(i_step)*v));
            t = theta(index) + (k(i_step)*v)/2;
            
            px(index+1) = px(index) + d*cos(t);
            py(index+1) = py(index) + d*sin(t);
            theta(index+1) = theta(index) + k(i_step)*v;
        end
        
    end
    
    px = 10^6*px;
    py = 10^6*py;
    
    rx = range(px);
    ry = range(py);
    rmax = 1.1 * max(rx, ry);
    
    figure;
    plot(py, -px);
    xlim([-rmax/4 rmax/4]);
    ylim([-rmax 0]);
    
end




% n_step = 14;
% % duty_cycle =     [0.00 0.00 0.00 0.00 0.25 0.50 0.75 1.00 0.75 0.50 0.25 0.00 0.00 0.00];
% rotation_steps = [0    0    0    0    0    1    0    0    0    0    0    0    0    0];
% duty_cycle = zeros(1, n_step);


