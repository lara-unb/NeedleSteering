function [radius, arc_length] = measureNeedleCurvature(pose1, pose2, step_size)

if(quatnorm(pose1.orientation) == 0 || quatnorm(pose1.orientation) == 0)
    
    radius = 0;
    arc_length = 0;
    
else
    
    dx = pose1.x - pose2.x;
    dy = pose1.y - pose2.y;
    dz = pose1.z - pose2.z;
    
    distance = sqrt(dx^2 + dy^2 + dz^2);
    
    v0 = [0 0 1];
    v1 = quatrotate(pose1.orientation, v0);
    v2 = quatrotate(pose2.orientation, v0);
    
    theta = acos(dot(v1, v2));
%     theta_deg = rad2deg(theta);

%% Method 1: use the aurora measurements for angle and distance
    
    radius = distance / (2*sin(theta/2));
    arc_length = radius * theta;
    
%% Method 2: use the aurora measurements only for angle

%     arc_length = step_size;
%     radius = arc_length / theta;
    
end