function correction_angle = measureNeedleCorrectionAngle(needle_quaternion, needle_N0)

Z = [0 0 1];

needle_N = quatrotate(needle_quaternion, needle_N0);

theta = rad2deg(acos(dot(needle_N, Z)));

cross_vector = cross(needle_N, Z);

if(cross_vector(2) > 0)
    correction_angle = theta;
else
    correction_angle = -theta;
end
