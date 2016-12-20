clear all
clc

%% Define a initial orientation for vector v

v0 = [0; 0; 1]

%% Define two rotation matrices to be applied to vector v

% 90 degrees of rotation along X
rot_1 = rotx(deg2rad(90))

% 90 degress of rotation along Y
rot_2 = roty(deg2rad(90))

% Composed rotation: 90 deg on X followed by 90 deg on Y
rot_21 = rot_2 * rot_1

%% Define two quaternions to represent the same transformation

% 90 degrees of rotation along X
quat_1 = angle2quat(deg2rad(90), 0, 0, 'xyz')

% 90 degress of rotation along Y
quat_2 = angle2quat(0, deg2rad(90), 0, 'xyz')

% Composed rotation: 90 deg on X followed by 90 deg on Y
quat_21 = quatmultiply(quat_1, quat_2)

%% Apply the rotation matrices to the vector v

v1m = rot_1 * v0

v2m = rot_2 * v1m

v3m = rot_21 * v0

%% Apply the quanternions to the vector v

v1q = quatrotate(quat_1, v0')'

v2q = quatrotate(quat_2, v1q')'

v3q = quatrotate(quat_21, v0')'

% OBS: I have found a way of calculating a vector v3q from a starting
% vector v0 and a quaternion quat_21
% quat_21 represents a series of rotations 






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Trying to find the needle rotation angle around v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Find the rotation matrix between v0 and v3q

% Math from: http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

v = cross(v0, v3q)

s = norm(v)

c = dot(v0, v3q)

vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
vx2 = vx*vx;

m = eye(3) + vx + vx2*((1-c)/(s^2))

%% Convert the rotation matrix into quaternion

% Math from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

qw = sqrt(max(0, 1 + m(1,1) + m(2,2) + m(3,3) ))/2
qx = sqrt(max(0, 1 + m(1,1) - m(2,2) - m(3,3) ))/2
qy = sqrt(max(0, 1 - m(1,1) + m(2,2) - m(3,3) ))/2
qz = sqrt(max(0, 1 - m(1,1) - m(2,2) + m(3,3) ))/2

qx = qx * (2*(m(3,2) > m(2,3))-1)
qy = qy * (2*(m(1,3) > m(3,1))-1)
qz = qz * (2*(m(2,1) > m(1,2))-1)

quat = [qw qx qy qz]

%% Find the quaternion associated only with the rotation around v

quat_rot_v = quatdivide(quat_21, quat)

%% Extract the rotation angle

angle = rad2deg(acos(quat_rot_v(1)))









