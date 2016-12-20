function quat = quatmultiplysym(p, q)

quat(1) = p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4);
quat(2) = p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3);
quat(3) = p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2);
quat(4) = p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1);