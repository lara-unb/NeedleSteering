function theta = differenceAngle(quat1, quat2)

inner_product = sum(quat1 .* quat2);
theta = acos(2*(inner_product^2) - 1);