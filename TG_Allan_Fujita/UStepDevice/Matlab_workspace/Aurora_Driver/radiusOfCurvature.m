function radius = radiusOfCurvature(trans1, trans2, rot1, rot2)

deltaTrans = trans2 - trans1;
deltaTrans2 = deltaTrans .^ 2;
distance = sqrt(sum(deltaTrans2));

angle = differenceAngle(rot1, rot2);

radius = distance / (2 * sin(angle/2));