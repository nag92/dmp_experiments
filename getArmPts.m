function [arm_pts] = getArmPts(x_ee, y_ee, z_ee)
    [q1, q2, q3] = IK(x_ee, y_ee, z_ee, 1); % elbow up
    arm_pts = FK(q1, q2, q3);
end

% returns all 4 points necessary to draw an RRR arm
function [arm_pts] = FK(q1, q2, q3)
    l1 = 4; l2 = 4; l3 = 4;
    p0 = [0, 0, 0];
    p1 = [0, 0, l1];
    p2 = [l2*cos(q2)*cos(q1), ...
          l2*cos(q2)*sin(q1), ...
          l1 + l2*sin(q2)];
      
    p3 = [(l2*cos(q2) + l3*cos(q2 + q3))*cos(q1), ...
          (l2*cos(q2) + l3*cos(q2 + q3))*sin(q1), ...
          l1 + l2*sin(q2) + l3*sin(q2 + q3)];
      
    arm_pts = [p0; p1; p2; p3];
end

% IK for an RRR arm
function [q1, q2, q3] = IK(x,y,z,elbow_dir)
    l1 = 4; l2 = 4; l3 = 4;
    q3 = elbow_dir * acos((x*x + y*y + (z - l1)^2 - l2*l2 - l3*l3) / (2*l2*l3));
    q2 = atan2 (z - l1 , sqrt(x*x + y*y)) - atan2((l3*sin(q3)), (l2 + l3*cos(q3)));
    q1 = atan2 (y , x);
end
