function [C] = magnetometer_jacob(m,q)
%MAGNETOMETER_JACOB Creates measurement jacobian for magnetometer model.
%   

f1_q0 = -2*q(4)*m(2)+2*q(3)*m(3);
f1_q1 = 2*q(3)*m(2)+2*q(4)*m(3);
f1_q2 = -4*q(3)*m(1)+2*q(2)*m(2)+2*q(1)*m(3);
f1_q3 = -4*q(4)*m(1)-2*q(1)*m(2)+2*q(2)*m(3);

f2_q0 = 2*q(4)*m(1)-2*q(2)*m(3);
f2_q1 = 2*q(3)*m(1)-4*q(2)*m(2)-2*q(1)*m(3);
f2_q2 = 2*q(2)*m(1)+2*q(4)*m(3);
f2_q3 = 2*q(1)*m(1)-4*q(4)*m(2)+2*q(3)*m(3);

f3_q0 = -2*q(3)*m(1)+2*q(2)*m(2);
f3_q1 = 2*q(4)*m(1)+2*q(1)*m(2)-4*q(2)*m(3);
f3_q2 = -2*q(1)*m(1)+2*q(4)*m(2)-4*q(3)*m(3);
f3_q3 = 2*q(2)*m(1)+2*q(3)*m(2);

C = [f1_q0, f1_q1, f1_q2, f1_q3;
     f2_q0, f2_q1, f2_q2, f2_q3;
     f3_q0, f3_q1, f3_q2, f3_q3];
 
end

