% AA 273 | Final Project | Time rate of change of quaternion
% Jin Woo Park
% June 1, 2017

function qdot = getqdot(w,q)

% Computes the time rate of change of quaternion at time t.
%
% INPUTS:
%           w: angular velocity (ran/s) <- in principle frame for us
%           q: quaterion at t
% OUTPUTS:
%        qdot: time rate of change of quaternion

    wx = w(1);
    wy = w(2);
    wz = w(3);
    Om = [0 wz -wy wx; -wz 0 wx wy; wy -wx 0 wz; -wx -wy -wz 0];
    qdot = 0.5*Om*q;
end