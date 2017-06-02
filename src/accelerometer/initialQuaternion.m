% AA 273 | Final Project | Initial Quaternion Computation
% Jin Woo Park
% June 1, 2017

function q0 = initialQuaternion (phi_1, theta_2, psi_3)

% Computes the initial quaterion at t = 0 where DCM is 3-2-1 rotation.
%
% INPUTS:
%       phi_1: angle about x-axis (in deg)
%     theta_2: angle about x-axis (in deg)
%       psi_3: angle about x-axis (in deg)
%
% OUTPUTS:
%          q0: initial quaternion

    % Angles to radians
    phi = deg2rad(phi_1);
    theta = deg2rad(theta_2);
    psi = deg2rad(psi_3);

    % Compute the initial DCM
    A1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    A2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    A3 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    A = A3*A2*A1;

    % Calculate quaternions from DCM
    q4 = 0.5*sqrt(1 + trace(A));
    q1 = 0.25*(A(2,3) - A(3,2))/q4;
    q2 = 0.25*(A(3,1) - A(1,3))/q4;
    q3 = 0.25*(A(1,2) - A(2,1))/q4;
    q0 = [q1; q2; q3; q4];
end