function [q] = QuatIntegration(q0,w,dt)
%QUATINTEGRATION Performs numerical integration of input quaternion.
%   The subsequent quaternion resulting from angular velocity w given by:
%       q_{t+1} = q_t + Aq_tdt
%   INPUTS:
%       q0 - (4x1 float vec) starting quaternion
%       w - (3x1 float vec) angular velocity
%       dt - (float) time discretization step
%   OUTPUTS:
%       q - (4x1 float vec) resulting quaternion
%

omega = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
A = 1/2*[0, -w';
         w, omega];
q = q0 + A*q0*dt;
q = q / norm(q);

end

function xrot = vec_rot(x,q)
    % Rotate (3x1 vector) by quaternion.
    % INPUTS:
    %   x - (3x1) input vector
    %   q - (4x1) quaternion by which to rotate vector
    % OUTPUTS:
    %   xrot - (3x1) rotated vector
    %
    R = [1-2*q(3)^2-2*q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)), 2*(q(2)*q(4)-q(1)*q(3));
         2*(q(2)*q(3)-q(1)*q(4)), 1-2*q(2)^2-2*q(4)^2, 2*(q(3)*q(4)+q(1)*q(2));
         2*(q(2)*q(4)+q(1)*q(3)), 2*(q(3)*q(4)-q(1)*q(2)), 1-2*q(2)^2-2*q(3)^2];
    xrot = R*x; 
end
