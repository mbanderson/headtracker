function [q,A] = QuatIntegration(q0,w,dt)
%QUATINTEGRATION Performs numerical integration of input quaternion.
%   The subsequent quaternion resulting from angular velocity w given by:
%       q_{t+1} = q_t + Aq_tdt
%   INPUTS:
%       q0 - (4x1 float vec) starting quaternion
%       w - (3x1 float vec) angular velocity
%       dt - (float) time discretization step
%   OUTPUTS:
%       q - (4x1 float vec) resulting quaternion
%       A - (4x4) quaternion dynamics matrix
%

omega = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
A = 1/2*[0, -w';
         w, omega];
q = (eye(4) - A*dt)*q0;
q = q / norm(q);

end
