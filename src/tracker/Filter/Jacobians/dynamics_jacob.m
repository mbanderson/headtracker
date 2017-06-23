function [A] = dynamics_jacob(w,dt)
%DYNAMICS_JACOB Generates Jacobian for quaternion integration dynamics.
%   INPUTS:
%       w  - (3x1) angular velocity
%       dt - (scalar) timestep
%   OUTPUTS:
%       A - (4x4) dynamics Jacobian for quaternion integration
%

A = dt*[1/dt,     -1/2*w(1), -1/2*w(2), -1/2*w(3);
        1/2*w(1),  1/dt,     -1/2*w(3),  1/2*w(2);
        1/2*w(2),  1/2*w(3),  1/dt,     -1/2*w(1);
        1/2*w(3), -1/2*w(2),  1/2*w(1),  1/dt];
    
end
