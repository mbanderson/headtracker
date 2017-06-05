function [w_handle,wdot_handle] = AngVelModel(T,amp)
%ANGVELMODEL Represents angular velocity as sine wave.
%   INPUTS:
%       T - (float) sine wave period
%       amp - (float) sine wave amplitude, RAD/S
%   OUTPUTS:
%       w_handle - (ftn handle) angular velocity f(t)
%       wdot_handle - (ftn handle) angular acceleration f(t)
%

w_handle = @(t) amp*sin(2*pi/T .* t);
wdot_handle = @(t) 2*pi/T*amp*cos(2*pi/T .* t);

end
