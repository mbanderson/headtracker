function [ys] = AngVelModel(T,ysat,ts)
%ANGVELMODEL Represents angular velocity as a saturated sine wave.
%   INPUTS:
%       T - (float) sine wave period
%       ysat - (float) maximum angular rate, RAD/S
%       ts - (Nx1 float vec) time domain at which to evaluate function
%   OUTPUTS:
%       ys - (Nx1 float vec) function values on input time domain
%

% Generate unsaturated sine wave, then apply saturation
unsat = sin(2*pi/T .* ts);
satftn = @(val) (ysat*sign(val))*(abs(val) > ysat) + ...
                val*(abs(val) <= ysat);
ys = arrayfun(satftn, unsat);

end
